/*
 * Copyright 2020 FIRST Tech Challenge Team 11792
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
 * associated documentation files (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge, publish, distribute,
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all copies or substantial
 * portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
 * NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import android.os.AsyncTask;

@TeleOp(name="Basic Movement", group="00-TeleOp")
public class BasicMovement extends LinearOpMode {

    private static final double DEADZONE    = 0.05; // Controls controller joystick deadzone
    private static final double SERVO_POWER = 1.00;
    private static final double SERVO_STEP  = 0.01;

    /*
    ######  #######    #     # ####### #######    ####### ######  ### #######
    #     # #     #    ##    # #     #    #       #       #     #  #     #
    #     # #     #    # #   # #     #    #       #       #     #  #     #
    #     # #     #    #  #  # #     #    #       #####   #     #  #     #
    #     # #     #    #   # # #     #    #       #       #     #  #     #
    #     # #     #    #    ## #     #    #       #       #     #  #     #
    ######  #######    #     # #######    #       ####### ######  ###    #

    ######  ####### #       ####### #     #    ####### #     # ###  #####
    #     # #       #       #     # #  #  #       #    #     #  #  #     #
    #     # #       #       #     # #  #  #       #    #     #  #  #
    ######  #####   #       #     # #  #  #       #    #######  #   #####
    #     # #       #       #     # #  #  #       #    #     #  #        #
    #     # #       #       #     # #  #  #       #    #     #  #  #     #
    ######  ####### ####### #######  ## ##        #    #     # ###  #####

    #       ### #     # #######
    #        #  ##    # #
    #        #  # #   # #
    #        #  #  #  # #####
    #        #  #   # # #
    #        #  #    ## #
    ####### ### #     # #######
    (Unless if you know what you are doing)
     */

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    // Setup a variable for each drive wheel to save power level for telemetry
    private double leftPower;
    private double rightPower;

    private double elevatorSpeed;

    private double extenderPower;

    private int sul, sdl = 0;

    public Robot robot;

    public Sensor sensor;

    @Override
    public void runOpMode() {
        // Get motors
        DcMotor sc = hardwareMap.get(DcMotor.class, "scissorLift");
        DcMotor lb = hardwareMap.get(DcMotor.class, "lb");
        DcMotor rb = hardwareMap.get(DcMotor.class, "rb");

        // Add motors into the list
        DcMotor[] motors = new DcMotor[] {
                sc,
                null, null, // Because we don't have front motors
                lb, rb
        };

        // Get servos
        Servo grabber = hardwareMap.get(Servo.class, "grabber");
        Servo rotate = hardwareMap.get(Servo.class, "rotate");

        // Add servos into the list
        Servo[] servos = new Servo[] {
                grabber,
                rotate
        };

        // Get CRServos
        CRServo armExtend = hardwareMap.get(CRServo.class, "extender");

        // Add CRServos into the list
        CRServo[] crServos = new CRServo[] {
                armExtend
        };

        // Add lists into the movement class
        Movement movement = new Movement
                .MovementBuilder()
                .motors(motors)
                .servos(servos)
                .crServos(crServos)
                .build();

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        sc.setDirection(DcMotor.Direction.FORWARD);
        lb.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.FORWARD);

        // Set motors to spin in the correct direction
        sc.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Switch direction of servo
        rotate.setDirection(Servo.Direction.REVERSE);

        // Get color sensors
        ColorSensor scissorDownLimit = hardwareMap.get(ColorSensor.class, "scissorDownLimit");
        ColorSensor scissorUpLimit = hardwareMap.get(ColorSensor.class, "scissorUpLimit");

        // Add color sensors into list
        ColorSensor[] colorSensors = new ColorSensor[] {
                scissorDownLimit,
                scissorUpLimit
        };

        // Add lists into sensor class
        Sensor sensor = new Sensor
                .SensorBuilder()
                .colorSensors(colorSensors)
                .build();

        // Add movement and sensor class into robot class
        Robot robot = new Robot.RobotBuilder()
                .sensor(sensor)
                .movement(movement)
                .linearOpMode(BasicMovement.this)
                .build();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        new AsyncColorSensor().executeOnExecutor(AsyncTask.THREAD_POOL_EXECUTOR, robot);

        new AsyncBase().executeOnExecutor(AsyncTask.THREAD_POOL_EXECUTOR, robot);

        new AsyncElevatorSpeed().executeOnExecutor(AsyncTask.THREAD_POOL_EXECUTOR, robot);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Arm extender
            if (gamepad2.dpad_up) {
                extenderPower = SERVO_POWER;
            } else if (gamepad2.dpad_down) {
                extenderPower = -SERVO_POWER;
            } else {
                extenderPower = 0;
            }
            robot.getMovement().getCrServos()[0].setPower(extenderPower);

            // Grabber
            if (gamepad2.a) {
                // Open and close grabber
                robot.getMovement().grab(true);
            } else if (gamepad2.b) {
                robot.getMovement().grab(false);
            }

            if (gamepad2.x) {
                robot.getMovement().setServo(1, robot.getMovement().getServos()[1].getPosition() + SERVO_STEP);
            } else if (gamepad2.y) {
                robot.getMovement().setServo(1, robot.getMovement().getServos()[1].getPosition() - SERVO_STEP);
            }

            // Display the current value(s)
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motor Power", "%5.2f", elevatorSpeed);
            telemetry.addData("Up Limit", sul);
            telemetry.addData("Down Limit", sdl);
            telemetry.addData("Arm Extend", robot.getMovement().getCrServos()[0].getPower());
            telemetry.addData("Grabber Position", robot.getMovement().getServos()[0].getPosition());
            telemetry.addData("Rotation Position", robot.getMovement().getServos()[1].getPosition());
            // Show the elapsed game time and wheel power.
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        }
    }

    private class AsyncBase extends AsyncTask<Robot, String, String> {
        @Override
        protected String doInBackground(Robot... params) {
            double mod;
            while (opModeIsActive()) {
                // Adjust speed based on the bumpers. Idea from Robotic Doges
                if (gamepad1.left_bumper || gamepad2.left_bumper) {
                    mod = 0.33;
                } else if (gamepad1.right_bumper || gamepad2.right_bumper) {
                    mod = 1;
                } else {
                    mod = 0.66;
                }

                // POV Mode uses left stick to go forward, and right stick to turn.
                // - This uses basic math to combine motions and is easier to drive straight.
                double drive = gamepad1.left_stick_y;
                double turn = gamepad1.right_stick_x;
                leftPower = Range.clip(drive + turn, -mod, mod);
                rightPower = Range.clip(drive - turn, -mod, mod);

                params[0].getMovement().move2x2(leftPower, rightPower);
            }
            return "";
        }
    }

    private class AsyncColorSensor extends AsyncTask<Robot, String, String> {
        @Override
        protected String doInBackground(Robot... params) {
            while (opModeIsActive()) {
                sdl = params[0].getSensor().getRGB(0);
                sul = params[0].getSensor().getRGB(1);
            }
            return "";
        }
    }

    private class AsyncElevatorSpeed extends AsyncTask<Robot, String, String> {
        @Override
        protected String doInBackground(Robot... params) {
            while (opModeIsActive()) {
                // Check if the limit switch is hit either way, and set the movable direction.
                if ((sul == 0) && (gamepad2.left_stick_y < DEADZONE)) {
                    // If we cannot go up, and the user tries to go up, we don't allow that to happen.
                    elevatorSpeed = 0;
                } else if ((sdl == 0) && (gamepad2.left_stick_y > DEADZONE)) {
                    // If we cannot go down, and the user tries to go down, we don't allow that to happen.
                    elevatorSpeed = 0;
                } else if (Math.abs(gamepad2.left_stick_y) > DEADZONE) {
                    // If the user moves the stick more than 10%, and none of the other conditions are fulfilled, we allow the scissor lift to move
                    elevatorSpeed = gamepad2.left_stick_y;
                } else {
                    // If the user is not doing anything, we don't allow the scissor lift to move
                    elevatorSpeed = 0;
                }
                params[0].getMovement().moveElevator(elevatorSpeed);
            }
            return "";
        }
    }
}
