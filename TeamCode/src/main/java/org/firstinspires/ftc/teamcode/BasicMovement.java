/*
 * Copyright 2019 FIRST Tech Challenge Team 11792
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
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Basic Movement", group="00-TeleOp")
public class BasicMovement extends LinearOpMode {

    private static final double DEADZONE = 0.05; // Controls controller joystick deadzone

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

    private long count = 0;

    @Override
    public void runOpMode() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;

        double mod;

        DcMotor sc = hardwareMap.get(DcMotor.class, "scissorLift");
        DcMotor lb = hardwareMap.get(DcMotor.class, "lb");
        DcMotor rb = hardwareMap.get(DcMotor.class, "rb");

        DcMotor[] motors = new DcMotor[] {
                sc,
                null, null, // Because we don't have front motors
                lb, rb
        };

        Servo armSwivel = hardwareMap.get(Servo.class, "armSwivel");
        Servo grabber = hardwareMap.get(Servo.class, "grabber");


        Servo[] servos = new Servo[] {
                armSwivel, grabber
        };

        Movement base = new Movement
                .MovementBuilder()
                .motors(motors)
                .servos(servos)
                .build();

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        sc.setDirection(DcMotor.Direction.FORWARD);
        lb.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.FORWARD);

        sc.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ColorSensor[] colorSensors = new ColorSensor[] {
                hardwareMap.get(ColorSensor.class, "scissorDownLimit"),
                hardwareMap.get(ColorSensor.class, "scissorUpLimit")
        };

        Sensor sensors = new Sensor
            .SensorBuilder()
            .colorSensors(colorSensors)
            .build();

        Robot robot = new Robot(sensors, base);

        double elevatorSpeed;

        double sul = 3;
        double sud = 3;

        //lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        robot.movement.setServo(0, 0.05);
        robot.movement.setServo(1, 1);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
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
            double turn  = gamepad1.right_stick_x;
            leftPower    = Range.clip(drive + turn, -mod, mod);
            rightPower   = Range.clip(drive - turn, -mod, mod);
            
            // Every tenth loop, check the elevator status.
            if (count % 10 == 0) {
                sul = robot.sensor.getRGB(0);
                sud = robot.sensor.getRGB(1);
            }

            // Check if the limit switch is hit either way, and set the movable direction.
            if ((sul == 0) && (gamepad2.left_stick_y < DEADZONE)) {
                // If we cannot go up, and the user tries to go up, we don't allow that to happen.
                elevatorSpeed = 0;
            } else if ((sud == 0) && (gamepad2.left_stick_y > DEADZONE)) {
                // If we cannot go down, and the user tries to go down, we don't allow that to happen.
                elevatorSpeed = 0;
            } else if (Math.abs(gamepad2.left_stick_y) > DEADZONE) {
                // If the user moves the stick more than 10%, and none of the other conditions are fulfilled, we allow the scissor lift to move
                elevatorSpeed = -gamepad2.left_stick_y;
            } else {
                // If the user is not doing anything, we don't allow the scissor lift to move
                elevatorSpeed = 0;
            }

            if (gamepad2.x) {
                robot.movement.setServo(0, 0.81);
            }
            if (gamepad2.y) {
                robot.movement.setServo(0, 0.05);
            }

            if (gamepad2.a) {
                robot.movement.setServo(1, 0.3);
            }
            if (gamepad2.b) {
                robot.movement.setServo(1, 0.63);
            }

            // Send calculated power to wheels
            robot.movement.move2x2(leftPower, rightPower);
            robot.movement.moveElevator(elevatorSpeed);

            // Display the current value(s)
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motor Power", "%5.2f", elevatorSpeed);
            telemetry.addData("Up Limit", sul);
            telemetry.addData("Down Limit", sud);
            telemetry.addData("Arm Position", robot.movement.getServos()[0].getPosition());
            telemetry.addData("Grabber Position", robot.movement.getServos()[1].getPosition());
            // Show the elapsed game time and wheel power.
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
            count++;
        }
    }
}
