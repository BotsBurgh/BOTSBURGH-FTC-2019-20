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
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import android.os.AsyncTask;

@TeleOp(name="Basic Movement", group="00-TeleOp")
public class BasicMovement extends LinearOpMode {

    private static final double DEADZONE    = 0.05; // Controls controller joystick deadzone
    private static final double SERVO_POWER = 1.00;

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

    private int sul, sdl = 0;

    @Override
    public void runOpMode() {
        InitRobot initializer = new InitRobot(BasicMovement.this, false);
        Robot robot = initializer.init();

        // Zero some servos
        robot.getMovement().setServo(Naming.SERVO_FOUNDATION_LEFT_NAME, 0); // Foundation
        robot.getMovement().setServo(Naming.SERVO_FOUNDATION_RIGHT_NAME, 180); // More foundation

        // Zeroing Swivel (Rotate)
        robot.getMovement().setServo("rotate", 1); // Since this is a digital servo, it is initialized to 1.

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
            double extenderPower;
            if (gamepad2.dpad_up) {
                extenderPower = SERVO_POWER;
            } else if (gamepad2.dpad_down) {
                extenderPower = -SERVO_POWER;
            } else {
                extenderPower = 0;
            }
            robot.getMovement().getCRServo(Naming.CRSERVO_EXTEND_NAME).setPower(extenderPower);

            // Grabber servo
            if (gamepad2.a) {
                // Open and close grabber
                robot.getMovement().openGrabber(true);
            } else if (gamepad2.b) {
                robot.getMovement().openGrabber(false);
            }

            // Arm tilt servo
            if (gamepad2.x) {
                robot.getMovement().openSwivel(true);
            } else if (gamepad2.y) {
                robot.getMovement().openSwivel(false);
            }

            // Foundation servos
            if (gamepad1.x) {
                robot.getMovement().grabFoundation(true);
            } else if (gamepad1.y) {
                robot.getMovement().grabFoundation(false);
            }

            // Display the current value(s)
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motor Power", "%5.2f", elevatorSpeed);
            telemetry.addData("Up Limit", sul);
            telemetry.addData("Down Limit", sdl);
            telemetry.addData("Arm Extend", robot.getMovement().getCRServo(Naming.CRSERVO_EXTEND_NAME).getPower());
            telemetry.addData("Grabber Position", robot.getMovement().getServo(Naming.SERVO_GRABBER_NAME).getPosition());
            telemetry.addData("Rotation Position", robot.getMovement().getServo(Naming.SERVO_ROTATE_NAME).getPosition());
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
                double turn  = gamepad1.right_stick_x;
                leftPower  = Range.clip(drive + turn, -mod, mod);
                rightPower = Range.clip(drive - turn, -mod, mod);

                if (InitRobot.MODE_4x4) {
                    params[0].getMovement().move2x4(leftPower, rightPower);
                } else {
                    params[0].getMovement().move2x2(leftPower, rightPower);
                }
            }
            return "";
        }
    }

    private class AsyncColorSensor extends AsyncTask<Robot, String, String> {
        @Override
        protected String doInBackground(Robot... params) {
            while (opModeIsActive()) {
                sdl = params[0].getSensor().getRGB(Naming.COLOR_SENSOR_DOWN_LIMIT_NAME);
                sul = params[0].getSensor().getRGB(Naming.COLOR_SENSOR_UP_LIMIT_NAME);
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
