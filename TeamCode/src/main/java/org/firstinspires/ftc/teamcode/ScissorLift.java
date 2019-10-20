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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Scissor Lift")
public class ScissorLift extends LinearOpMode {
    // Declare variables
    private static final double ELEVATOR_THRESH = 0.6;
    private static final int CYCLE_MS = 50;
    private double elevator_speed;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor scissorlift;
    private Sensor scissorcolor;
    private boolean direction, switched; // True for up, false for down

    @Override
    public void runOpMode() {
        scissorlift = hardwareMap.get(DcMotor.class, "scissorlift");
        scissorcolor = new Sensor(hardwareMap.get(ColorSensor.class, "scissorcolor"));

        direction = true; // Because we start at the bottom (without the lift extended)
        switched = false;
        elevator_speed = 0;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (scissorcolor.getRGB() == 0) {
                if (direction == true && !switched) {
                    direction = false; switched = true;
                } else if (direction == false && !switched) {
                    direction = true; switched = true;
                }

                telemetry.addData("Can Go UP", direction == true);
            } else {
                switched = false;
            }

            if ((gamepad2.left_stick_y > 0 && direction == false) || gamepad2.left_stick_y < 0 && direction == true) {
                elevator_speed = 0;
            } else {
                elevator_speed = gamepad2.left_stick_y;
            }

            // Let User manually change direction
            if (gamepad2.a) {
                direction = true;
            } else if (gamepad2.b) {
                direction = false;
            }

            // Display the current value
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motor Power", "%5.2f", elevator_speed * ELEVATOR_THRESH);
            telemetry.addData("Direction", direction);
            telemetry.addData(">", "Press Stop to end test." );
            telemetry.update();

            // Set the motor to the new power and pause;
            scissorlift.setPower(elevator_speed * ELEVATOR_THRESH);
            sleep(CYCLE_MS);
            idle();
        }
        scissorlift.setPower(0);

        telemetry.addData(">", "Done");
        telemetry.update();
    }
}
