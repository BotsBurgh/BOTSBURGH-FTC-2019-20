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
    private static final double ELEVATOR_THRESH = 0.75;
    private static final int CYCLE_MS = 50;
    private double elevatorSpeed;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor scissorLift;
    private Sensor scissorDownLimit, scissorUpLimit; // UpLimit prevents the scissor lift from going
                                                    // up, and DownLimit is the opposite.
    private double sul, sud;
    @Override
    public void runOpMode() {
        scissorLift = hardwareMap.get(DcMotor.class, "scissorLift");
        scissorDownLimit = new Sensor(hardwareMap.get(ColorSensor.class, "scissorDownLimit"));
        scissorUpLimit = new Sensor(hardwareMap.get(ColorSensor.class, "scissorUpLimit"));

        elevatorSpeed = 0;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            sul = scissorUpLimit.getRGB();
            sud = scissorDownLimit.getRGB();
            // Check if the limit switch is hit either way, and set the movable direction.
            if ((sul == 0) && (gamepad2.left_stick_y < 0.1)) {
                // If we cannot go up, and the user tries to go up, we don't allow that to happen.
                elevatorSpeed = 0;
            } else if ((sud == 0) && (gamepad2.left_stick_y > 0.1)) {
                // If we cannot go down, and the user tries to go down, we don't allow that to happen.
                elevatorSpeed = 0;
            } else if (Math.abs(gamepad2.left_stick_y) > 0.1){
                // If the user moves the stick more than 10%, and none of the other conditions are fulfilled, we allow the scissor lift to move
                elevatorSpeed = gamepad2.left_stick_y;
            } else {
                // If the user is not doing anything, we don't allow the scissor lift to move
                elevatorSpeed = 0;
            }

            // Display the current value
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motor Power", "%5.2f", elevatorSpeed * ELEVATOR_THRESH);
            telemetry.addData("Up Limit", sul);
            //telemetry.addData("Up Red", scissorUpLimit.getRed());
            telemetry.addData("Down Limit", sud);
            //telemetry.addData("Down Red", scissorDownLimit.getRed());
            telemetry.addData(">", "Press Stop to end test." );
            telemetry.update();

            // Set the motor to the new power and pause;
            scissorLift.setPower(elevatorSpeed * ELEVATOR_THRESH);
            sleep(CYCLE_MS);
            idle();
        }
        scissorLift.setPower(0);

        telemetry.addData(">", "Done");
        telemetry.update();
    }
}
