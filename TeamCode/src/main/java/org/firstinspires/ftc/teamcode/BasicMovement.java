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
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Basic Movement")
public class BasicMovement extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor lf, lb, rf, rb;
    Movement base;
    // Declare variables
    private double elevatorSpeed;
    private DcMotor scissorLift;
    private Sensor scissorDownLimit, scissorUpLimit; // UpLimit prevents the scissor lift from going
                                                     // up, and DownLimit is the opposite.
    private double sul, sud; // Sensor up limit and down limit. Stores color values from sensors

    private long count = 0;

    @Override
    public void runOpMode() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        lb = hardwareMap.get(DcMotor.class, "lb");
        rb = hardwareMap.get(DcMotor.class, "rb");
        scissorLift = hardwareMap.get(DcMotor.class, "scissorLift");

        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;

        double mod;

        //base = new Movement(lf, rf, lb, rb);
        base = new Movement(lb, rb, scissorLift);

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        lb.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.FORWARD);

        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        scissorDownLimit = new Sensor(hardwareMap.get(ColorSensor.class, "scissorDownLimit"));
        scissorUpLimit   = new Sensor(hardwareMap.get(ColorSensor.class, "scissorUpLimit"));

        elevatorSpeed = 0;

        sul = 3;
        sud = 3;

        //lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Adjust speed based on the bumpers. Idea from Robotic Doges
            if (gamepad1.left_bumper) {
                mod = 1;
            } else if (gamepad1.right_bumper) {
                mod = 0.33;
            } else {
                mod = 0.66;
            }

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;
            leftPower    = Range.clip(drive + turn, -mod, mod) ;
            rightPower   = Range.clip(drive - turn, -mod, mod) ;

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;

            if (count % 10 == 0) {
                sul = scissorUpLimit.getRGB();
                sud = scissorDownLimit.getRGB();
            }

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

            // Send calculated power to wheels
            base.move2x2(leftPower, rightPower);
            base.moveElevator(elevatorSpeed);

            // Display the current value(s)
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motor Power", "%5.2f", elevatorSpeed);
            telemetry.addData("Up Limit", sul);
            telemetry.addData("Down Limit", sud);
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
            count++;
        }
    }
}
