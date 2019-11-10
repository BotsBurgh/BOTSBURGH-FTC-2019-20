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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Mecanum Drive", group = "Linear OpMode")
public class Mecanum extends LinearOpMode {
    static final int    CYCLE_MS = 50;
    private ElapsedTime runtime  = new ElapsedTime();
    private DcMotor motorFL, motorFR, motorBL, motorBR;

    @Override
    public void runOpMode() {

        // Init Motors
        motorFL = hardwareMap.get(DcMotor.class, "fl");
        motorFR = hardwareMap.get(DcMotor.class, "fr");
        motorBL = hardwareMap.get(DcMotor.class, "bl");
        motorBR = hardwareMap.get(DcMotor.class, "br");

        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorBL.setDirection(DcMotor.Direction.REVERSE);

        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Movement movement = new Movement(motorFL, motorFR, motorBL, motorBR);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        telemetry.addData(">", "Press start");
        telemetry.update();

        waitForStart();

        telemetry.addData(">", "Press stop");
        telemetry.update();

        waitForStart();
        while(opModeIsActive()) {

            double x1 = gamepad1.left_stick_x;
            double y1 = gamepad1.left_stick_y;
            double rotation = gamepad1.right_stick_x;

            double flPower = Range.clip((y1 - x1 - rotation), -.8, .8);
            double frPower = Range.clip((y1 + x1 + rotation), -.8, .8);
            double blPower = Range.clip((y1 + x1 - rotation), -.8, .8);
            double brPower = Range.clip((y1 - x1 + rotation), -.8, .8);

            movement.move4x4(flPower, frPower, blPower, brPower);

            telemetry.addData("Back Left", motorBL.getCurrentPosition());
            telemetry.addData("Back Right", motorBR.getCurrentPosition());
            telemetry.addData("Front Left", motorFL.getCurrentPosition());
            telemetry.addData("Front right", motorFR.getCurrentPosition());
            telemetry.update();
        }
    }
}
