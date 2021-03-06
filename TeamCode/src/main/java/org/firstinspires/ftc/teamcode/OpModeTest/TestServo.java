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

package org.firstinspires.ftc.teamcode.OpModeTest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="ServoTester", group="20-Test")
public class TestServo extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Servo s1 = hardwareMap.get(Servo.class, "s1");
        Servo s2 = hardwareMap.get(Servo.class, "s2");
        Servo s3 = hardwareMap.get(Servo.class, "s3");
        Servo s4 = hardwareMap.get(Servo.class, "s4");
        Servo s5 = hardwareMap.get(Servo.class, "s5");
        Servo s6 = hardwareMap.get(Servo.class, "s6");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while(opModeIsActive()) {
            if (gamepad1.x) {
                // s1.setPosition(90);
                s2.setPosition(90);
                s3.setPosition(90);
                s4.setPosition(90);
                s5.setPosition(90);
                s6.setPosition(90);

                sleep(1000);
                // s1.setPosition(0);
                s2.setPosition(0);
                s3.setPosition(0);
                s4.setPosition(0);
                s5.setPosition(0);
                s6.setPosition(0);
            }
        }
    }
}
