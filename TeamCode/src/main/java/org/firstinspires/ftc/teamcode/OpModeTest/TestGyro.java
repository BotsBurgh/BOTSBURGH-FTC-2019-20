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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Api.Config.InitRobot;
import org.firstinspires.ftc.teamcode.Api.Config.Naming;
import org.firstinspires.ftc.teamcode.Api.Robot;

@TeleOp(name="Gyroscope Test", group="20-Test")
public class TestGyro extends LinearOpMode {
    // Declare OpMode Members
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        InitRobot initializer = new InitRobot(TestGyro.this);
        Robot robot = initializer.init();

        // Initialize Gyroscope
        robot.getSensor().initGyro(Naming.GYRO_0_NAME);
        robot.getSensor().initGyro(Naming.GYRO_1_NAME);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Do stuff here
            telemetry.addData("Gyroscope X", robot.getSensor().getGyro(Naming.GYRO_0_NAME).getAngularOrientation().thirdAngle);
            telemetry.addData("Gyroscope Y", robot.getSensor().getGyro(Naming.GYRO_0_NAME).getAngularOrientation().secondAngle);
            telemetry.addData("Gyroscope Z", robot.getSensor().getGyro(Naming.GYRO_0_NAME).getAngularOrientation().firstAngle);
            telemetry.update();
        }
    }
}
