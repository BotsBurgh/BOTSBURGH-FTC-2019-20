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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Autonomous(name="Gyroscope Turning Test")
public class TestGyroTurn extends LinearOpMode {
    // Declare OpMode Members
    private ElapsedTime runtime = new ElapsedTime();

    private Robot robot;

    @Override
    public void runOpMode() {
        DcMotor sc = hardwareMap.get(DcMotor.class, "scissorLift"); // Scissor lift
        DcMotor lb = hardwareMap.get(DcMotor.class, "lb"); // Left back motor
        DcMotor rb = hardwareMap.get(DcMotor.class, "rb"); // Right back motor

        // Because motors are in opposite directions, we have to reverse one motor.
        lb.setDirection(DcMotor.Direction.REVERSE); // Left motor is set to move in the reverse direction
        rb.setDirection(DcMotor.Direction.FORWARD); // Right motor is set to move in the forward direction

        // We want both motors to shut off power, so we can completely stop the robot when we command the robot to stop.
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // Power to left motor will be reset to zero
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // Power to right motor will be reset to zero

        // Initialize color sensors
        ColorSensor[] colorSensors = new ColorSensor[] {
                hardwareMap.get(ColorSensor.class, "scissorDownLimit"),
                hardwareMap.get(ColorSensor.class, "scissorUpLimit")
        };

        // Initialize Web Cam
        WebcamName[] webcams = new WebcamName[] {
                hardwareMap.get(WebcamName.class, "Webcam 1")
        };

        // Initialize Gyros
        BNO055IMU[] gyros = new BNO055IMU[] {
                hardwareMap.get(BNO055IMU.class, "imu"),
                hardwareMap.get(BNO055IMU.class, "imu 1")
        };

        // Initializes the scissor lift mechanism, left back motor, and right back motor
        DcMotor[] motors = new DcMotor[] {
                sc,
                null, null, // Because we don't have front motors
                lb, rb
        };

        // Initialize sensor class
        Sensor sensor = new Sensor
                .SensorBuilder()
                .colorSensors(colorSensors)
                .webcams(webcams)
                .gyros(gyros)
                .build();

        // Initializes movement class
        Movement movement = new Movement
                .MovementBuilder()
                .motors(motors)
                .build();

        // Initializes the robot object
        Robot robot = new Robot.RobotBuilder()
                .sensor(sensor)
                .movement(movement)
                .linearOpMode(TestGyroTurn.this)
                .build();
        // Initialize gyros
        robot.getSensor().calibrateGyro(0);
        robot.getSensor().calibrateGyro(1);
        robot.getSensor().initGyro(0);
        robot.getSensor().initGyro(1);

        double offset = sensor.getGyros()[0].getAngularOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.ZYX,
                AngleUnit.DEGREES
        ).firstAngle;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            robot.gyroTurn(0, 0.5, 0 + offset);
            sleep(5000);
            robot.gyroTurn(0, 0.5, 90 + offset);
            sleep(5000);
            robot.gyroTurn(0, 0.5, 180 + offset);
            sleep(5000);
            robot.gyroTurn(0, 0.5, 270 + offset);
            sleep(5000);
        }
    }
}
