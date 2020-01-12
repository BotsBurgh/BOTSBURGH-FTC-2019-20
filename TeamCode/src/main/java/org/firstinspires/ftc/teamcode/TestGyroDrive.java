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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

@Autonomous(name="Gyroscope Driving Test")
public class TestGyroDrive extends LinearOpMode {
    // Declare OpMode Members
    private ElapsedTime runtime = new ElapsedTime();

    Robot robot;
    private static final double DRIVE_SPEED = 0.5;


    @Override
    public void runOpMode() {
        DcMotor sc = hardwareMap.get(DcMotor.class, "scissorLift"); // Scissor lift
        DcMotor lb = hardwareMap.get(DcMotor.class, "lb"); // Left back motor
        DcMotor rb = hardwareMap.get(DcMotor.class, "rb"); // Right back motor

        // Because motors are in opposite directions, we have to reverse one motor.
        lb.setDirection(DcMotor.Direction.FORWARD); // Left motor is set to move in the reverse direction
        rb.setDirection(DcMotor.Direction.REVERSE); // Right motor is set to move in the forward direction

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
                .linearOpMode(TestGyroDrive.this)
                .build();
        
        // Initialize gyros
        robot.getSensor().calibrateGyro(0);
        robot.getSensor().calibrateGyro(1);
        robot.getSensor().initGyro(0);
        robot.getSensor().initGyro(1);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        robot.gyroDrive(0, DRIVE_SPEED,18, robot.getSensor().getGyros()[0].getAngularOrientation().firstAngle, true);
    }
}
