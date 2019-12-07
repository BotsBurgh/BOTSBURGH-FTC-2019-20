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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.ClassFactory;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.lang.reflect.Array;
import java.util.ArrayList;


@Autonomous(name="Autonomous Blue", group="Autonomous")
public class AutonomousBlue extends LinearOpMode {
    // Declare OpMode Members
    private ElapsedTime runtime = new ElapsedTime();

    private static final double DRIVE_SPEED = 0.7;
    private static final double TURN_SPEED = 0.5;

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

        // Initializes the scissor lift mechanism, left back motor, and right back motor
        DcMotor[] motors = new DcMotor[] {
                sc,
                null, null, // Because we don't have front motors
                lb, rb
        };

        // Initialize Gyros
        BNO055IMU[] gyros = new BNO055IMU[] {
                hardwareMap.get(BNO055IMU.class, "imu"),
                hardwareMap.get(BNO055IMU.class, "imu 1")
        };

        // Initialize sensor class
        Sensor sensor = new Sensor
                .SensorBuilder()
                .withColorSensors(colorSensors)
                .withGyros(gyros)
                .withWebcams(webcams)
                .build();

        // Initializes movement class
        Movement movement = new Movement
                .MovementBuilder()
                .withMotors(motors)
                .build();

        // Initializes the robot object
        Robot robot = new Robot(sensor, movement);

        VectorF target;

        /*
        // Initialize VuForia
        robot.sensor.initVuforia(hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()), 0
        );

        // Check if we can use TFOD. If we can, initialize it.
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            robot.sensor.initTfod(hardwareMap.appContext.getResources().getIdentifier(
                    "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName())
            );
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

         */

        robot.sensor.initGyro(0);
        robot.sensor.initGyro(1);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Here goes.

            /*
            * Robot faces the track
            * Using vuforia, robot approaches black box
            * Using the scissor lift mechanism, it'll grab the black box
            * Robot backs up by about 1.5 feet
            * Robot rotates until gyro detects 90º
            * Robot approaches (3, -1)
            * Robot turns back until gyro detects 0º
            * Robot approaches (-4. -1)
            * Robot turns until gyro detects 135º
            * Robot approaches (-1, 3)
            * Robot drops black box on foundation
            * Robot rotates until gyro detects 150º and pushes foundation along
            * Robot pushes foundation to triangle slot
            * Robot rotates itself and foundation until gro detects 360º
            * Robot moves to (-5, -1)
            */

            /*
            robot.gyroDrive(0, DRIVE_SPEED, 24, robot.sensor.getGyro(0).getAngularOrientation().firstAngle);
            ArrayList<ArrayList<Float>> pos = new ArrayList<>();
            pos = robot.sensor.getTfod();
            double distance = ((pos.get(0).get(0))+(pos.get(0).get(3)))/2;
            double turn = Math.acos(3.5/distance);
            robot.gyroTurn(0, TURN_SPEED, turn);
            robot.gyroDrive(0, DRIVE_SPEED, distance, robot.sensor.getGyro(0).getAngularOrientation().firstAngle);
            // TODO - Grabbing the black box
            robot.gyroTurn(0, TURN_SPEED, 90-turn);
            robot.gyroDrive(0, DRIVE_SPEED, 18, robot.sensor.getGyro(0).getAngularOrientation().firstAngle);
            robot.gyroTurn(0, TURN_SPEED, 135); // Robot turns to 135º
            robot.gyroDrive(0, DRIVE_SPEED, 68, robot.sensor.getGyro(0).getAngularOrientation().firstAngle); // Robot nears the opponent team's bridge
            robot.gyroTurn(0, TURN_SPEED, 69);
            robot.gyroDrive(0, DRIVE_SPEED, 87, robot.sensor.getGyro(0).getAngularOrientation().firstAngle);
            // TODO - Dropping black box on to the foundation
            robot.gyroTurn(0, TURN_SPEED, 145);
            robot.gyroDrive(0, DRIVE_SPEED, 87, robot.sensor.getGyro(0).getAngularOrientation().firstAngle);
            robot.gyroTurn(0,TURN_SPEED, -145);
            robot.gyroDrive(0, DRIVE_SPEED, 70, robot.sensor.getGyro(0).getAngularOrientation().firstAngle);
             */

            robot.gyroDrive(0, DRIVE_SPEED, 96, robot.sensor.getGyro(0).getAngularOrientation().firstAngle); // Robot goes across the field
            robot.gyroTurn(0, TURN_SPEED, 90); // Turn 90 degrees so we are facing the
            robot.gyroDrive(0, DRIVE_SPEED, 24, robot.sensor.getGyro(0).getAngularOrientation().firstAngle);


            telemetry.addData("Status", "Run Time: " + runtime.toString());
        }
    }
}
