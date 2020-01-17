package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

public class InitRobot {
    LinearOpMode l;

    InitRobot(LinearOpMode l) {
        this.l = l;
    }

    Robot init() {
        DcMotor sc = l.hardwareMap.get(DcMotor.class, "scissorLift"); // Scissor lift
        DcMotor lb = l.hardwareMap.get(DcMotor.class, "lb"); // Left back motor
        DcMotor rb = l.hardwareMap.get(DcMotor.class, "rb"); // Right back motor

        // Because motors are in opposite directions, we have to reverse one motor.
        lb.setDirection(DcMotor.Direction.REVERSE); // Left motor is set to move in the reverse direction
        rb.setDirection(DcMotor.Direction.FORWARD); // Right motor is set to move in the forward direction

        // We want both motors to shut off power, so we can completely stop the robot when we command the robot to stop.
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // Power to left motor will be reset to zero
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // Power to right motor will be reset to zero

        // Initialize color sensors
        ColorSensor[] colorSensors = new ColorSensor[] {
                l.hardwareMap.get(ColorSensor.class, "scissorDownLimit"),
                l.hardwareMap.get(ColorSensor.class, "scissorUpLimit")
        };

        // Initialize Web Cam
        WebcamName[] webcams = new WebcamName[] {
                l.hardwareMap.get(WebcamName.class, "Webcam 1")
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
                .build();

        // Initializes movement class
        Movement movement = new Movement
                .MovementBuilder()
                .motors(motors)
                .build();

        // Initializes the robot object
        Robot robot = new Robot
                .RobotBuilder()
                .sensor(sensor)
                .movement(movement)
                .linearOpMode(l)
                .build();

        // Initialize VuForia
        robot.getSensor().initVuforia(l.hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", l.hardwareMap.appContext.getPackageName()), 0
        );

        // Check if we can use TFOD. If we can, initialize it.
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            robot.getSensor().initTfod(l.hardwareMap.appContext.getResources().getIdentifier(
                    "tfodMonitorViewId", "id", l.hardwareMap.appContext.getPackageName())
            );
        } else {
            l.telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        return robot;
    }
}
