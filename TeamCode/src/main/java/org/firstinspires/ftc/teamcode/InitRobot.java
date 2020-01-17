package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

class InitRobot {
    private LinearOpMode l;

    InitRobot(LinearOpMode l) {
        this.l = l;
    }

    Robot robot;

    Robot init() {
        // Get motors
        DcMotor sc = l.hardwareMap.get(DcMotor.class, "scissorLift");
        DcMotor lb = l.hardwareMap.get(DcMotor.class, "lb");
        DcMotor rb = l.hardwareMap.get(DcMotor.class, "rb");

        // Add motors into the list
        DcMotor[] motors = new DcMotor[] {
                sc,
                null, null, // Because we don't have front motors
                lb, rb
        };

        // Get servos
        Servo grabber = l.hardwareMap.get(Servo.class, "grabber");
        Servo rotate = l.hardwareMap.get(Servo.class, "rotate");
        Servo fRight = l.hardwareMap.get(Servo.class, "foundationRight");
        Servo fLeft = l.hardwareMap.get(Servo.class, "foundationLeft");

        // Add servos into the list
        Servo[] servos = new Servo[] {
                grabber,
                rotate,
                fRight,
                fLeft
        };

        // Get CRServos
        CRServo armExtend = l.hardwareMap.get(CRServo.class, "extender");

        // Add CRServos into the list
        CRServo[] crServos = new CRServo[] {
                armExtend
        };

        // Add lists into the movement class
        Movement movement = new Movement
                .MovementBuilder()
                .motors(motors)
                .servos(servos)
                .crServos(crServos)
                .build();

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        sc.setDirection(DcMotor.Direction.FORWARD);
        lb.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.FORWARD);

        // Set motors to spin in the correct direction
        sc.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Switch direction of servo
        rotate.setDirection(Servo.Direction.REVERSE);

        // Get color sensors
        ColorSensor scissorDownLimit = l.hardwareMap.get(ColorSensor.class, "scissorDownLimit");
        ColorSensor scissorUpLimit = l.hardwareMap.get(ColorSensor.class, "scissorUpLimit");

        // Add color sensors into list
        ColorSensor[] colorSensors = new ColorSensor[] {
                scissorDownLimit,
                scissorUpLimit
        };

        // Add lists into sensor class
        Sensor sensor = new Sensor
                .SensorBuilder()
                .colorSensors(colorSensors)
                .build();

        // Add movement and sensor class into robot class
        robot = new Robot.RobotBuilder()
                .sensor(sensor)
                .movement(movement)
                .linearOpMode(l)
                .build();

        return robot;
    }
}
