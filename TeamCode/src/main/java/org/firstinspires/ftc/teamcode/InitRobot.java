package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.HashMap;

// TODO: JavaDoc
class InitRobot {
    private LinearOpMode l;

    private boolean vuforia;

    InitRobot(LinearOpMode linearOpMode, Boolean vuforiaEnabled) {
        this.l = linearOpMode;
        this.vuforia = vuforiaEnabled;
    }

    InitRobot(LinearOpMode linearOpMode) {
        this(linearOpMode, true);
    }

    Robot robot;

    // TODO: JavaDoc
    Robot init() {
        /*
        * #######                   ######
        * #       #####  # #####    #     # ###### #       ####  #    #
        * #       #    # #   #      #     # #      #      #    # #    #
        * #####   #    # #   #      ######  #####  #      #    # #    #
        * #       #    # #   #      #     # #      #      #    # # ## #
        * #       #    # #   #      #     # #      #      #    # ##  ##
        * ####### #####  #   #      ######  ###### ######  ####  #    #
        */

        // Get motors
        DcMotor sc = l.hardwareMap.get(DcMotor.class, "scissorLift");
        DcMotor lb = l.hardwareMap.get(DcMotor.class, "lb");
        DcMotor rb = l.hardwareMap.get(DcMotor.class, "rb");

        HashMap<String, DcMotor> motors = new HashMap<>();
        motors.put(Naming.MOTOR_BL_NAME, lb);
        motors.put(Naming.MOTOR_BR_NAME, rb);
        motors.put(Naming.MOTOR_LIFT_NAME, sc);

        // Get servos
        Servo grabber = l.hardwareMap.get(Servo.class, "grabber");
        Servo rotate = l.hardwareMap.get(Servo.class, "rotate");
        Servo fRight = l.hardwareMap.get(Servo.class, "foundationRight");
        Servo fLeft = l.hardwareMap.get(Servo.class, "foundationLeft");

        // Add servos into the list
        HashMap<String, Servo> servos = new HashMap<>();
        servos.put(Naming.SERVO_GRABBER_NAME, grabber);
        servos.put(Naming.SERVO_ROTATE_NAME, rotate);
        servos.put(Naming.SERVO_FOUNDATION_RIGHT_NAME, fRight);
        servos.put(Naming.SERVO_FOUNDATION_LEFT_NAME, fLeft);

        // Get CRServos
        CRServo armExtend = l.hardwareMap.get(CRServo.class, "extender");

        // Add CRServos into the list
        HashMap<String, CRServo> crServos = new HashMap<>();
        crServos.put(Naming.CRSERVO_EXTEND_NAME, armExtend);

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
        HashMap<String, ColorSensor> colorSensors = new HashMap<>();
        colorSensors.put(Naming.COLOR_SENSOR_DOWN_LIMIT_NAME, scissorDownLimit);
        colorSensors.put(Naming.COLOR_SENSOR_UP_LIMIT_NAME, scissorUpLimit);

        // Get webcams
        WebcamName webcam1 = l.hardwareMap.get(WebcamName.class, "Webcam 1");

        // Add webcams to list
        HashMap<String, WebcamName> webcams = new HashMap<>();
        webcams.put(Naming.WEBCAME_0_NAME, webcam1);

        BNO055IMU gyro0 = l.hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU gyro1 = l.hardwareMap.get(BNO055IMU.class, "imu 1");

        HashMap<String, BNO055IMU> gyros = new HashMap<>();
        gyros.put(Naming.GYRO_0_NAME, gyro0);
        gyros.put(Naming.GYRO_1_NAME, gyro1);

        // Add lists into sensor class
        Sensor sensor = new Sensor
                .SensorBuilder()
                .colorSensors(colorSensors)
                .webcams(webcams)
                .gyros(gyros)
                .build();

        // Add movement and sensor class into robot class
        robot = new Robot.RobotBuilder()
                .sensor(sensor)
                .movement(movement)
                .linearOpMode(l)
                .build();

        // Initialize gyros
        for (String i : gyros.keySet()) {
            robot.getSensor().initGyro(i);
        }

        if (vuforia) {
            // Initialize Vuforia
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
        }

        return robot;
    }

    // TODO: JavaDoc
    void deInit() {
        if (vuforia) {
            robot.getSensor().deactivateTfod();
            robot.getSensor().deactivateVuforia();
        }
    }
}
