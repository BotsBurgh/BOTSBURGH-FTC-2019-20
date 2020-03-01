package org.firstinspires.ftc.teamcode.Api.Config;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.teamcode.Api.Hardware.SmartMotor;
import org.firstinspires.ftc.teamcode.Api.Hardware.SmartServo;
import org.firstinspires.ftc.teamcode.Api.Movement;
import org.firstinspires.ftc.teamcode.Api.Robot;
import org.firstinspires.ftc.teamcode.Api.Sensor;

import java.util.HashMap;
import java.util.Objects;

/**
 * The Robot Initializer. Place initialzation code here. This prevents needing to sync the init code
 * between all opmodes.
 */
public class InitRobot {
    public static final boolean MODE_4x4 = false; // True if you are using 4x4 drive

    private LinearOpMode l;

    private boolean vuforia;

    public InitRobot(LinearOpMode linearOpMode, Boolean vuforiaEnabled) {
        this.l = linearOpMode;
        this.vuforia = vuforiaEnabled;
    }

    public InitRobot(LinearOpMode linearOpMode) {
        this(linearOpMode, true);
    }

    private Robot robot;

    // TODO: JavaDoc
    public Robot init() {
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
        SmartMotor sc, bl, br, fl, fr;
        sc = new SmartMotor(l.hardwareMap.get(DcMotor.class, Naming.MOTOR_LIFT_NAME));
        bl = new SmartMotor(l.hardwareMap.get(DcMotor.class, Naming.MOTOR_BL_NAME));
        br = new SmartMotor(l.hardwareMap.get(DcMotor.class, Naming.MOTOR_BR_NAME));
        if (MODE_4x4) {
            fl = new SmartMotor(l.hardwareMap.get(DcMotor.class, Naming.MOTOR_FL_NAME));
            fr = new SmartMotor(l.hardwareMap.get(DcMotor.class, Naming.MOTOR_FR_NAME));
        }

        HashMap<String, SmartMotor> motors = new HashMap<>();
        motors.put(Naming.MOTOR_LIFT_NAME, sc);
        motors.put(Naming.MOTOR_BL_NAME, bl);
        motors.put(Naming.MOTOR_BR_NAME, br);
        if (MODE_4x4) {
            motors.put(Naming.MOTOR_FL_NAME, fl);
            motors.put(Naming.MOTOR_FR_NAME, fr);
        }

        // Get servos
        SmartServo grabber = new SmartServo(l.hardwareMap.get(Servo.class, Naming.SERVO_GRABBER_NAME));
        SmartServo rotate = new SmartServo(l.hardwareMap.get(Servo.class, Naming.SERVO_ROTATE_NAME));
        SmartServo fLeftNew = new SmartServo(l.hardwareMap.get(Servo.class, Naming.SERVO_FOUNDATION_LEFT_NEW_NAME));
        SmartServo fRightNew = new SmartServo(l.hardwareMap.get(Servo.class, Naming.SERVO_FOUNDATION_RIGHT_NEW_NAME));

        fLeftNew.setDirection(Servo.Direction.REVERSE);
        fRightNew.setDirection(Servo.Direction.FORWARD);

        // Add servos into the list
        HashMap<String, SmartServo> servos = new HashMap<>();
        servos.put(Naming.SERVO_GRABBER_NAME, grabber);
        servos.put(Naming.SERVO_ROTATE_NAME, rotate);
        servos.put(Naming.SERVO_FOUNDATION_LEFT_NEW_NAME, fLeftNew);
        servos.put(Naming.SERVO_FOUNDATION_RIGHT_NEW_NAME, fRightNew);

        // Get CRServos
        CRServo armExtend = l.hardwareMap.get(CRServo.class, Naming.CRSERVO_EXTEND_NAME);

        // Add CRServos into the list
        HashMap<String, CRServo> crServos = new HashMap<>();
        crServos.put(Naming.CRSERVO_EXTEND_NAME, armExtend);

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        sc.setDirection(DcMotor.Direction.FORWARD);
        bl.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.FORWARD);
        if (MODE_4x4) {
            fl.setDirection(DcMotor.Direction.REVERSE);
            fr.setDirection(DcMotor.Direction.FORWARD);
        }

        // Set motors to spin in the correct direction
        sc.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (MODE_4x4) {
            fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // Switch direction of servo
        rotate.setDirection(Servo.Direction.REVERSE);

        // Get color sensors
        ColorSensor scissorDownLimit = l.hardwareMap.get(ColorSensor.class, Naming.COLOR_SENSOR_DOWN_LIMIT_NAME);
        ColorSensor scissorUpLimit = l.hardwareMap.get(ColorSensor.class, Naming.COLOR_SENSOR_UP_LIMIT_NAME);
        ColorSensor parkSensor = l.hardwareMap.get(ColorSensor.class, Naming.COLOR_SENSOR_PARK);

        // Add color sensors into list
        HashMap<String, ColorSensor> colorSensors = new HashMap<>();
        colorSensors.put(Naming.COLOR_SENSOR_DOWN_LIMIT_NAME, scissorDownLimit);
        colorSensors.put(Naming.COLOR_SENSOR_UP_LIMIT_NAME, scissorUpLimit);
        colorSensors.put(Naming.COLOR_SENSOR_PARK, parkSensor);

        // Get webcams
        WebcamName webcam1 = l.hardwareMap.get(WebcamName.class, Naming.WEBCAME_0_NAME);

        // Add webcams to list
        HashMap<String, WebcamName> webcams = new HashMap<>();
        webcams.put(Naming.WEBCAME_0_NAME, webcam1);

        BNO055IMU gyro0 = l.hardwareMap.get(BNO055IMU.class, Naming.GYRO_0_NAME);
        BNO055IMU gyro1 = l.hardwareMap.get(BNO055IMU.class, Naming.GYRO_1_NAME);

        HashMap<String, BNO055IMU> gyros = new HashMap<>();
        gyros.put(Naming.GYRO_0_NAME, gyro0);
        gyros.put(Naming.GYRO_1_NAME, gyro1);

        // Add lists into the movement class
        Movement movement = Movement.builder()
                .motors(motors)
                .servos(servos)
                .crServos(crServos)
                .build();

        // Add lists into sensor class
        Sensor sensor = Sensor.builder()
                .colorSensors(colorSensors)
                .webcams(webcams)
                .gyros(gyros)
                .build();

        // Add movement and sensor class into robot class
        robot = Robot.builder()
                .sensor(sensor)
                .movement(movement)
                .linearOpMode(l)
                .build();

        // Send power to servos so they don't move
        for (String key : servos.keySet()) {
            robot.getMovement().setServo(key, Objects.requireNonNull(servos.get(key)).getPosition());
        }

        // Initialize gyros
        for (String key : gyros.keySet()) {
            robot.getSensor().initGyro(key);
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
