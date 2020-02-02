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
    static final boolean MODE_4x4 = true; // True if you are using 4x4 drive

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
        DcMotor sc, bl, br, fl, fr;
        sc = l.hardwareMap.get(DcMotor.class, Naming.MOTOR_LIFT_NAME);
        bl = l.hardwareMap.get(DcMotor.class, Naming.MOTOR_BL_NAME);
        br = l.hardwareMap.get(DcMotor.class, Naming.MOTOR_BR_NAME);
        if (MODE_4x4) {
            fl = l.hardwareMap.get(DcMotor.class, Naming.MOTOR_FL_NAME);
            fr = l.hardwareMap.get(DcMotor.class, Naming.MOTOR_FR_NAME);
        }

        HashMap<String, DcMotor> motors = new HashMap<>();
        motors.put(Naming.MOTOR_LIFT_NAME, sc);
        motors.put(Naming.MOTOR_BL_NAME, bl);
        motors.put(Naming.MOTOR_BR_NAME, br);
        if (MODE_4x4) {
            motors.put(Naming.MOTOR_FL_NAME, fl);
            motors.put(Naming.MOTOR_FR_NAME, fr);
        }

        // Get servos
        Servo grabber = l.hardwareMap.get(Servo.class, Naming.SERVO_GRABBER_NAME);
        Servo rotate = l.hardwareMap.get(Servo.class, Naming.SERVO_ROTATE_NAME);
        Servo fRight = l.hardwareMap.get(Servo.class, Naming.SERVO_FOUNDATION_RIGHT_NAME);
        Servo fLeft = l.hardwareMap.get(Servo.class, Naming.SERVO_FOUNDATION_LEFT_NAME);

        // Add servos into the list
        HashMap<String, Servo> servos = new HashMap<>();
        servos.put(Naming.SERVO_GRABBER_NAME, grabber);
        servos.put(Naming.SERVO_ROTATE_NAME, rotate);
        servos.put(Naming.SERVO_FOUNDATION_RIGHT_NAME, fRight);
        servos.put(Naming.SERVO_FOUNDATION_LEFT_NAME, fLeft);

        // Get CRServos
        CRServo armExtend = l.hardwareMap.get(CRServo.class, Naming.CRSERVO_EXTEND_NAME);

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

        // Add color sensors into list
        HashMap<String, ColorSensor> colorSensors = new HashMap<>();
        colorSensors.put(Naming.COLOR_SENSOR_DOWN_LIMIT_NAME, scissorDownLimit);
        colorSensors.put(Naming.COLOR_SENSOR_UP_LIMIT_NAME, scissorUpLimit);

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
                .MODE_4x4(MODE_4x4)
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
