package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "ServoTester")
public class ServoTest extends LinearOpMode {
    private Servo s1, s2, s3, s4, s5, s6;

    @Override
    public void runOpMode() {
        s1 = hardwareMap.get(Servo.class, "s1");
        s2 = hardwareMap.get(Servo.class, "s2");
        s3 = hardwareMap.get(Servo.class, "s3");
        s4 = hardwareMap.get(Servo.class, "s4");
        s5 = hardwareMap.get(Servo.class, "s5");
        s6 = hardwareMap.get(Servo.class, "s6");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while(opModeIsActive()) {
            if (gamepad1.x) {
                s1.setPosition(90);
                s2.setPosition(90);
                s3.setPosition(90);
                s4.setPosition(90);
                s5.setPosition(90);
                s6.setPosition(90);

                sleep(1000);
                s1.setPosition(0);
                s2.setPosition(0);
                s3.setPosition(0);
                s4.setPosition(0);
                s5.setPosition(0);
                s6.setPosition(0);
            }
        }
    }
}
