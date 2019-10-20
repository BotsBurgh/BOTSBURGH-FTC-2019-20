package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name="Light Sensor Test")
public class LightSensorTest extends LinearOpMode {
    Sensor ls;
    @Override
    public void runOpMode() {
        ls = new Sensor(hardwareMap.get(ColorSensor.class, "ls"));

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Light ", ls.getLight());
            telemetry.addData("Dark ", ls.getDark());
            telemetry.update();
        }
    }
}
