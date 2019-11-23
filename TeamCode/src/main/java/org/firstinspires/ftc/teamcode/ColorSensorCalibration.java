package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name="Color Sensor Calibration")
public class ColorSensorCalibration extends LinearOpMode {
    Sensor cl1, cl2;
    @Override
    public void runOpMode() {
        cl1 = new Sensor(hardwareMap.get(ColorSensor.class, "scissorDownLimit"));
        cl2 = new Sensor(hardwareMap.get(ColorSensor.class, "scissorUpLimit"));

        waitForStart();

        while (opModeIsActive()) {
            // do stuff
            telemetry.addData("Down limit: ", cl1.getRed());
            telemetry.addData("Up limit: ", cl2.getRed());
            telemetry.update();
            sleep(50);
        }
    }
}
