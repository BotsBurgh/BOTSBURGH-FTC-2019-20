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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name="Color Sensor Calibration", group="10-Calibration")
/*
 * The main purpose of this is to calibrate our color sensors. We found that ambient red was causing
 * issues, so we made this file to get proper values for our Sensor class.
 */
public class CalibrationColorSensor extends LinearOpMode {
    private static final double RED_THESH =   500;
    private static final double GREEN_THESH = 700;
    private static final double BLUE_THESH =  600;

    double red, green, blue;
    int def;
    @Override
    public void runOpMode() {
        ColorSensor cl1 = hardwareMap.get(ColorSensor.class, "cl");

        waitForStart();

        while (opModeIsActive()) {
            // do stuff
            red   = cl1.red();
            green = cl1.green();
            blue  = cl1.blue();

            if ((red>blue) && (red>green) && (red>RED_THESH)) {
                def =  0;
            } else if ((green>red) && (green>blue) && (green>GREEN_THESH)) {
                def = 1;
            } else if ((blue>red) && (blue>green) && (blue>BLUE_THESH)) {
                def = 2;
            } else {
                def = 3;
            }
            telemetry.addData("Red", red);
            telemetry.addData("Green", green);
            telemetry.addData("Blue", blue);
            telemetry.addData("Def", def);

            telemetry.update();
            sleep(50);
        }
    }
}
