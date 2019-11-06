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

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREVColorDistance;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.LightSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

public class Sensor {
    // Potentiometer configuration
    public static int POT_MAX = 270; // Max range in degrees
    public static double Vmax = 0.004; // Minimum voltage
    public static double Vmin = 3.304; // Maximum voltage

    // VuForia configuration
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    // Color sensor configuration
    private static final double RED_THESH =   30;
    private static final double GREEN_THESH = 50;
    private static final double BLUE_THESH =  40;

    // Light sensor configuration
    private static final double LIGHT_THRESH = 20;

    // TODO: Initialize more sensors
    BNO055IMU gyro; // Initializes gyroscope
    AnalogInput pot; // Initializes potentiometer
    DigitalChannel button; // Initializes button
    ColorSensor cd_color; // Initializes color sensor
    DistanceSensor cd_dist; // Initializes distance sensor

    private static final String VUFORIA_KEY = "AcM0K6z/////AAABmeiIHPqExEm6uvdttqzvUM8yc5vG8YPI75H9AWdWhYDwS3uA8rxBOa8gofNaaTRkLfYpu0EcoykMACJ9vm2u9D0uBFlsxkOSGnjSGZOH7jjS2A+rm0WyOyZ7krIdfoNm+2yV+nPqoQwFApuUDVN7d/HDXq+iW1P+21ZG1ahvPeDr4zJqoHLf9AvNaUzDWssKFBshs6MXdHPH7TaNAHebpqOwVvwOriBRaM/2ffxi/676+DEGypvu5pRcTwmzkCiP3BEdFVpG8BH1jUEcZ+GQd0s59hhqKV2tJZIQwQgvzZISTGSLZHZ06Ag5tOA+m9zIW5M8UpkdWrFEO7mGBRZnMmW0Ztle8Lg+lEHd6t5lZwuS";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;


    // Put all sensor stuff in here.
    // TODO: Constructor + JavaDoc
    Sensor(BNO055IMU gyro) { // Constructor (to use in another file)
        this.gyro = gyro;
    }

    Sensor(AnalogInput pot) { // Another constructor
        this.pot = pot;
    }
    public double getPot() {
        return (POT_MAX/(Vmax-Vmin))*(pot.getVoltage()-Vmin); // Converts voltage to angle (degrees)
    }

    Sensor(ColorSensor cd) {
        this.cd_color = cd;
    }
    Sensor(DistanceSensor cd) {
        this.cd_dist = cd;
    }
    Sensor(ColorSensor cd, DistanceSensor cd2) {
        this.cd_color = cd;
        this.cd_dist  = cd2;
    }

    public double getLight() {
        return (cd_color.red() + cd_color.blue() + cd_color.green()) / 3.0;
    }

    public boolean getDark() {
        return (getLight() < LIGHT_THRESH);
    }

    /**
     * Gets the RGB value of the color sensor
     * @return 0 if red, 1 if green, 2 if blue, 3 if none
     */
    public int getRGB() {
        if ((cd_color.red()>cd_color.blue()) && (cd_color.red()>cd_color.green()) && cd_color.red()>RED_THESH) {
            return 0;
        } else if ((cd_color.green()>cd_color.red()) && (cd_color.green()>cd_color.blue()) && cd_color.red()>GREEN_THESH) {
            return 1;
        } else if ((cd_color.blue()>cd_color.red()) && (cd_color.blue()>cd_color.green()) && cd_color.red()>BLUE_THESH) {
            return 2;
        } else {
            return 3;
        }
    }

    public int getRed() {
        return cd_color.red();
    }

    public int getGreen() {
        return cd_color.green();
    }

    public int getBlue() {
        return cd_color.blue();
    }

    Sensor(DigitalChannel button) {
        this.button = button;
        button.setMode(DigitalChannel.Mode.INPUT);
    }

    public boolean isPressed() {
        return !(button.getState() == true);
    }

    public String track(VuforiaLocalizer vuforia, TFObjectDetector tfod) {
        return null;
    }
}

