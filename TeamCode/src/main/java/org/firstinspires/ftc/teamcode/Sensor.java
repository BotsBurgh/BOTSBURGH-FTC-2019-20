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

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.LightSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

/*
 TODO: Just a big issue we have here in this file is that we don't have enough documentation
 */

class Sensor {
    // Potentiometer configuration
    private static final int    POT_MAX = 270;   // Max range in degrees
    private static final double Vmax    = 0.004; // Minimum voltage
    private static final double Vmin    = 3.304; // Maximum voltage

    // VuForia configuration
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK; // Back camera or front
    private static final boolean PHONE_IS_PORTRAIT = true; // Set to true because our camera is rotated at 90 degrees

    // Phone configuration
    private static       float phoneXRotate    = 0;
    private static       float phoneYRotate    = 0;
    private static final float phoneZRotate    = 9.5f;

    // Color sensor configuration
    private static final double RED_THESH =   500;
    private static final double GREEN_THESH = 700;
    private static final double BLUE_THESH =  600;

    // Light sensor configuration
    private static final double LIGHT_THRESH = 20;

    /*
        ######  #######    #     # ####### #######    ####### ######  ### #######
        #     # #     #    ##    # #     #    #       #       #     #  #     #
        #     # #     #    # #   # #     #    #       #       #     #  #     #
        #     # #     #    #  #  # #     #    #       #####   #     #  #     #
        #     # #     #    #   # # #     #    #       #       #     #  #     #
        #     # #     #    #    ## #     #    #       #       #     #  #     #
        ######  #######    #     # #######    #       ####### ######  ###    #

        ######  ####### #       ####### #     #    ####### #     # ###  #####
        #     # #       #       #     # #  #  #       #    #     #  #  #     #
        #     # #       #       #     # #  #  #       #    #     #  #  #
        ######  #####   #       #     # #  #  #       #    #######  #   #####
        #     # #       #       #     # #  #  #       #    #     #  #        #
        #     # #       #       #     # #  #  #       #    #     #  #  #     #
        ######  ####### ####### #######  ## ##        #    #     # ###  #####

        #       ### #     # #######
        #        #  ##    # #
        #        #  # #   # #
        #        #  #  #  # #####
        #        #  #   # # #
        #        #  #    ## #
        ####### ### #     # #######

        (Unless you know what you are doing)

     */

    // VuForia global variables
    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    private List<VuforiaTrackable> allTrackables;
    private VuforiaTrackables targetsSkyStone;

    // To get this to work, copy the file VuForiaKey.java.example to VuForiaKey.java and add your key in that file.
    private static final String VUFORIA_KEY = VuForiaKey.VUFORIAKEY;

    // Since ImageTarget trackables use mm to specify their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch; // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23f   * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59; // Units are degrees
    private static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;


    /**
     * This is the webcam we are to use. As with other hardware devices such as motors and
     * servos, this device is identified using the robot configuration tool in the FTC application.
     */
    private WebcamName webcamName = null;

    private boolean targetVisible = false;

    // TODO: Initialize more sensors
    private BNO055IMU[] gyro; // Initialize gyroscopes
    private AnalogInput[] pot; // Initialize potentiometers
    private DigitalChannel[] button; // Initialize buttons
    private ColorSensor[] color; // Initialize color sensors
    private DistanceSensor[] distance; // Initialize distance sensors
    private WebcamName[] webcams; // Initialize webcams

    private Sensor(SensorBuilder b) {
        this.gyro = b.gyro;
        this.pot = b.pot;
        this.button = b.button;
        this.color = b.color;
        this.distance = b.distance;
        this.webcams = b.webcams;
    }

    /**
     * Gets the RGB value of the color sensor
     * @return 0 if red, 1 if green, 2 if blue, 3 if none
     */
    public int getRGB(int id) {
        if ((color[id].red()>color[id].blue()) && (color[id].red()>color[id].green()) && color[id].red()>RED_THESH) {
            return 0;
        } else if ((color[id].green()>color[id].red()) && (color[id].green()>color[id].blue()) && color[id].red()>GREEN_THESH) {
            return 1;
        } else if ((color[id].blue()>color[id].red()) && (color[id].blue()>color[id].green()) && color[id].red()>BLUE_THESH) {
            return 2;
        } else {
            return 3;
        }
    }

    /**
     * Gets if a color sensor detects red
     * @param id ID of the color sensor
     * @return Boolean on if the sensor detects red or not
     */
    int getRed(int id) {
        return color[id].red();
    }

    /**
     * Gets if a color sensor detects green
     * @param id ID of the color sensor
     * @return Boolean on if the sensor detects green or not
     */
    int getGreen(int id) {
        return color[id].green();
    }

    /**
     * Gets if a color sensor detects blue
     * @param id ID of the color sensor
     * @return Boolean on if the sensor detects blue or not
     */
    int getBlue(int id) {
        return color[id].blue();
    }

    /**
     * Gets if a button is pressed
     * @param id ID of the button
     * @return Boolean of if the button is pressed or not
     */
    boolean getButton(int id) {
        return !(button[id].getState());
    }

    /**
     * Gets the position (in degrees) of a potentiometer
     * @param id ID of the potentiometer
     * @return Degrees of the potentiometer
     */
    double getPot(int id) {
        return (POT_MAX/(Vmax-Vmin))*(pot[id].getVoltage()-Vmin); // Converts voltage to angle (degrees)
    }

    // TODO: Add Javadoc / other documentation
    void vuforiaInit(int cameraMonitorViewId) { // Probably needs to be called only once to
        // initialize. Not really tested yet. It's gonna cause some issues, so we're gonna have to
        // add some type of check step if to make sure it has not already been initialized.
        /*
         * Retrieve the camera we are to use.
         */

        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        /*
         * We also indicate which camera on the RC we wish to use.
         */
        parameters.cameraName = webcamName;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
        blueRearBridge.setName("Blue Rear Bridge");
        VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
        redRearBridge.setName("Red Rear Bridge");
        VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
        redFrontBridge.setName("Red Front Bridge");
        VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
        blueFrontBridge.setName("Blue Front Bridge");
        VuforiaTrackable red1 = targetsSkyStone.get(5);
        red1.setName("Red Perimeter 1");
        VuforiaTrackable red2 = targetsSkyStone.get(6);
        red2.setName("Red Perimeter 2");
        VuforiaTrackable front1 = targetsSkyStone.get(7);
        front1.setName("Front Perimeter 1");
        VuforiaTrackable front2 = targetsSkyStone.get(8);
        front2.setName("Front Perimeter 2");
        VuforiaTrackable blue1 = targetsSkyStone.get(9);
        blue1.setName("Blue Perimeter 1");
        VuforiaTrackable blue2 = targetsSkyStone.get(10);
        blue2.setName("Blue Perimeter 2");
        VuforiaTrackable rear1 = targetsSkyStone.get(11);
        rear1.setName("Rear Perimeter 1");
        VuforiaTrackable rear2 = targetsSkyStone.get(12);
        rear2.setName("Rear Perimeter 2");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables = new ArrayList<>();
        allTrackables.addAll(targetsSkyStone);

        /*
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        // Set the position of the Stone Target.  Since it's not fixed in position, assume it's at the field origin.
        // Rotated it to to face forward, and raised it to sit on the ground correctly.
        // This can be used for generic target-centric approach algorithms
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //Set the position of the bridge support targets with relation to origin (center of field)
        blueFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, bridgeRotZ)));

        blueRearBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, bridgeRotZ)));

        redFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, 0)));

        redRearBridge.setLocation(OpenGLMatrix
                .translation(bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, 0)));

        //Set the position of the perimeter targets with relation to origin (center of field)
        red1.setLocation(OpenGLMatrix
                .translation(quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        red2.setLocation(OpenGLMatrix
                .translation(-quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        front1.setLocation(OpenGLMatrix
                .translation(-halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

        front2.setLocation(OpenGLMatrix
                .translation(-halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        blue1.setLocation(OpenGLMatrix
                .translation(-quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        blue2.setLocation(OpenGLMatrix
                .translation(quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        rear1.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));

        rear2.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //
        // Create a transformation matrix describing where the phone is on the robot.
        //
        // NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
        // Lock it into Portrait for these numbers to work.
        //
        // Info:  The coordinate frame for the robot looks the same as the field.
        // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
        // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
        //
        // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
        // pointing to the LEFT side of the Robot.
        // The two examples below assume that the camera is facing forward out the front of the robot.

        // We need to rotate the camera around it's long axis to bring the correct camera forward.
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90 ;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        // TODO: Find the actual values of these numbers
        final float CAMERA_FORWARD_DISPLACEMENT  = 7.88f  * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
        final float CAMERA_VERTICAL_DISPLACEMENT = 4.5f   * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 5.625f * mmPerInch;   // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        //  Let all the trackable listeners know where the phone is.
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }

        // WARNING:
        // In this sample, we do not wait for PLAY to be pressed.  Target Tracking is started immediately when INIT is pressed.
        // This sequence is used to enable the new remote DS Camera Preview feature to be used with this sample.
        // CONSEQUENTLY do not put any driving commands in this loop.
        // To restore the normal OpMode structure, just un-comment the following line:

        // waitForStart();

        // Note: To use the remote camera preview:
        // AFTER you hit Init on the Driver Station, use the "options menu" to select "Camera Stream"
        // Tap the preview window to receive a fresh image.

        targetsSkyStone.activate();
    }

    VectorF getVuforiaPosition() {
        VectorF translation;
        Orientation rotation;
        targetVisible = false;
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }

        // Provide feedback as to where the robot is located (if we know).
        if (targetVisible) {
            // express position (translation) of robot in inches.
            translation = lastLocation.getTranslation();

            // express the rotation of the robot in degrees.
            rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
        } else {
            translation = null;
        }
        return translation;
    }

    Orientation getVuforiaRotation() {
        VectorF translation;
        Orientation rotation;
        targetVisible = false;
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }

        // Provide feedback as to where the robot is located (if we know).
        if (targetVisible) {
            // express position (translation) of robot in inches.
            translation = lastLocation.getTranslation();

            // express the rotation of the robot in degrees.
            rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
        } else {
            translation = null;
            rotation = null;
        }
        return rotation;
    }

    void stopVuforia() {
        targetsSkyStone.deactivate();
    }

    /**
     * Magic for using a dynamic set of motors. See the README for more information
     */
    static class SensorBuilder {
        private BNO055IMU[] gyro; // Initialize gyroscopes
        private AnalogInput[] pot; // Initialize potentiometers
        private DigitalChannel[] button; // Initialize buttons
        private ColorSensor[] color; // Initialize color sensors
        private DistanceSensor[] distance; // Initialize distance sensors
        private WebcamName[] webcams; // Initialize webcams

        SensorBuilder() {}

        SensorBuilder withButtons(DigitalChannel... c) {
            this.button = c;
            return this;
        }

        SensorBuilder withPotentiometers(AnalogInput... a) {
            this.pot = a;
            return this;
        }

        SensorBuilder withColorSensors(ColorSensor... c) {
            this.color = c;
            return this;
        }

        SensorBuilder withGyros(BNO055IMU... g) {
            this.gyro = g;
            return this;
        }

        SensorBuilder withDistanceSensors(DistanceSensor... d) {
            this.distance = d;
            return this;
        }

        SensorBuilder withWebcams(WebcamName... c) {
            this.webcams = c;
            return this;
        }

        Sensor build() {
            return new Sensor(this);
        }
    }
}
