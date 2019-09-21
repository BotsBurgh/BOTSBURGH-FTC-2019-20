/*
Copyright 2019 FIRST Tech Challenge Team 11792
Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.internal.android.dex.util.Unsigned;

import java.util.Locale;

import static android.os.SystemClock.sleep;

/**
 * The Movement class
 *
 * In this class, you can move the robot easily.
 * So far: Move the arm and drive the robot
 * Created by Nitesh and Sambhav
 */
public class Movement {
    private double TURN_POWER  = 0.4; // How fast to turn
    private double DRIVE_POWER = 0.6; // How fast to drive
    private int SLEEP_MS = 100; // For scanning the servo

    // Arm configuration
    final private static double ARM_POWER     = 0.1;  // Base power sent to arm. Will be adjusted.
    final private static double EXTEND_POWER  = 0.6;  // Extending power/speed
    final private static int    EXTEND_TIC    = 2000; // Extend distance (in tics)
    final private static double ARM_MAX       = 90.0; // The degrees that the arm is at it's maximum angle
    final private static double ARM_MIN       = 0.0;  // The degrees that the arm is at it's minimum angle
    final private static double FREEZE_THRESH = 5.0;  // The play in the arm (for preventing it from moving)
    final private static double FREEZE_STEP   = 0.0001; // The step value for the arm freezing

    private double ARM_SERVO = 1; // The range of the servos in the arm
    private DcMotor motorFL, motorFR, motorBL, motorBR;
    private BNO055IMU gyro;
    private Servo s1, s2;
    private CRServo wheel;
    private DcMotor armTilt, armExtend;

    /**
     * Initialize the class with driving functionality
     * @param motorFL The front left motor
     * @param motorFR The front right motor
     * @param motorBL The back left motor
     * @param motorBR The back right motor
     * @param gyro The BNO055IMU gyroscope
     */
    Movement(DcMotor motorFL, DcMotor motorFR, DcMotor motorBL, DcMotor motorBR, BNO055IMU gyro) {
        this.motorFL   = motorFL;
        this.motorFR   = motorFR;
        this.motorBL   = motorBL;
        this.motorBR   = motorBR;
        this.gyro      = gyro;
    }

    Movement(DcMotor motorFL, DcMotor motorFR, DcMotor motorBL, DcMotor motorBR) {
        this.motorFL   = motorFL;
        this.motorFR   = motorFR;
        this.motorBL   = motorBL;
        this.motorBR   = motorBR;
    }

    Movement(DcMotor motorL, DcMotor motorR) {
        this.motorBL   = motorL;
        this.motorBR   = motorR;
    }

    /**
     * Initialize the class with just the lower arm functionality
     * @param armTilt The motor moving the arm
     * @param armExtend The motor extending the arm
     */
    //Movement(DcMotor armTilt, DcMotor armExtend) {
    //    this.armTilt = armTilt;
    //    this.armExtend = armExtend;
    //}

    /**
     * Initialize the class with just the upper arm functionality
     * @param s1 The servo on the right
     * @param s2 The servo on the left
     * @param wl The CRservo on the intake
     */
    Movement(Servo s1, Servo s2, CRServo wl) {
        this.s1 = s1;
        this.s2 = s2;
        this.wheel = wl;
    }

    /**
     * Moves based on the encoder
     * @param inches How much to move forward or backward in inches
     */
    public void moveEnc(int inches) {
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setTargetPosition(inches);
        motorFR.setTargetPosition(inches);
        motorBL.setTargetPosition(inches);
        motorBR.setTargetPosition(inches);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (inches < 0) {
            motorFL.setPower(DRIVE_POWER);
            motorFR.setPower(DRIVE_POWER);
            motorBL.setPower(DRIVE_POWER);
            motorBR.setPower(DRIVE_POWER);
        } else if (inches > 0) {
            motorFL.setPower(-DRIVE_POWER);
            motorFR.setPower(-DRIVE_POWER);
            motorBL.setPower(-DRIVE_POWER);
            motorBR.setPower(-DRIVE_POWER);
        } else {
            motorFL.setPower(0);
            motorFR.setPower(0);
            motorBL.setPower(0);
            motorBR.setPower(0);
        }
    }

    /**
     * Turns the robot with the gyroscope
     * @param angles Turns the robot with an Orientation object
     */
    public void gyroTurn(double angles) {
        gyro.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        Orientation current = gyro.getAngularOrientation();
        if (current.firstAngle > angles) {
            while (Math.abs(current.firstAngle - angles)>5) {
                motorFL.setPower(TURN_POWER);
                motorFR.setPower(-TURN_POWER);
                motorBL.setPower(TURN_POWER);
                motorBR.setPower(-TURN_POWER);
            }
        } else if (current.firstAngle < angles) {
            while (Math.abs(current.firstAngle-angles)>5) {
                motorFL.setPower(-TURN_POWER);
                motorFR.setPower(TURN_POWER);
                motorBL.setPower(-TURN_POWER);
                motorBR.setPower(TURN_POWER);
            }
        } else {
            motorFL.setPower(0);
            motorFR.setPower(0);
            motorBL.setPower(0);
            motorBR.setPower(0);
        }

    }

    /**
     * Moves servos based on speed
     * @param servo The servo to move
     * @param target The end location
     * @param speed How fast (percent). Must be positive
     */
    public void servoMove(Servo servo, double target, double speed) {
        double increment = 1.0-Math.abs(speed);
        double current=servo.getPosition();
        while (current!=target) {
            if (current<target) {
                current+=increment;
                servo.setPosition(current);
                sleep(SLEEP_MS);
            } else if (current>target) {
                current-=increment;
                servo.setPosition(current);
                sleep(SLEEP_MS);
            } else {
                break;
            }
        }
    }

    /**
     * Moves each of the four motors individually. Best for Mecanum drives.
     * @param flPower Power to the front left wheel
     * @param frPower Power to the front right wheel
     * @param blPower Power to the back left wheel
     * @param brPower Power to the back right wheel
     */
    public void quadMove(double flPower, double frPower, double blPower, double brPower) {
        motorFL.setPower(flPower);
        motorFR.setPower(frPower);
        motorBL.setPower(blPower);
        motorBR.setPower(brPower);
        //if ((flPower > 0) && ()
    }

    public void move2x4(double lPower, double rPower) {
        motorFL.setPower(lPower);
        motorFR.setPower(rPower);
        motorBL.setPower(lPower);
        motorBR.setPower(rPower);
    }

    public void move2x2(double lPower, double rPower) {
        motorBL.setPower(lPower);
        motorBR.setPower(rPower);
    }

    public void armTilt(double degrees) {
        armTilt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int target = (int)((27.0*degrees)/28.0);
        armTilt.setTargetPosition(target);
        armTilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * The speed of the arm intake. Use continuous rotation servos.
     * @param speed The speed of the arm intake
     */
    public void armIntake(double speed) {
        wheel.setPower(speed);

    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    private String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    private String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

}