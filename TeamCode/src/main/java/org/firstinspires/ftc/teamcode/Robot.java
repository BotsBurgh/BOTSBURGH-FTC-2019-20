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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Integrates Sensor class and Movement class so we can use VuForia and motors in one function.
 * Use like so: Robot robot = new Robot(new Sensor(whatever), new Movement(whatever));
 * Refer to Movement and Sensor classes for more information on what those classes do.
 * NOTE: You really should not have to edit this file. If you find an error occurring here, add it
 * to our GitHub issues page at https://github.com/botsburgh/BOTSBURGH-FTC-2019-20/issues
 */
public class Robot {
    Sensor sensor;
    Movement movement;

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    private static final double HEADING_THRESHOLD = 2;      // As tight as we can make it with an integer gyro
    private static final double P_TURN_COEFF      = 0.1;    // Larger is more responsive, but also less stable
    private static final double P_DRIVE_COEFF     = 0.15;     // Larger is more responsive, but also less stable
    /**
     * Initialize robot with both sensor and movement functionality
     * @param s Sensor class
     * @param m Movement class
     */
    Robot(Sensor s, Movement m) {
        sensor = s;
        movement = m;
    }

    /**
     * Initialize robot with only sensor functionality
     * @param s Sensor class
     */
    Robot(Sensor s) {
        sensor = s;
    }

    /**
     * Initialize robot with only movement functionality
     * @param m Movement class
     */
    Robot(Movement m) {
        movement = m;
    }

    void gyroTurn(int id, double speed, double angle) {
        while (!onHeading(id, speed, angle, P_TURN_COEFF)) {
            // Do nothing
        }
    }

    private boolean onHeading(int id, double speed, double angle, double PCoeff) {
        double  error = getError(id, angle);
        double steer;
        boolean onTarget = false;
        double  leftSpeed;
        double  rightSpeed;

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer      = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget   = true;
        } else {
            steer      = getSteer(error, PCoeff);
            rightSpeed = speed * steer;
            leftSpeed  = -rightSpeed;
        }

        // Send desired speeds to motors.
        movement.move2x2(leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    private double getError(int id, double targetAngle) {
        // calculate error in -179 to +180 range
        double error = targetAngle - sensor.getGyro(id).getAngularOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.ZYX,
                AngleUnit.DEGREES
        ).firstAngle;

        while (error > 180) {
            error -= 360;
        }

        while (error <= -180) {
            error += 360;
        }

        return error;
    }

    /**
     *  Method to drive on a fixed compass bearing (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the opmode running.
     *
     * @param speed      Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroDrive(int id, double speed, double distance, double angle) {
        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;
        DcMotor leftDrive, rightDrive;

        leftDrive  = movement.getMotor(3);
        rightDrive = movement.getMotor(4);

        // Determine new target position, and pass to motor controller
        moveCounts = (int)(distance * COUNTS_PER_INCH);
        newLeftTarget = movement.getMotor(4).getCurrentPosition() + moveCounts;
        newRightTarget = rightDrive.getCurrentPosition() + moveCounts;

        // Set Target and Turn On RUN_TO_POSITION
        movement.getMotor(3).setTargetPosition(newLeftTarget);
        movement.getMotor(4).setTargetPosition(newRightTarget);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // start motion.
        speed = Range.clip(Math.abs(speed), 0.0, 1.0);
        leftDrive.setPower(speed);
        rightDrive.setPower(speed);

        // keep looping while we are still active, and BOTH motors are running.
        while ((leftDrive.isBusy() && rightDrive.isBusy())) {
            // adjust relative speed based on heading error.
            error = getError(id, angle);
            steer = getSteer(error, P_DRIVE_COEFF);

            // if driving in reverse, the motor correction also needs to be reversed
            if (distance < 0) {
                steer *= -1.0;
            }

            leftSpeed = speed - steer;
            rightSpeed = speed + steer;

            // Normalize speeds if either one exceeds +/- 1.0;
            max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
            if (max > 1.0) {
                leftSpeed /= max;
                rightSpeed /= max;
            }

            movement.move2x2(leftSpeed, rightSpeed);
        }

        // Stop all motion;
        movement.move2x2(0,0);

        // Turn off RUN_TO_POSITION
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return Desired steering force
     */
    private double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    void vuForiaTurn(double degrees) {
        Orientation startingOri = sensor.getVuforiaRotation();
        double startingDegrees = startingOri.thirdAngle;
        double currentDegrees = sensor.getVuforiaRotation().thirdAngle;
        // Find out if we have to turn right or left.
        if (degrees < startingDegrees) {
            // Turn left
            while (degrees < currentDegrees) {
                movement.move2x2(-movement.TURN_POWER, movement.TURN_POWER);
                currentDegrees = sensor.getVuforiaRotation().thirdAngle;
            }
        } else if (degrees > startingDegrees) {
            // Turn right
            while (degrees > currentDegrees) {
                movement.move2x2(movement.TURN_POWER, -movement.TURN_POWER);
                currentDegrees = sensor.getVuforiaRotation().thirdAngle;
            }
        } else {
            // Don't turn
            movement.move2x2(0,0);
        }

    }

    void vuForiaGoto(VectorF targetPos) {
        VectorF startingPos = sensor.getVuforiaPosition();
        Orientation startingOri = sensor.getVuforiaRotation();

        // Do some trig to find out the angle we have to turn to in order to go to the target pos.
        double degrees = Math.atan(
                (Math.abs((targetPos.get(1) -
                        startingPos.get(1))))/
                    (Math.abs((targetPos.get(0) -
                            startingPos.get(0))))
        ); // The formula: degrees = atan((y2-y1)/(x2-x1))

        Orientation targetOri = new Orientation();
        targetOri.firstAngle = startingOri.firstAngle;
        targetOri.secondAngle = startingOri.secondAngle;
        targetOri.thirdAngle = startingOri.thirdAngle + (float)degrees; // The robot turns on the
                                                                        // Z-Axis
        // Alright, now we have to turn to the targetOri and drive forward until we get to the targetPos
        // First, turn the robot
        vuForiaTurn(targetOri.thirdAngle);

        // Next, go to the position we want to drive to
        startingPos = sensor.getVuforiaPosition(); // We may have moved a little bit when we turned

        // Get distance via distance formula
        double distance = Math.sqrt(
                Math.pow(Math.abs(targetPos.get(1) - Math.abs(startingPos.get(1))), 2) +
                        Math.pow(Math.abs(targetPos.get(0) - Math.abs(startingPos.get(0))), 2)
        ); // The formula: distance = sqrt((y2-y1)^2 + (x2-x1)^2)
        double distanceSoFar = 0;
        VectorF currentPos;

        // While we are not there yet, get there.
        while (distanceSoFar < distance) {
            movement.move2x2(movement.DRIVE_POWER, movement.DRIVE_POWER);
            // TODO: Add functionality to ensure straight driving
            currentPos = sensor.getVuforiaPosition(); // We may have moved a little bit when we turned
            distanceSoFar = distance - (Math.sqrt((Math.abs(targetPos.get(1) - Math.abs(currentPos.get(1)))) + (Math.abs(targetPos.get(0) - Math.abs(currentPos.get(0))))));
        }
        // Done! This has NOT been tested yet
    }
}
