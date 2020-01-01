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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import lombok.Builder;
import lombok.Getter;

/**
 * Integrates Sensor class and Movement class so we can use VuForia and motors in one function.
 * Use like so: Robot robot = new Robot(new Sensor(whatever), new Movement(whatever));
 * Refer to Movement and Sensor classes for more information on what those classes do.
 * NOTE: You really should edit this file to suit your robot. If you find an error occurring here,
 * add it to our GitHub issues page at https://github.com/botsburgh/BOTSBURGH-FTC-2019-20/issues
 */
@Builder
public class Robot {
    @Getter private Sensor sensor;
    @Getter private Movement movement;

    private static final double COUNTS_PER_MOTOR_REV  = 1440; // eg: TETRIX Motor Encoder
    private static final double DRIVE_GEAR_REDUCTION  = 2.0;  // This is < 1.0 if geared UP
    private static final double WHEEL_DIAMETER_INCHES = 4.0;  // For figuring circumference
    private static final double COUNTS_PER_INCH       = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    private static final double HEADING_THRESHOLD = 2.0;  // Any less and our robot acts up
    private static final double P_TURN_COEFF      = 0.1;  // Larger is more responsive, but also less stable
    private static final double P_DRIVE_COEFF     = 0.15; // Larger is more responsive, but also less stable

    private static final double TURN_TIMEOUT      = 4.0;  // Timeout to prevent non-stop turning
    private static final double DRIVE_TIMEOUT     = 10.0; // Maximum execution time for driving

    // Quick and dirty hack to prevent issues with stopping the robot
    private LinearOpMode linearOpMode;

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    void gyroTurn(int id, double speed, double angle) {
        double offset = sensor.getGyros()[id].getAngularOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.ZYX,
                AngleUnit.DEGREES
        ).firstAngle;

        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        while (linearOpMode.opModeIsActive()) {
            if (onHeading(id, speed, angle + offset, P_TURN_COEFF) && (linearOpMode.opModeIsActive())) {
                break;
            } else {
                linearOpMode.telemetry.update();
            }
        }
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    private boolean onHeading(int id, double speed, double angle, double PCoeff) {
        double  error = getError(id, angle);
        double  steer;
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

        // Display drive status for the driver.
        linearOpMode.telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
        linearOpMode.telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);

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
        double error = targetAngle - sensor.getGyros()[id].getAngularOrientation(
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
     * GyroDrive wrapper with debug
     * @param id         ID of the gyroscope to be used
     * @param speed      Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    void gyroDrive(int id, double speed, double distance, double angle) {
        gyroDrive(id, speed, distance, angle, false);
    }

    /**
     *  Method to drive on a fixed compass bearing (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the opmode running.
     *
     * @param id         ID of the gyroscope to be used
     * @param speed      Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param debug      Used to debug this function
     */
    void gyroDrive(int id, double speed, double distance, double angle, boolean debug) {
        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;
        DcMotor leftDrive, rightDrive;

        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        leftDrive  = movement.getMotors()[3];
        rightDrive = movement.getMotors()[4];

        if (linearOpMode.opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newLeftTarget = leftDrive.getCurrentPosition() + moveCounts;
            newRightTarget = rightDrive.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            leftDrive.setTargetPosition(newLeftTarget);
            rightDrive.setTargetPosition(newRightTarget);

            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            leftDrive.setPower(speed);
            rightDrive.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (linearOpMode.opModeIsActive() && (leftDrive.isBusy() && rightDrive.isBusy()) &&
                    (runtime.seconds()<=DRIVE_TIMEOUT)) {
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
                    leftSpeed  /= max;
                    rightSpeed /= max;
                }

                movement.move2x2(leftSpeed, rightSpeed);

                if (debug) {
                    // Display drive status for the driver.
                    linearOpMode.telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                    linearOpMode.telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
                    linearOpMode.telemetry.addData("Actual",  "%7d:%7d",      leftDrive.getCurrentPosition(),
                            rightDrive.getCurrentPosition());
                    linearOpMode.telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                }
            }
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

    /**
     * Turn using VuForia. Not tested yet.
     * @param degrees Degrees to turn
     */
    void vuForiaTurn(double degrees) {
        Orientation startingOri = sensor.getVuforiaRotation();
        double startingDegrees = startingOri.thirdAngle; // x
        double currentDegrees = sensor.getVuforiaRotation().thirdAngle;
        // Find out if we have to turn right or left.
        if (degrees < startingDegrees) {
            // Turn left
            while ((degrees < currentDegrees) && linearOpMode.opModeIsActive()) {
                movement.move2x2(-movement.TURN_POWER, movement.TURN_POWER);
                currentDegrees = sensor.getVuforiaRotation().thirdAngle;
            }
        } else if (degrees > startingDegrees) {
            // Turn right
            while ((degrees > currentDegrees) && linearOpMode.opModeIsActive()) {
                movement.move2x2(movement.TURN_POWER, -movement.TURN_POWER);
                currentDegrees = sensor.getVuforiaRotation().thirdAngle;
            }
        } else {
            // Don't turn
            movement.move2x2(0,0);
        }
    }

    /**
     * Go to a position on the field using VuForia. Not tested yet
     * @param targetPos VectorF to go to
     */
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
        targetOri.firstAngle = startingOri.firstAngle; // z
        targetOri.secondAngle = startingOri.secondAngle; // y
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
        while ((distanceSoFar < distance) && linearOpMode.opModeIsActive()) {
            movement.move2x2(movement.DRIVE_POWER, movement.DRIVE_POWER);
            // TODO: Add functionality to ensure straight driving
            currentPos = sensor.getVuforiaPosition(); // We may have moved a little bit when we turned
            distanceSoFar = distance - (Math.sqrt((Math.abs(targetPos.get(1) -
                    Math.abs(currentPos.get(1)))) + (Math.abs(targetPos.get(0) -
                    Math.abs(currentPos.get(0))))));
        }
    }
}
