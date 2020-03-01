/*
Copyright 2020 FIRST Tech Challenge Team 11792
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

package org.firstinspires.ftc.teamcode.Api;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Api.Config.Naming;
import org.firstinspires.ftc.teamcode.Api.Hardware.SmartMotor;
import org.firstinspires.ftc.teamcode.Api.Hardware.SmartServo;

import java.util.HashMap;
import java.util.Objects;

import lombok.AccessLevel;
import lombok.Builder;
import lombok.Getter;

/**
 * The Movement class. Interfaces with servos and motors so you don't have to
 */
@Builder
public class Movement {

    // Autonomous
    final double DRIVE_POWER = 0.6; // How fast to drive

    // Elevator configuration
    private final static double ELEVATOR_POWER   = 1.00;

    // Servo configuration
    private final static int    SERVO_SLEEP      = 10; // Milliseconds
    private final static double SERVO_STEP       = 0.01;  // Degrees
    private final static double GRABBER_OPEN     = 0; // Degrees
    private final static double GRABBER_CLOSE    = 0.65; // Degrees
    private final static double SWIVEL_OPEN      = 0; // Degrees
    private final static double SWIVEL_CLOSE     = 1; // Degrees
    private final static double FOUNDATION_OPEN  = 0.3;
    private final static double FOUNDATION_CLOSE = 0.95;


    /**
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
     (Unless if you know what you are doing)
     */

    @Getter(AccessLevel.PUBLIC) private HashMap<String, SmartMotor> motors;
    @Getter(AccessLevel.PUBLIC) private HashMap<String, SmartServo> servos;
    @Getter(AccessLevel.PUBLIC) private HashMap <String, CRServo> crServos;

    // Getters

    public SmartMotor getMotor(String id) {
        return motors.get(id);
    }

    public SmartServo getServo(String id) {
        return servos.get(id);
    }

    public CRServo getCRServo(String id) {
        return crServos.get(id);
    }

    /**
     * Moves each of the four motors individually. Best for mecanum drives.
     * @param flPower Power to the front left wheel
     * @param frPower Power to the front right wheel
     * @param blPower Power to the back left wheel
     * @param brPower Power to the back right wheel
     */
    public void move4x4(double flPower, double frPower, double blPower, double brPower) {
        Objects.requireNonNull(motors.get(Naming.MOTOR_FL_NAME)).setPower(flPower);
        Objects.requireNonNull(motors.get(Naming.MOTOR_FR_NAME)).setPower(frPower);
        Objects.requireNonNull(motors.get(Naming.MOTOR_BL_NAME)).setPower(blPower);
        Objects.requireNonNull(motors.get(Naming.MOTOR_BR_NAME)).setPower(brPower);
    }

    /**
     * Move the base with four motors based on individual sides, not wheels
     * @param lPower Power to the left side
     * @param rPower Power to the right side
     */
    public void move2x4(double lPower, double rPower) {
        Objects.requireNonNull(motors.get(Naming.MOTOR_FL_NAME)).setPower(lPower);
        Objects.requireNonNull(motors.get(Naming.MOTOR_FR_NAME)).setPower(rPower);
        Objects.requireNonNull(motors.get(Naming.MOTOR_BL_NAME)).setPower(lPower);
        Objects.requireNonNull(motors.get(Naming.MOTOR_BR_NAME)).setPower(rPower);
    }

    /**
     * Move the base with two motors and two values
     * @param lPower Power sent to back left motor
     * @param rPower Power sent to back right motor
     */
    public void move2x2(double lPower, double rPower) {
        Objects.requireNonNull(motors.get(Naming.MOTOR_BL_NAME)).setPower(lPower);
        Objects.requireNonNull(motors.get(Naming.MOTOR_BR_NAME)).setPower(rPower);
    }

    /**
     * Moves the lift up and down, depending on the power sent to the motor. Subject to threshold
     * @param speed Speed of the elevator
     */
    public void moveElevator(double speed) {
        Objects.requireNonNull(motors.get(Naming.MOTOR_LIFT_NAME)).setPower(speed*ELEVATOR_POWER);
    }

    /**
     * Sets the servo to a specific position. Useful if we do not want to slowly scan the servo to a position
     * @param id ID of the servo
     * @param degrees Position (in degrees) to set the servo to.
     */
    public void setServo(String id, double degrees) {
        Objects.requireNonNull(servos.get(id)).setPosition(degrees);
    }

    /**
     * Scan the servo (move the servo slowly) to a position.
     * @param id ID of servo
     * @param degrees Position (in degrees) to scan the servo to.
     */
    public void scanServo(String id, double degrees, boolean clockwise) {
        while (Math.abs(Objects.requireNonNull(servos.get(id)).getPosition() - degrees) < 0.001) {
            if (clockwise) {
                // Scan down
                Objects.requireNonNull(servos.get(id)).setPosition(Objects.requireNonNull(servos.get(id)).getPosition() - SERVO_STEP);
            } else {
                // Scan up
                Objects.requireNonNull(servos.get(id)).setPosition(Objects.requireNonNull(servos.get(id)).getPosition() + SERVO_STEP);
            }
        }
    }

    /**
     * Set the speed of a continuous rotation servo
     * @param id ID of CRServo
     * @param power Power (and subsequently speed) sent to CRServo
     */
    public void setServoSpeed(String id, double power) {
        Objects.requireNonNull(crServos.get(id)).setPower(power);
    }

    /**
     * Opens the grabber based on a boolean assignment
     * @param command true to open the grabber or false to close the grabber
     */
    public void openGrabber(boolean command) {
        SmartServo sg; // sg: Servo grabber
        sg = servos.get(Naming.SERVO_GRABBER_NAME);
        if (command) {
            assert sg != null;
            sg.setPosition(GRABBER_OPEN); // Opens the grabber
        } else {
            assert sg != null;
            sg.setPosition(GRABBER_CLOSE); // Closes the grabber
        }
    }

    /**
     * Opens the swivel based on a boolean assignment
     * @param command true to open the swivel or false to close the swivel
     */
    public void openSwivel(boolean command) {
        SmartServo ss; // ss: Servo Swivel
        ss = servos.get(Naming.SERVO_ROTATE_NAME);
        if (command) {
            assert ss != null;
            ss.setPosition(SWIVEL_OPEN); // Opens the swivel
        } else {
            assert ss != null;
            ss.setPosition(SWIVEL_CLOSE); // Closes the swivel
        }
    }

    public void grabFoundation(boolean command) {
        SmartServo slfn, srfn;
        slfn = servos.get(Naming.SERVO_FOUNDATION_LEFT_NEW_NAME); // sfln: Servo Left Foundation New
        srfn = servos.get(Naming.SERVO_FOUNDATION_RIGHT_NEW_NAME); // sfrn: Servo Right Foundation New
        if (command) { // Grabs foundation
            assert slfn != null;
            slfn.setPosition(FOUNDATION_CLOSE);
            assert srfn != null;
            srfn.setPosition(FOUNDATION_CLOSE);
        } else { // Releases foundation
            assert slfn != null;
            slfn.setPosition(FOUNDATION_OPEN);
            assert srfn != null;
            srfn.setPosition(FOUNDATION_OPEN);
        }
    }
}