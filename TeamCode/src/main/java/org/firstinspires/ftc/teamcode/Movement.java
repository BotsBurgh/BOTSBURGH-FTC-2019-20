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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import lombok.Builder;
import lombok.Getter;

/**
 * The Movement class. Interfaces with servos and motors so you don't have to
 */
@Builder
class Movement {
    // Motor configuration
    private static final double COUNTS_PER_MOTOR_REV  = 1440 ;
    private static final double DRIVE_GEAR_REDUCTION  = 1.0 ; // This is < 1.0 if geared UP
    private static final double WHEEL_DIAMETER_INCHES = 4.0 ; // For calculating circumference

    // Autonomous
    final double TURN_POWER  = 0.4; // How fast to turn
    final double DRIVE_POWER = 0.6; // How fast to drive

    // Elevator configuration
    private final static double ELEVATOR_POWER = 1.00;

    // Servo configuration
    private final static int SERVO_SLEEP = 10; // Milliseconds
    private final static double SERVO_STEP  = 0.01;  // Degrees
    private final static double GRABBER_OPEN = 0; // Degrees
    private final static double GRABBER_CLOSE = 0.65; // Degrees
    private final static double SWIVEL_OPEN = 0; // Degrees
    private final static double SWIVEL_CLOSE = 1; // Degrees

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

    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    @Getter DcMotor[] motors;
    @Getter private Servo[] servos;
    @Getter private CRServo[] crServos;

    /**
     * Moves each of the four motors individually. Best for mecanum drives.
     * @param flPower Power to the front left wheel
     * @param frPower Power to the front right wheel
     * @param blPower Power to the back left wheel
     * @param brPower Power to the back right wheel
     */
    void move4x4(double flPower, double frPower, double blPower, double brPower) {
        motors[1].setPower(flPower);
        motors[2].setPower(frPower);
        motors[3].setPower(blPower);
        motors[4].setPower(brPower);
    }

    /**
     * Move the base with four motors based on individual sides, not wheels
     * @param lPower Power to the left side
     * @param rPower Power to the right side
     */
    void move2x4(double lPower, double rPower) {
        motors[1].setPower(lPower);
        motors[2].setPower(rPower);
        motors[3].setPower(lPower);
        motors[4].setPower(rPower);
    }

    /**
     * Move the base with two motors and two values
     * @param lPower Power sent to back left motor
     * @param rPower Power sent to back right motor
     */
    void move2x2(double lPower, double rPower) {
        motors[3].setPower(lPower);
        motors[4].setPower(rPower);
    }

    /**
     * Moves the motors[0] up and down, depending on the power sent to the motor. Subject to threshold
     * @param speed Speed of the elevator
     */
    void moveElevator(double speed) {
        motors[0].setPower(speed*ELEVATOR_POWER);
    }

    /**
     * Sets the servo to a specific position. Useful if we do not want to slowly scan the servo to a position
     * @param id ID of the servo
     * @param degrees Position (in degrees) to set the servo to.
     */
    void setServo(int id, double degrees) {
        servos[id].setPosition(degrees);
    }

    /**
     * Scan the servo (move the servo slowly) to a position.
     * @param id ID of servo
     * @param degrees Position (in degrees) to scan the servo to.
     */
    void scanServo(int id, double degrees, boolean clockwise) {
        while (Math.abs(servos[id].getPosition() - degrees) < 0.001) {
            if (clockwise) {
                // Scan down
                servos[id].setPosition(servos[id].getPosition() - SERVO_STEP);
            } else {
                // Scan up
                servos[id].setPosition(servos[id].getPosition() + SERVO_STEP);
            }
        }
    }

    /**
     * Set the speed of a continuous rotation servo
     * @param id ID of CRServo
     * @param power Power (and subsequently speed) sent to CRServo
     */
    void setServoSpeed(int id, double power) {
        crServos[id].setPower(power);
    }

    /**
     * Moves the robot with four chassis motors a set number of inches
     * @param inches Inches to move forward (or backward)
     */
    public void moveEnc1x4(int inches) {
        DcMotor fr, fl, br, bl;
        fr = motors[1];
        fl = motors[2];
        br = motors[3];
        bl = motors[4];
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setTargetPosition(inches);
        fl.setTargetPosition(inches);
        br.setTargetPosition(inches);
        bl.setTargetPosition(inches);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (inches < 0) {
            fr.setPower(DRIVE_POWER);
            fl.setPower(DRIVE_POWER);
            br.setPower(DRIVE_POWER);
            bl.setPower(DRIVE_POWER);
        } else if (inches > 0) {
            fr.setPower(-DRIVE_POWER);
            fl.setPower(-DRIVE_POWER);
            br.setPower(-DRIVE_POWER);
            bl.setPower(-DRIVE_POWER);
        } else {
            fr.setPower(0);
            fl.setPower(0);
            br.setPower(0);
            bl.setPower(0);
        }
    }

    /**
     * Moves the robot with two chassis motors a set number of inches
     * @param inches Inches to move forward (or backward)
     */
    public void moveEnc1x2(double inches) {
        DcMotor bl, br;
        bl = motors[3];
        br = motors[4];
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setTargetPosition((int)(inches*COUNTS_PER_INCH));
        br.setTargetPosition((int)(inches*COUNTS_PER_INCH));
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (inches < 0) {
            bl.setPower(DRIVE_POWER);
            br.setPower(DRIVE_POWER);
        } else if (inches > 0) {
            bl.setPower(-DRIVE_POWER);
            br.setPower(-DRIVE_POWER);
        } else {
            bl.setPower(0);
            br.setPower(0);
        }
    }

    /**
     * Grabs the block based on a boolean assignment
     * @param command True for grabbing block and false for releasing block
     */
    void grab(boolean command) {
        Servo sg; // sg: Servo Grabber
        sg = servos[0];
        if (command) {
            sg.setPosition(GRABBER_OPEN);
        } else {
            sg.setPosition(GRABBER_CLOSE);
        }
    }

    void swivel(boolean command) {
        Servo ss; // ss: Servo Swivel
        ss = servos[1];
        if (command) {
            ss.setPosition(1);
        } else {
            ss.setPosition(0);
        }
    }
}