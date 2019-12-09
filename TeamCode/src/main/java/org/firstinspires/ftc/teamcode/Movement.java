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

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * The Movement class. Moves the robot around through multiple functions.
 */
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

    private DcMotor[] motors;
    private Servo[] servos;
    private CRServo[] crServos;
    
    private Movement(MovementBuilder b) {
        this.motors = b.motors;
        this.servos = b.servos;
        this.crServos = b.crServos;
    }

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

    Servo getServo(int id) {
        return servos[id];
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

    DcMotor getMotor(int id) {
        return motors[id];
    }

    /**
     * Set the speed of a continuous rotation servo
     * @param id ID of CRServo
     * @param power Power (and subsequently speed) sent to CRServo
     */
    void setServoSpeed(int id, double power) {
        crServos[id].setPower(power);
    }

    public void moveEnc1x4(int inches) {
        motors[1].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors[2].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors[3].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors[4].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors[1].setTargetPosition(inches);
        motors[2].setTargetPosition(inches);
        motors[3].setTargetPosition(inches);
        motors[4].setTargetPosition(inches);
        motors[1].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motors[2].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motors[3].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motors[4].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (inches < 0) {
            motors[1].setPower(DRIVE_POWER);
            motors[2].setPower(DRIVE_POWER);
            motors[3].setPower(DRIVE_POWER);
            motors[4].setPower(DRIVE_POWER);
        } else if (inches > 0) {
            motors[1].setPower(-DRIVE_POWER);
            motors[2].setPower(-DRIVE_POWER);
            motors[3].setPower(-DRIVE_POWER);
            motors[4].setPower(-DRIVE_POWER);
        } else {
            motors[1].setPower(0);
            motors[2].setPower(0);
            motors[3].setPower(0);
            motors[4].setPower(0);
        }
    }

    public void moveEnc1x2(double inches) {
        motors[3].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors[4].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors[3].setTargetPosition((int)(inches*COUNTS_PER_INCH));
        motors[4].setTargetPosition((int)(inches*COUNTS_PER_INCH));
        motors[3].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motors[4].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (inches < 0) {
            motors[3].setPower(DRIVE_POWER);
            motors[4].setPower(DRIVE_POWER);
        } else if (inches > 0) {
            motors[3].setPower(-DRIVE_POWER);
            motors[4].setPower(-DRIVE_POWER);
        } else {
            motors[3].setPower(0);
            motors[4].setPower(0);
        }
    }

    /**
     * Magic for using a dynamic set of motors. See the README for more information
     */
    static class MovementBuilder {
        private DcMotor[] motors;
        private Servo[] servos;
        private CRServo[] crServos;

        /**
         * In this format:
         * [       Elevator,
         *  Front Left, Front Right,
         *  Back Left,  Back Right]
         *  So, Elevator is id 0
         *  FL is 1
         *  FR is 2
         *  BL is 3
         *  BR is 4
         */
        MovementBuilder withMotors(DcMotor... m) {
            this.motors = m;
            return this;
        }
        
        MovementBuilder withServos(Servo... s) {
            this.servos = s;
            return this;
        }
        
        MovementBuilder withCRServos(CRServo... c) {
            this.crServos = c;
            return this;
        }
        
        Movement build() {
            return new Movement(this);
        }
    }
}