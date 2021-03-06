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

package org.firstinspires.ftc.teamcode.OpModeAutonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Api.Config.Naming;
import org.firstinspires.ftc.teamcode.Api.Robot;

/**
 * This file makes it easier to prevent repetition between autonomous files. By keeping everything
 * together, we are able to prevent redundancy with the autonomous programs.
 */
class AutonomousMain {
    // Declare OpMode Members

    private static final double DRIVE_SPEED = 0.85;
    private static final double DRIVE_SPEED_SLOW = 0.5;
    private static final double TURN_SPEED = 0.4;

    private Robot robot;

    /**
     * The autonomous main constructor.
     * @param r Robot object passed to the AutonomousClass
     */
    AutonomousMain(Robot r) {
        robot = r;
    }

    void park(int side) {
        park(side, 1);
    }

    /**
     * Move until we hit a color (red or blue)
     * @param side -1 is red, 1 is blue
     */
    void park(int side, int direction) {
        if (side == -1) {
            while (!(robot.getSensor().getRed(Naming.COLOR_SENSOR_PARK) >= 70)) { // FIXME: Bug where some situations will cause a false positive, eg: a highly reflective surface on the field
                robot.getMovement().move2x2(-direction*DRIVE_SPEED_SLOW, -direction*DRIVE_SPEED_SLOW);
            }
        } else if (side == 1) {
            while (!(robot.getSensor().getBlue(Naming.COLOR_SENSOR_PARK) >= 70)) { // FIXME: Bug where some situations will cause a false positive, eg: a highly reflective surface on the field
                robot.getMovement().move2x2(-direction*DRIVE_SPEED_SLOW, -direction*DRIVE_SPEED_SLOW);
            }
        }

        robot.getMovement().move2x2(0,0);
    }

    /**
     * Does all of the autonomous: Grab a block (w/out VuForia), drop off the block on the
     * foundation, grab the foundation, pull the foundation to the parking zone, and park
     * @param side -1 is red, 1 is blue
     */
    void all(int side) {
        double offset = offset();

        // Blocks + Autonomous (In Progress)
        autonomousShared(); // Preparation for block grabbing
        sleep(250);
        robot.gyroDrive(Naming.GYRO_0_NAME,DRIVE_SPEED, 18.5, 0, true); // Robot approaches the block
        sleep(500); // Wait time to prepare the grabber for grabbing the block
        robot.getMovement().openGrabber(false); // Robot grabs the block
        sleep(500); // Robot process the grab to avoid unintentional errors
        robot.gyroTurn(Naming.GYRO_0_NAME,TURN_SPEED, side*90+offset); // Robot turns toward the alliance bridge
        robot.gyroDrive(Naming.GYRO_0_NAME,DRIVE_SPEED, 57, 0, true); // Robot drives under the alliance bridge with the
        robot.getMovement().moveElevator(-1);
        sleep(500); // Wait time to prepare robot to release block
        robot.getMovement().moveElevator(0);
        robot.gyroTurn(Naming.GYRO_0_NAME,TURN_SPEED, 0+offset); // Robot turns toward the foundation
        robot.gyroDrive(Naming.GYRO_0_NAME,DRIVE_SPEED, 1, 0, true); // Robot approaches the foundation
        robot.getMovement().openGrabber(true); // Robot releases the block
        sleep(250);
        robot.getMovement().grabFoundation(true); // Robot grabs the foundation
        sleep(250); // Wait time for robot to process the foundation grab
        robot.gyroTurn(Naming.GYRO_0_NAME,TURN_SPEED, side*26+offset); // Robot turns to angle foundation for placement in the building site
        robot.gyroDrive(Naming.GYRO_0_NAME,DRIVE_SPEED, -15, 0, true); // Robot moves the foundation back
        sleep(250); // Wait time to process the angling
        robot.gyroDrive(Naming.GYRO_0_NAME,DRIVE_SPEED,-18, 0, true); // Robot moves the foundation close to the building zone
        robot.getMovement().grabFoundation(false); // Robot releases foundation
        robot.gyroTurn(Naming.GYRO_0_NAME,TURN_SPEED, -side*90+offset); // Robot turns to knock foundation into building site

        elevatorDown(); // Move the elevator down

        park(side); // Robot drives under alliance bridge and parks
    }

    void block(int side) {
        block(side, false);
    }

    /**
     * Drops off the block to the other side.
     * @param side -1 is red, 1 is blue
     * @param wall true if we want to stay near the wall
     */
    void block(int side, boolean wall) {
        double offset = offset();

        autonomousShared();
        sleep(1000);

        robot.gyroDrive(Naming.GYRO_0_NAME, DRIVE_SPEED_SLOW, 24, 0, true); // Robot approaches the block
        sleep(400);
        robot.getMovement().openGrabber(false);
        sleep(400);
        if ((side == Naming.SIDE_RED) && (!wall)) {
            robot.gyroDrive(Naming.GYRO_0_NAME, DRIVE_SPEED, 10, 0, true);
            robot.gyroDrive(Naming.GYRO_0_NAME, DRIVE_SPEED, -10, 0, true);
        }
        if (wall) {
            robot.gyroDrive(Naming.GYRO_0_NAME, DRIVE_SPEED, -18, 0, true);
        }
        robot.gyroTurn(Naming.GYRO_0_NAME,TURN_SPEED, side*90 + offset);
        robot.gyroDrive(Naming.GYRO_0_NAME,DRIVE_SPEED, 35, 0, true);
        robot.getMovement().openGrabber(true);
        park(side, -1);
    }

    void foundation(int side) {
        foundation(side, false);
    }

    /**
     * Start on the building side of the field. Move the foundation into the corner and park.
     * @param side -1 is red, 1 is blue
     */
    void foundation(int side, boolean wall) {

        double offset = offset(); // Setting up the offset

        robot.gyroDrive(Naming.GYRO_0_NAME, DRIVE_SPEED_SLOW, 43.3, 0, true);
        sleep(500);
        robot.gyroTurn(Naming.GYRO_0_NAME, TURN_SPEED, side*100+offset);
        sleep(500);
        robot.gyroDrive(Naming.GYRO_0_NAME, DRIVE_SPEED, 6, 0, true);
        sleep(500);
        robot.gyroTurn(Naming.GYRO_0_NAME, TURN_SPEED, side*150+offset);
        sleep(500);
        robot.gyroDrive(Naming.GYRO_0_NAME, DRIVE_SPEED, 20, 0, true);
        sleep(500);
        if (wall) {
            if (side == Naming.SIDE_RED) {
                robot.gyroTurn(Naming.GYRO_0_NAME, TURN_SPEED, side * -125 + offset);
            } else {
                robot.gyroTurn(Naming.GYRO_0_NAME, TURN_SPEED, side * -128 + offset);
            }
        } else {
            if (side == Naming.SIDE_RED) {
                robot.gyroTurn(Naming.GYRO_0_NAME, TURN_SPEED, side * -95 + offset);
            } else {
                robot.gyroTurn(Naming.GYRO_0_NAME, TURN_SPEED, side * -110 + offset);
            }
        }
        sleep(500);
        park(side);
    }

    /**
     * Startup function for all of the autonomous functions
     */
    private void autonomousShared() {
        robot.getMovement().openSwivel(true);
        sleep(1000);
        robot.getMovement().openGrabber(true);
    }

    /**
     * Gets the offset for the autonomous functions. Keeps the values for turning consistent.
     * @return The angle the robot starts at.
     */
    private double offset() {
        return robot.getSensor().getGyro(Naming.GYRO_0_NAME).getAngularOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.ZYX,
                AngleUnit.DEGREES
        ).firstAngle;
    }

    /**
     * Move the elevator all the way down
     */
    private void elevatorDown() {
        while (robot.getSensor().getRGB(Naming.COLOR_SENSOR_DOWN_LIMIT_NAME) != 0) { // Move down the elevator
            robot.getMovement().moveElevator(1);
        }
        robot.getMovement().moveElevator(0);
    }

    /**
     * Wrapper for the sleep function for code readability
     * @param milliseconds How long to sleep
     */
    private void sleep(int milliseconds) {
        robot.getLinearOpMode().sleep(milliseconds);
    }
}
