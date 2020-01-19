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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

class AutonomousMain {
    // Declare OpMode Members

    private static final double DRIVE_SPEED = 0.85;
    private static final double DRIVE_SPEED_SLOW = 0.7;
    private static final double TURN_SPEED = 0.6;

    Robot robot;

    AutonomousMain(Robot r) {
        robot = r;
    }

    void blue() {
        // TODO
    }

    void red() {
        // TODO
    }

    void red_block() {
        together_block(-1);
    }

    void blue_block() {
        together_block(1);
    }

    void red_foundation() {
        together_foundation(-1);
    }

    void blue_foundation() {
        together_foundation(1);
    }

    void blue_all() {
        together_all(1);
    }

    void red_all() {
        together_all(-1);
    }

    void cheat() {
        robot.getMovement().move2x2(-DRIVE_SPEED_SLOW, -DRIVE_SPEED_SLOW);
        robot.getLinearOpMode().sleep(1000);
        robot.getMovement().move2x2(0, 0);
    }

    /**
     *
     * @param side -1 is red, 1 is blue
     */
    private void together_all(int side) {
        double offset = robot.getSensor().getGyros()[0].getAngularOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.ZYX,
                AngleUnit.DEGREES
        ).firstAngle;



        // Blocks + Autonomous (In Progress)
        shared(); // Preparation for block grabbing
        sleep(250);
        robot.gyroDrive(0, DRIVE_SPEED, 18.5, 0, true); // Robot approaches the block
        sleep(500); // Wait time to prepare the grabber for grabbing the block
        robot.getMovement().openGrabber(false); // Robot grabs the block
        sleep(500); // Robot process the grab to avoid unintentional errors
        robot.gyroTurn(0, TURN_SPEED, side*90+offset); // Robot turns toward the alliance bridge
        robot.gyroDrive(0, DRIVE_SPEED, 57, 0, true); // Robot drives under the alliance bridge with the
        robot.getMovement().moveElevator(-1);
        sleep(500); // Wait time to prepare robot to release block
        robot.getMovement().moveElevator(0);
        robot.gyroTurn(0, TURN_SPEED, 0+offset); // Robot turns toward the foundation
        robot.gyroDrive(0, DRIVE_SPEED, 1, 0, true); // Robot approaches the foundation
        robot.getMovement().openGrabber(true); // Robot releases the block
        sleep(250);
        robot.getMovement().grabFoundation(true); // Robot grabs the foundation
        sleep(250); // Wait time for robot to process the foundation grab
        robot.gyroTurn(0, TURN_SPEED, side*26+offset); // Robot turns to angle foundation for placement in the building site
        robot.gyroDrive(0, DRIVE_SPEED, -15, 0, true); // Robot moves the foundation back
        sleep(250); // Wait time to process the angling
        robot.gyroDrive(0, DRIVE_SPEED,-18, 0, true); // Robot moves the foundation close to the building zone
        robot.getMovement().grabFoundation(false); // Robot releases foundation
        robot.gyroTurn(0, TURN_SPEED, -side*90+offset); // Robot turns to knock foundation into building site
        while (robot.getSensor().getRGB(0) != 0) {
            robot.getMovement().moveElevator(1);
        }
        robot.getMovement().moveElevator(0);
        robot.gyroDrive(0, DRIVE_SPEED, 19, 0,true); // Robot drives under alliance bridge and parks
    }

    /**
     *
     * @param side -1 is red, 1 is blue
     */
    private void together_block(int side) {
        double offset = robot.getSensor().getGyros()[0].getAngularOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.ZYX,
                AngleUnit.DEGREES
        ).firstAngle;

        shared();
        robot.gyroDrive(0, DRIVE_SPEED, 24, 0, true);
        robot.getMovement().openGrabber(false);
        sleep(500);
        robot.gyroDrive(0, DRIVE_SPEED, -10, 0, true);
        robot.gyroTurn(0, TURN_SPEED, side*90 + offset);
        robot.gyroDrive(0, DRIVE_SPEED, 58, 0, true);
        robot.getMovement().openGrabber(true);
        robot.gyroDrive(0, DRIVE_SPEED, -20, 0, true);
    }

    private void together_foundation(int side) {
        double offset = robot.getSensor().getGyros()[0].getAngularOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.ZYX,
                AngleUnit.DEGREES
        ).firstAngle;

        robot.gyroDrive(0, DRIVE_SPEED, 20, 0, true);
        robot.gyroTurn(0, TURN_SPEED, side*90+offset);
        robot.gyroDrive(0, DRIVE_SPEED, 61, 0, true);
        robot.gyroTurn(0, TURN_SPEED, 0+offset);
        robot.gyroDrive(0, DRIVE_SPEED, 2, 0, true);
        robot.getMovement().grabFoundation(true);
        sleep(500);
        robot.gyroDrive(0, DRIVE_SPEED, -15, 0, true);
        robot.gyroTurn(0, TURN_SPEED, side*26+offset);
        sleep(500);
        robot.gyroDrive(0, DRIVE_SPEED,-18, 0, true);
        robot.getMovement().grabFoundation(false);
        robot.gyroTurn(0, TURN_SPEED, side*-90+offset);
        robot.gyroDrive(0, DRIVE_SPEED, 22, 0,true);
        shared();
    }

    private void shared() {
        robot.getMovement().openGrabber(false);
        robot.getMovement().openSwivel(true); // Open arm swivel
        robot.getMovement().openGrabber(true); // Close openGrabber
    }

    private void sleep(int milliseconds) {
        robot.getLinearOpMode().sleep(milliseconds);
    }
}
