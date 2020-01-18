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

    private static final double DRIVE_SPEED = 0.7;
    private static final double TURN_SPEED = 0.5;
    private static final double SERVO_STEP = 0.01;

    Robot robot;

    AutonomousMain(Robot r) {
        robot = r;
    }

    void blue() {
        shared();
        robot.gyroDrive(0, DRIVE_SPEED, 24, robot.getSensor().getGyros()[0].getAngularOrientation().firstAngle);
        /**
        ArrayList<ArrayList<Float>> pos;
        pos = robot.getSensor().getTfodPositions();
        double distance = ((pos.get(0).get(0)) + (pos.get(0).get(3))) / 2; // Robot's calculation to get to the block
        robot.getMovement().openGrabber(false);
        double turn = Math.acos(3.5 / distance);
        robot.gyroTurn(0, TURN_SPEED, turn);
        robot.gyroDrive(0, DRIVE_SPEED, distance, robot.getSensor().getGyros()[0].getAngularOrientation().firstAngle);
         */
        robot.getMovement().openGrabber(false);
        robot.gyroTurn(0, TURN_SPEED, 90);
        robot.gyroDrive(0, DRIVE_SPEED, 38, robot.getSensor().getGyros()[0].getAngularOrientation().firstAngle);
        robot.getMovement().openGrabber(true);
        // robot.gyroDrive(0, DRIVE_SPEED,38, );
        /**
        robot.gyroTurn(0, TURN_SPEED, -45); // Robot turns to 135º
        robot.gyroDrive(0, DRIVE_SPEED, 68, robot.getSensor().getGyros()[0].getAngularOrientation().firstAngle); // Robot nears the opponent team's bridge
        robot.gyroTurn(0, TURN_SPEED, 69);
        robot.gyroDrive(0, DRIVE_SPEED, 87, robot.getSensor().getGyros()[0].getAngularOrientation().firstAngle);
        robot.getMovement().setServo(1, robot.getMovement().getServos()[1].getPosition() - SERVO_STEP);
        robot.getMovement().openGrabber(true);
        robot.gyroTurn(0, TURN_SPEED, 145);
        robot.gyroDrive(0, DRIVE_SPEED, 87, robot.getSensor().getGyros()[0].getAngularOrientation().firstAngle);
        robot.gyroTurn(0, TURN_SPEED, -145);
        robot.gyroDrive(0, DRIVE_SPEED, 70, robot.getSensor().getGyros()[0].getAngularOrientation().firstAngle);
        robot.gyroDrive(0, DRIVE_SPEED, 96, robot.getSensor().getGyros()[0].getAngularOrientation().firstAngle); // Robot goes across the field
        robot.gyroTurn(0, TURN_SPEED, 90); // Turn 90 degrees so we are facing the
        robot.gyroDrive(0, DRIVE_SPEED, 24, robot.getSensor().getGyros()[0].getAngularOrientation().firstAngle);
         */
    }

    void red() {
        shared();
        robot.gyroDrive(0, DRIVE_SPEED, 24, robot.getSensor().getGyros()[0].getAngularOrientation().firstAngle);
        /*
        ArrayList<ArrayList<Float>> pos;
        pos = robot.getSensor().getTfodPositions();
        double distance = ((pos.get(0).get(0))+(pos.get(0).get(3)))/2;
        double turn = Math.acos(3.5/distance);
         */
        //robot.gyroDrive(0, DRIVE_SPEED, distance, robot.getSensor().getGyros()[0].getAngularOrientation().firstAngle);
        robot.getMovement().openGrabber(true);
        robot.gyroTurn(0, TURN_SPEED, -90);
        robot.gyroDrive(0, DRIVE_SPEED, 18, robot.getSensor().getGyros()[0].getAngularOrientation().firstAngle);
        robot.gyroDrive(0, DRIVE_SPEED, 68, robot.getSensor().getGyros()[0].getAngularOrientation().firstAngle); // Robot nears the opponent team's bridge
        robot.gyroTurn(0, TURN_SPEED, -69);
        robot.gyroDrive(0, DRIVE_SPEED, 87, robot.getSensor().getGyros()[0].getAngularOrientation().firstAngle);
        robot.getMovement().openGrabber(false);
        robot.gyroTurn(0, TURN_SPEED, -145);
        robot.gyroDrive(0, DRIVE_SPEED, 87, robot.getSensor().getGyros()[0].getAngularOrientation().firstAngle);
        robot.gyroTurn(0,TURN_SPEED, 145);
        robot.gyroDrive(0, DRIVE_SPEED, 70, robot.getSensor().getGyros()[0].getAngularOrientation().firstAngle);
        robot.gyroDrive(0, DRIVE_SPEED, 96, robot.getSensor().getGyros()[0].getAngularOrientation().firstAngle); // Robot goes across the field
        robot.gyroTurn(0, TURN_SPEED, -90); // Turn 90 degrees so we are facing the
        robot.gyroDrive(0, DRIVE_SPEED, 24, robot.getSensor().getGyros()[0].getAngularOrientation().firstAngle);
    }

    void cheat() {
        robot.getMovement().move2x2(DRIVE_SPEED, DRIVE_SPEED);
        robot.getLinearOpMode().sleep(1500);
        robot.getMovement().move2x2(0, 0);
    }

    void blue_new() {
        double offset = robot.getSensor().getGyros()[0].getAngularOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.ZYX,
                AngleUnit.DEGREES
        ).firstAngle;

        // shared();
        /*
        robot.gyroDrive(0, DRIVE_SPEED, 20, 0, true);
        sleep(750);
        robot.getMovement().openGrabber(false);
        sleep(750);
        robot.gyroDrive(0, DRIVE_SPEED, -15, 0, true);
        sleep(1000);
        robot.gyroTurn(0, TURN_SPEED, -90 + offset);
        sleep(1000);
        robot.gyroDrive(0, DRIVE_SPEED, 38, 0, true);
        sleep(750);
        robot.getMovement().openGrabber(true);
        sleep(750);
        robot.gyroDrive(0,DRIVE_SPEED, -15, 0, true);
        */

        robot.gyroDrive(0, DRIVE_SPEED, 10, 0, true);
        sleep(500);
        robot.gyroTurn(0, TURN_SPEED, -90+offset);
        sleep(500);
        robot.gyroDrive(0, DRIVE_SPEED, 63, 0, true);
        sleep(500);
        robot.gyroTurn(0, TURN_SPEED, -90+offset);
        sleep(500);
        robot.gyroDrive(0, DRIVE_SPEED, 3, 0, true);
        sleep(500);
        robot.getMovement().grabFoundation(true);
        sleep(500);
        robot.gyroDrive(0, DRIVE_SPEED,-8, 0, true);
        sleep(500);
        robot.gyroTurn(0, TURN_SPEED, 90+offset);
        sleep(500);
        robot.gyroDrive(0, DRIVE_SPEED, -60, 0,true);

        /*
        robot.getMovement().openGrabber(false);
        sleep(3000);
        robot.getMovement().openGrabber(true);
        sleep(1500);
        robot.gyroDrive(0, DRIVE_SPEED, -10, 0, true);
        robot.gyroTurn(0, TURN_SPEED, -90 + offset);
        robot.gyroDrive(0, DRIVE_SPEED, 58, 0, true);
        robot.getMovement().openGrabber(false);
        sleep(1500);
        robot.gyroDrive(0, DRIVE_SPEED, -20, 0, true);
         */
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
