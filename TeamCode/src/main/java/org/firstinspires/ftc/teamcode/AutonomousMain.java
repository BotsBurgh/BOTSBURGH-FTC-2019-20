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

import java.util.ArrayList;


class AutonomousMain {
    // Declare OpMode Members

    private static final double DRIVE_SPEED = 0.7;
    private static final double TURN_SPEED = 0.5;

    Robot robot;

    AutonomousMain(Robot r) {
        robot = r;
    }

    void blue() {
        robot.gyroDrive(0, DRIVE_SPEED, 24, robot.getSensor().getGyros()[0].getAngularOrientation().firstAngle);
        ArrayList<ArrayList<Float>> pos;
        pos = robot.getSensor().getTfod();
        double distance = ((pos.get(0).get(0)) + (pos.get(0).get(3))) / 2; // figure out what this means
        double turn = Math.acos(3.5 / distance);
        robot.gyroTurn(0, TURN_SPEED, turn);
        robot.gyroDrive(0, DRIVE_SPEED, distance, robot.getSensor().getGyros()[0].getAngularOrientation().firstAngle);
        robot.getMovement().grab(true);
        robot.gyroTurn(0, TURN_SPEED, 90 - turn);
        robot.gyroDrive(0, DRIVE_SPEED, 18, robot.getSensor().getGyros()[0].getAngularOrientation().firstAngle);
        robot.gyroTurn(0, TURN_SPEED, 135); // Robot turns to 135º
        robot.gyroDrive(0, DRIVE_SPEED, 68, robot.getSensor().getGyros()[0].getAngularOrientation().firstAngle); // Robot nears the opponent team's bridge
        robot.gyroTurn(0, TURN_SPEED, 69);
        robot.gyroDrive(0, DRIVE_SPEED, 87, robot.getSensor().getGyros()[0].getAngularOrientation().firstAngle);
        robot.getMovement().grab(false);
        robot.gyroTurn(0, TURN_SPEED, 145);
        robot.gyroDrive(0, DRIVE_SPEED, 87, robot.getSensor().getGyros()[0].getAngularOrientation().firstAngle);
        robot.gyroTurn(0, TURN_SPEED, -145);
        robot.gyroDrive(0, DRIVE_SPEED, 70, robot.getSensor().getGyros()[0].getAngularOrientation().firstAngle);
        robot.gyroDrive(0, DRIVE_SPEED, 96, robot.getSensor().getGyros()[0].getAngularOrientation().firstAngle); // Robot goes across the field
        robot.gyroTurn(0, TURN_SPEED, 90); // Turn 90 degrees so we are facing the
        robot.gyroDrive(0, DRIVE_SPEED, 24, robot.getSensor().getGyros()[0].getAngularOrientation().firstAngle);
    }

    void red() {
        robot.gyroDrive(0, DRIVE_SPEED, 24, robot.getSensor().getGyros()[0].getAngularOrientation().firstAngle);
        ArrayList<ArrayList<Float>> pos;
        pos = robot.getSensor().getTfod();
        double distance = ((pos.get(0).get(0))+(pos.get(0).get(3)))/2;
        double turn = Math.acos(3.5/distance);
        robot.gyroTurn(0, TURN_SPEED, turn);
        robot.gyroDrive(0, DRIVE_SPEED, distance, robot.getSensor().getGyros()[0].getAngularOrientation().firstAngle);
        // TODO - Grabbing the black box
        robot.gyroTurn(0, TURN_SPEED, -(90-turn));
        robot.gyroDrive(0, DRIVE_SPEED, 18, robot.getSensor().getGyros()[0].getAngularOrientation().firstAngle);
        robot.gyroDrive(0, DRIVE_SPEED, 68, robot.getSensor().getGyros()[0].getAngularOrientation().firstAngle); // Robot nears the opponent team's bridge
        robot.gyroTurn(0, TURN_SPEED, -69);
        robot.gyroDrive(0, DRIVE_SPEED, 87, robot.getSensor().getGyros()[0].getAngularOrientation().firstAngle);
        // TODO - Dropping black box on to the foundation
        robot.gyroTurn(0, TURN_SPEED, -145);
        robot.gyroDrive(0, DRIVE_SPEED, 87, robot.getSensor().getGyros()[0].getAngularOrientation().firstAngle);
        robot.gyroTurn(0,TURN_SPEED, 145);
        robot.gyroDrive(0, DRIVE_SPEED, 70, robot.getSensor().getGyros()[0].getAngularOrientation().firstAngle);
    }
}
