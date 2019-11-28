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

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

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

    /**
     * Turns the robot with the gyroscope
     * @param degrees Turns the robot with an Orientation object
     */
    void gyroTurn(double degrees) {
        sensor.getGyro(0).startAccelerationIntegration(new Position(), new Velocity(), 500);
        Orientation current = sensor.getGyro(0).getAngularOrientation();
        if (current.firstAngle > degrees) {
            while (Math.abs(current.firstAngle - degrees) > 5) {
                movement.move2x2(movement.TURN_POWER, -movement.TURN_POWER);
            }
        } else if (current.firstAngle < degrees) {
            while (Math.abs(current.firstAngle - degrees) > 5) {
                movement.move2x2(-movement.TURN_POWER, movement.TURN_POWER);
            }
        } else {
            movement.move2x2(0,0);
        }
    }

    void gyroDrive(double inches) {
        sensor.getGyro(0).startAccelerationIntegration(new Position(), new Velocity(), 500);
        Orientation current = sensor.getGyro(0).getAngularOrientation();
        // TODO
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
