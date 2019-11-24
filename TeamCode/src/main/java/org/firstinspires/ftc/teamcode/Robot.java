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

/**
 * Integrates Sensor class and Movement class. Mostly for making code look nice.
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
}
