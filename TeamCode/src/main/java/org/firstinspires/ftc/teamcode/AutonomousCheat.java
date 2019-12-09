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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="AutonomousCheat")
public class AutonomousCheat extends LinearOpMode {

    private static final double DRIVE_SPEED = 0.5; // Controls controller joystick deadzone

    /*
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

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private long count = 0;

    @Override
    public void runOpMode() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;

        double mod;

        DcMotor sc = hardwareMap.get(DcMotor.class, "scissorLift");
        DcMotor lb = hardwareMap.get(DcMotor.class, "lb");
        DcMotor rb = hardwareMap.get(DcMotor.class, "rb");

        DcMotor[] motors = new DcMotor[] {
                sc,
                null, null, // Because we don't have front motors
                lb, rb
        };

        Servo armSwivel = hardwareMap.get(Servo.class, "armSwivel");
        Servo grabber = hardwareMap.get(Servo.class, "grabber");


        Servo[] servos = new Servo[] {
                armSwivel, grabber
        };

        Movement base = new Movement
                .MovementBuilder()
                .withMotors(motors)
                .withServos(servos)
                .build();

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        sc.setDirection(DcMotor.Direction.FORWARD);
        lb.setDirection(DcMotor.Direction.FORWARD);
        rb.setDirection(DcMotor.Direction.REVERSE);

        sc.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ColorSensor[] colorSensors = new ColorSensor[] {
                hardwareMap.get(ColorSensor.class, "scissorDownLimit"),
                hardwareMap.get(ColorSensor.class, "scissorUpLimit")
        };

        Sensor sensors = new Sensor
                .SensorBuilder()
                .withColorSensors(colorSensors)
                .build();

        Robot robot = new Robot(sensors, base);

        //lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        robot.movement.setServo(0, 0.05);
        robot.movement.setServo(1, 1);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        robot.movement.move2x2(DRIVE_SPEED, DRIVE_SPEED);
        sleep(1500);
        robot.movement.move2x2(0, 0);

    }
}
