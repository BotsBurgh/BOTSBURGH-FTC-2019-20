## TeamCode Module

Note: Everything here is copyrighted by FTC Team BotsBurgh 11792 under the MIT License. See the file LICENSE for more details.

### Welcome to our TeamCode Module!

This is where the magic of our robot happens. If you are here, it means you want to gain a better understanding of our code. For the remainder of our code, you will see a comprehensive breakdown of our code so it can be easier for you to use.

As a quick breakdown, you will see the following files:

1. [Sensor.java](####sensor.java)
1. [Movement.java](####movement.java)
1. [Robot.java](####robot.java)
1. [BasicMovement.java](####basicmovement.java)
1. [AutonomousMain.java](####autonomousmain.java)
1. [Miscellaneous Calibration Files](####miscellaneous-calibration-files)
1. [Miscellaneous Test Files](####miscellaneous-test-files)

In each file we will go over what the file does and the individual functions in it. We will NOT be going over it line-by-line. The files are commented enough that you should be able to deduce what is going on.

Foreword: Our code is written in a Modular approach. This means that you can copy over the "Big Three" files (more on that later) over to your folder and utilize the sheer power of a well-written API for your robot. These files are great for beginners and experts alike. Better yet, the "Big Three" can be used year after year (with some exceptions. We wrote it like this because a few years ago, we realized that it was difficult to change variables all across the code, such as when we wanted the robot to go slower during both Autonomous and TeleOp. From there, Nitesh and Myself (Sambhav) decided to centralize the most-used parts of our code, such as moving the robot around and moving the lift up and down. Because of this, the programmers have had to do less work every year, leading to a better robot.

Lets begin!

#### Sensor.java

This file is part of the "Big Three" of our robot, along with [Movement.java](####movement.java) and [Robot.java](####robot.java), as it is essential to moving around our robot.

Starting at the beginning of the file, we see a few static variables. These, you probably should change depending on your robot. As outlined in the [Miscellaneous Test Files](####miscellaneous-test-files) section, use the calibration OpModes to find the right values for the static variables. There are a few static variables:

1. POT_MAX: This is the maximum range of the potentiometer (in degrees)
1. Vmax: This is the maximum voltage of the potentiometer. See [Potentiometer Calibration](#####calibrationpotentiometer.java)
1. Vmin

#### Movement.java

#### Robot.java

#### BasicMovement.java

#### AutonomousMain.java

#### Miscellaneous Calibration Files

These files are used for finding values of sensors. Useful to ensure if the sensors are working right, and to find suitable values for configuration. These files are all independent from the "Big Three".

##### CalibrationPotentiometer.java

This is used to calibrate potentiometers. To use, run this OpMode. Turn the potentiometer all the way to one end, and note the number. Turn it to the other end and note down that number too. The smaller number (usually very close to zero) is `Vmin` in [Sensor.java](####sensor.java) and the higher number is `Vmax`. Along with this, you must also find the range of a potentiometer, usually 270°. The number should be on the spec sheet. This number is `POT_MAX`.

##### CalibrationColorSensor.java

This is used to find suitable thresholds for the red, green, and blue detectors in [Sensor.java](####sensor.java). 

#### Miscellaneous Test Files