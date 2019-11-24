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
1. Vmin: This is the minimum voltage of the potentiometer. See [Potentiometer Calibration](#####calibrationpotentiometer.java)
1. CAMERA_CHOICE: This is the variable which stores what camera we are going to use for VuForia. It can be either `BACK` or `FRONT`. Because we are using an external webcam, then this must be `BACK`.
1. PHONE_IS_PORTRAIT: This is the variable which stores if the camera is rotated or not. Because our (external) camera is at an angle, then this is true. 
1. phoneXRotate: Phone X orientation. Don't use this, because it will be changed based on `PHONE_IS_PORTRAIT`.
1. phoneYRotate: Same thing as phoneXRotate above.
1. PhoneZRotate: This accounts for an upward tilt of the phone. Because our camera is at the bottom of the robot, then we had to tilt it up by 9.5 degrees. Set to zero if your phone is perfectly straight.
1. RED_THESH: The threshold for detecting if something is red or not. Refer to the [Color Sensor Calibration](#####calibrationcolorsensor.java) section to find appropriate values.
1. GREEN_THESH: The threshold for detecting if something is green or not. Refer to the [Color Sensor Calibration](#####calibrationcolorsensor.java) section to find appropriate values.
1. BLUE_THESH: The threshold for detecting if something is blue or not. Refer to the [Color Sensor Calibration](#####calibrationcolorsensor.java) section to find appropriate values.

Most rookie teams don't need to edit anything below this section, but they should in order to get a better understanding of what is going on. Below, we are going to look at three main things: First, we are going to understand some more static variables, then we are going to look at the functions, and lastly we are going to look at the SensorBuilder class.


#### Movement.java

#### Robot.java

#### BasicMovement.java

#### AutonomousMain.java

#### Miscellaneous Calibration Files

These files are used for finding values of sensors. Useful to ensure if the sensors are working right, and to find suitable values for configuration. These files are all independent from the "Big Three".

##### CalibrationColorSensor.java

TODO

##### CalibrationPotentiometer.java

This is used to calibrate potentiometers. To use, run this OpMode. Turn the potentiometer all the way to one end, and note the number. Turn it to the other end and note down that number too. The smaller number (usually very close to zero) is `Vmin` in [Sensor.java](####sensor.java) and the higher number is `Vmax`. Along with this, you must also find the range of a potentiometer, usually 270°. The number should be on the spec sheet. This number is `POT_MAX`.

##### CalibrationColorSensor.java

This is used to find suitable thresholds for the red, green, and blue detectors in [Sensor.java](####sensor.java). Use the returned values to 

#### Miscellaneous Test Files