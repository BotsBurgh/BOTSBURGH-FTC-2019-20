# TeamCode Module

Note: Everything here is copyrighted by FTC Team BotsBurgh 11792 under the MIT License unless specified otherwise. See the file LICENSE for more details.

## Welcome to our TeamCode Module

This is where the magic of our robot happens. If you are here, it means you want to gain a better understanding of our code. For the remainder of our code, you will see a comprehensive breakdown of our code so it can be easier for you to use.

As a quick breakdown, you will see the following files:

1. [Sensor.java](#sensorjava)
1. [Movement.java](#movementjava)
1. [Robot.java](#robotjava)
1. [InitRobot.java](#initrobotjava)
1. [Naming.java](#namingjava)
1. [Driving.java](#drivingjava)
1. [AutonomousMain.java](#autonomousmainjava)
1. [Miscellaneous Autonomous Files](#miscellaneous-autonomous-files)
1. [Miscellaneous Calibration Files](#miscellaneous-calibration-files)
1. [Miscellaneous Test Files](#miscellaneous-test-files)

In each file we will go over what the file does and the individual functions in it. We will NOT be going over it line-by-line. The files are commented enough that you should be able to deduce what is going on.

### Foreword

Our code is written in a Modular approach. This means that you can copy over the "Big Three" files (Sensor, Movement, and Robot) over to your folder and utilize the sheer power of a well-written API for your robot. These files are great for beginners and experts alike. Better yet, the "Big Three" can be used year after year (with some exceptions. We wrote it like this because a few years ago, we realized that it was difficult to change variables all across the code, such as when we wanted the robot to go slower during both Autonomous and TeleOp. From there, Nitesh and Myself (Sambhav) decided to centralize the most-used parts of our code, such as moving the robot around and moving the lift up and down. Because of this, the programmers have had to do less work every year, leading to a better robot.

Lets begin!

## Sensor.java

This file is part of the "Big Three" of our robot, along with [Movement.java](#movementjava) and [Robot.java](#robotjava), as it is essential to moving around our robot.

Starting at the beginning of the file, we see a few static variables. These, you probably should change depending on your robot. As outlined in the [Miscellaneous Test Files](#miscellaneous-test-files) section, use the calibration OpModes to find the right values for the static variables. There are a few static variables:

1. POT_MAX: This is the maximum range of the potentiometer (in degrees)
1. Vmax: This is the maximum voltage of the potentiometer. See [Potentiometer Calibration](#calibrationpotentiometerjava)
1. Vmin: This is the minimum voltage of the potentiometer. See [Potentiometer Calibration](#calibrationpotentiometerjava)
1. CAMERA_CHOICE: This is the variable which stores what camera we are going to use for VuForia. It can be either `BACK` or `FRONT`. Because we are using an external webcam, then this must be `BACK`.
1. PHONE_IS_PORTRAIT: This is the variable which stores if the camera is rotated or not. Because our (external) camera is at an angle, then this is true.
1. phoneXRotate: Phone X orientation. Don't use this, because it will be changed based on `PHONE_IS_PORTRAIT`.
1. phoneYRotate: Same thing as phoneXRotate above.
1. PhoneZRotate: This accounts for an upward tilt of the phone. Because our camera is at the bottom of the robot, then we had to tilt it up by 9.5 degrees. Set to zero if your phone is perfectly straight.
1. RED_THESH: The threshold for detecting if something is red or not. Refer to the [Color Sensor Calibration](#calibrationcolorsensorjava) section to find appropriate values.
1. GREEN_THESH: The threshold for detecting if something is green or not. Refer to the [Color Sensor Calibration](#calibrationcolorsensorjava) section to find appropriate values.
1. BLUE_THESH: The threshold for detecting if something is blue or not. Refer to the [Color Sensor Calibration](#calibrationcolorsensorjava) section to find appropriate values.

Most rookie teams don't need to edit anything below this section, but they should at least look at it in order to get a better understanding of what is going on. Below, we are going to look at three main things: First, we are going to understand some more static variables, then we are going to look at the functions, and lastly we are going to look at the SensorBuilder class.

### Additional Static Variables

1. VUFORIA_KEY: This stores the VuForia key. People who copy and paste our code without reading the documentation will run into an error: The file `VuForiaKey.java` does not exist. You must copy the file `VuForiaKey.java.example` to `VuForiaKey.java` and add your key in there. We did this because we thought that some people may inadvertently use our API key, leading to errors for us.
1. mmPerInch: Simple conversion factor for millimeters and inches.
1. mmTargetHeight: The height of the target in millimeters.
1. stoneZ: The orientation of the stone's Z-axis.
1. bridgeX: The X-position of the bridge.
1. bridgeY: The Y-position of the bridge.
1. bridgeZ: The Z-position of the bridge.
1. bridgeRotY: The Y-rotation of the bridge.
1. bridgeRotZ: The Z-rotation of the bridge.
1. halfField: The size of the half field.
1. quadField: The size of the half of half field.

### Sensor Functions

1. getRGB: Returns an integer whether the color detected by the sensor is red, green, or blue based on the thresholds outlined earlier. 0 for red, 1 for green, 2 for blue, 3 for grey. Is overloaded for custom thresholds.
1. getRed: Returns raw red value.
1. getGreen: Returns raw green value.
1. getBlue: Returns raw blue.
1. getButton: Returns true or false based on whether a button is pressed or not.
1. getPotDeg: Returns the degrees of the potentiometer.
1. initGyro: Initialize gyroscope.
1. calibrateGyro: Calibrate gyroscope.
1. initVuforia: Initializes VuForia. Only really needs to be called once.
1. getVuforiaPosition: Returns a Vector of the position of the robot based on VuForia.
1. getVuforiaRotation: Returns an Orientation of the rotation of the robot based on VuForia.
1. initTfod: Initializes TensorFlow object detection.
1. getTfodPositions: Gets the skystones detected with VuForia and TensorFlow.
1. deactivateVuforia: Turns off VuForia.
1. deactivateTfod: Turns off TFOD.

### SensorBuilder

After talking with Austin Frownfelter, we were introduced to something called Project Lombok. Project Lombok is a compile-time only library which processes annotations to prevent boilerplate code, or slightly altered repeated code. With Project Lombok, we could get rid of all of our getter and setter methods and our builder functionality. By adding a `@Getter` and `@Setter` before a variable, we have a getter and setter added to our code (when we compile our code). When we add a `@Builder` before a class, we get a Builder class which will do everything for us.

```java
@Builder
class Sensor {
    ...
    @Getter BNO055IMU[] gyros; // Initialize gyroscopes
    @Getter AnalogInput[] pot; // Initialize potentiometers
    @Getter private DigitalChannel[] button; // Initialize buttons
    @Getter private ColorSensor[] colorSensors; // Initialize color sensors
    @Getter private DistanceSensor[] distance; // Initialize distance sensors
    @Getter private WebcamName[] webcams; // Initialize webcams
    ...
}
```

It is very simple to use. We would pass a list of, say, color sensors to the builder like so:

```java
ColorSensor[] colorSensors = new ColorSensor[] {
    hardwareMap.get(ColorSensor.class, "scissorDownLimit"),
    hardwareMap.get(ColorSensor.class, "scissorUpLimit")
};

Sensor sensors = new Sensor
    .SensorBuilder()
    .colorSensors(colorSensors)
    .build();
```

With this, we eliminated over a hundred lines of code and improved the readability of our code.

## Movement.java

Movement.java fulfills similar requirements that [Sensor.java](#sensorjava) does: centralize the robot's movement functions into one file. This is somewhat easier to write than the Sensor class, but comes with its own issues we have to fix. First, we will go over some static variables that the users probably should change (but usually are fine without changing them), then we will move on to the functions, and lastly to the builder.

### Static Variables

1. ELEVATOR_POWER: The maximum power sent to the elevator / scissor lift.
1. SERVO_STEP: The degrees the servo should scan by.
1. SERVO_SLEEP: The time (in milliseconds) we should wait before scanning the next step in a servo. The total time it will take to scan a servo can be represented by multiplying SERVO_SLEEP by degrees, then dividing by SERVO_STEP.
1. COUNTS_PER_MOTOR_REV: Usually found on the motor spec sheet (used for encoders).
1. DRIVE_GEAR_REDUCTION: The gear reduction or increase (used for encoders).
1. WHEEL_DIAMETER_INCHES: The diameter of the wheel (used for encoders).
1. GRABBER_OPEN: The position of the servo of the grabber in the "open" position
1. GRABBER_CLOSE: The position of the servo of the grabber in the "close" position
1. FOUNDATION_OPEN: THe position of the servo(s) of the foundation grabber in the "open" position.
1. FOUNDATION_CLOSE: THe position of the servo(s) of the foundation grabber in the "open" position.

Unlike [Sensor.java](#sensorjava), there is not really much more to this than the configuration variable listed above other than a static variable used for combining `COUNTS_PER_MOTOR_REV`, `DRIVE_GEAR_REDUCTION`, and `WHEEL_DIAMETER_INCHES` into `COUNTS_PER_INCH`. Most rookie teams don't need to edit anything below this section, but they should in order to get a better understanding of what is going on. Below, we are going to look at two main things: First, we are going to look at the functions, and lastly we are going to look at the MovementBuilder class.

### Movement Functions

1. move4x4: Uses four motors and four variables, moves each wheel independently from the others. This is useful for a mecanum drive.
1. move2x4: Uses four motors and two variables, moves each side independently from the other. This is useful for a traditional drivetrain.
1. move2x2: Uses two motors and two variables, moves each wheel independently from the other. This is useful for a back-wheel drive.
1. moveElevator: Moves the elevator up and down with respect to the elevator speed cap outlined in the [Static Variables](#static-variables) section.
1. setServo: Sets a servo to a specific position. Useful for a grabber.
1. scanServo: Scans a servo (moves the servo to a position slowly). Useful for something requiring less speed.
1. setServoSpeed: Sets the speed of a CR Servo.

### MovementBuilder

Similarly to [Sensor.java](#sensorjava), we also are using a builder class to replace our constructors. We added a single line — `@Builder` — to add a builder.

```java
DcMotor sc, bl, br, fl, fr;
sc = l.hardwareMap.get(DcMotor.class, Naming.MOTOR_LIFT_NAME);
bl = l.hardwareMap.get(DcMotor.class, Naming.MOTOR_BL_NAME);
br = l.hardwareMap.get(DcMotor.class, Naming.MOTOR_BR_NAME);

HashMap<String, DcMotor> motors = new HashMap<>();
motors.put(Naming.MOTOR_LIFT_NAME, sc);
motors.put(Naming.MOTOR_BL_NAME, bl);
motors.put(Naming.MOTOR_BR_NAME, br);

Movement base = new Movement
        .MovementBuilder()
        .motors(motors)
        .build();

// Most robots need the motor on one side to be reversed to drive forward
// Reverse the motor that runs backwards when connected directly to the battery
sc.setDirection(DcMotor.Direction.FORWARD);
lb.setDirection(DcMotor.Direction.REVERSE);
rb.setDirection(DcMotor.Direction.FORWARD);

sc.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
```

We set the motors to use their own variables so we could change some settings of the motors later, such as the zero-power behavior and the direction the motors spin in.

## Robot.java

This file is the epitome of our code, as it integrates [Sensor.java](#sensorjava) and [Movement.java](#movementjava) into one class. This allows us to create functions combining both [Sensor.java](#sensorjava) and [Movement.java](#movementjava), such as those which need to move based on VuForia. In this class, we have a few functions doing just that:

1. gyroTurn: This turns the robot based on the internal gyroscope. Simple code, integrates gyroscope functionality from [Sensor.java](#sensorjava) and the `move2x2` or `move4x4` function from [Movement.java](#movementjava). Useful for autonomous in conditions where VuForia is unavailable or unreliable. However, as VuForia is more accurate, you *should* use that instead. See the functions below for instructions on how to do that.
1. onHeading: Performs one cycle of closed loop heading control.
1. getError: Determines the error between the target angle and the robot's current heading.
1. gyroDrive: This moves the robot forward based on encoders, and corrects for drift based on gyroscopes. When possible, use vuForiaGoto to move the robot, as it is more accurate. However, this is good for situations where VuForia functionality is unreliable. Best suited for autonomous.
1. getSteer: Returns desired steering force.  +/- 1 range.  +ve = steer left
1. vuForiaTurn: This function uses the VuForia orientation function from [Sensor.java](#sensorjava) to determine the angle of the robot, then, using the [Movement.java](#movementjava) class, it will turn the robot until it meets the target angle. Best suited for autonomous.
1. vuForiaGoto: This function uses the VuForia position function from [Sensor.java](#sensorjava) and some trigonometry to determine how far the robot must turn (using the vuForiaTurn function above), then drive forward (using the [Movement.java](#movementjava) class). Best suited for autonomous.

So far, there are two main ways of moving around. One way (the one we are using in tournaments) uses gyroscope(s) and encoders to move the robot around. The robot uses a gyroscope to turn and encoders to move forward / backward. Currently, this is the most tested (and buggy) part of our code. It is known for the gyro turn not to work well with four motor drives or with inaccurate gyroscopes. However, for simple robots, this method of moving works fine. A large portion of this code is copied over from the samples, because we wanted to spend the least amount of time on getting the code, and more time testing and tweaking the code.

```java
void gyroTurn(String id, double speed, double angle) {
    ElapsedTime runtime = new ElapsedTime();
    runtime.reset();
    while (linearOpMode.opModeIsActive() && !linearOpMode.isStopRequested()) {
        if (onHeading(id, speed, angle, P_TURN_COEFF) && (linearOpMode.opModeIsActive())) {
            break;
        } else {
            linearOpMode.telemetry.update();
        }
    }
}
```

WARNING: As both the `vuForiaTurn` and `vuForiaGoto` functions are not tested yet (because we don't have a field to test it in), don't use it unless you can find out if it works or not.

The code in here is mostly simple, with the exception of the math parts. In this class, we used two main math equations: the distance formula and the inverse tangent formula. Using the built-in `Math` library, these functions were not too complex.

To figure out how much we have to turn:

```java
double degrees = Math.atan(
        (Math.abs((targetPos.get(1) -
                startingPos.get(1))))/
            (Math.abs((targetPos.get(0) -
                    startingPos.get(0))))
); // The formula: degrees = atan((y2-y1)/(x2-x1))
```

And to figure out how much we have to drive:

```java
double distance = Math.sqrt(
        Math.pow(Math.abs(targetPos.get(1) - Math.abs(startingPos.get(1))), 2) +
                Math.pow(Math.abs(targetPos.get(0) - Math.abs(startingPos.get(0))), 2)
); // The formula: distance = sqrt((y2-y1)^2 + (x2-x1)^2)
```

This file serves the purpose of bridging the gap between the two modular files, [Sensor.java](#sensorjava) and [Movement.java](#movementjava). Because the design of our code is meant to be modular, the bridge between them needs some tweaking to get it to work everywhere. In this file, you may have to modify some functions to suit your robot, such as replacing the `move2x2` function with `move4x4`, or something else, based on your robot's design.

## InitRobot.java

In order to make robot hardware configuration easier, we added a centralized file for configuring the hardware. Under the `init()` function, add the code to initialize the robot. The only difference is that you must replace all occurrences of `hardwareMap` with `l.hardwareMap`.

Here is an example of initializing an OpMode.

```java
InitRobot initializer = new InitRobot(Driving.this, false);
Robot robot = initializer.init();
```

To initialize hardware, we utilize HashMaps to store everything.

```java
DcMotor bl, br, fl, fr;
bl = l.hardwareMap.get(DcMotor.class, Naming.MOTOR_BL_NAME);
br = l.hardwareMap.get(DcMotor.class, Naming.MOTOR_BR_NAME);
fl = l.hardwareMap.get(DcMotor.class, Naming.MOTOR_FL_NAME);
fr = l.hardwareMap.get(DcMotor.class, Naming.MOTOR_FR_NAME);

HashMap<String, DcMotor> motors = new HashMap<>();

motors.put(Naming.MOTOR_BL_NAME, bl);
motors.put(Naming.MOTOR_BR_NAME, br);
motors.put(Naming.MOTOR_FL_NAME, fl);
motors.put(Naming.MOTOR_FR_NAME, fr);
```

Our initializer is dependent on [Naming.java](#namingjava), where we utilize hashmap names to make sure there are less NullPointerExceptions.

We send all of these to the [Robot.java](#robotjava) class and the Robot object is returned after initialization.

```java
// Add lists into the movement class
Movement movement = new Movement
        .MovementBuilder()
        .motors(motors)
        .servos(servos)
        .crServos(crServos)
        .build();

// Add lists into sensor class
Sensor sensor = new Sensor
        .SensorBuilder()
        .colorSensors(colorSensors)
        .webcams(webcams)
        .gyros(gyros)
        .build();

// Add movement and sensor class into robot class
robot = new Robot.RobotBuilder()
        .sensor(sensor)
        .movement(movement)
        .linearOpMode(l)
        .build();

return robot;
```

In this file, near the very top, there is a variable called `MODE_4x4`, which can be set to switch everything over from a two motor drive to four motor drive. This will switch over Autonomous and TeleOp from a two motor drive to four motor drive. This is useful for teams who utilize a four motor drive and do not wish to change too much code to make this API work.

## Naming.java

This file appears at first glance to increase complexity of the code, which happens to be against our KISS (Keep it simple, sillyhead) principle. However, this file prevents errors on the programmer side by reducing NullPointerExceptions. By placing the naming of everything in this file, we are able to call the hashmap based entirely on the static strings in this file. By using this, we eliminate any error from typos or misinterpretation. The current process of adding hardware to the robot is to first add the name of the hardware in the file.

For example, if we want to add four drive motors, we would add these lines in the [Naming.java](#namingjava) file.

```java
static final String MOTOR_FL_NAME = "fl";
static final String MOTOR_BL_NAME = "bl";
static final String MOTOR_FR_NAME = "fr";
static final String MOTOR_BR_NAME = "br";
```

We would also add the corresponding lines in [InitRobot.java](#initrobotjava).

## Driving.java

This file is our primary TeleOp program. In this, we integrate the "Big Three" classes to run our robot. This file is a great example of what can be done with the API. In this file, we are doing a few things: First, we are moving around the robot, and second, we are moving the elevator up and down with respect to the color sensors detecting if the elevator is overstepping. However, these are all done with an asynchronous function.

As an example of the controller running in an async function:

```java
private class AsyncBase extends AsyncTask<Robot, String, String> {
    @Override
    protected String doInBackground(Robot... params) {
        double mod;
        while (opModeIsActive()) {
            // Adjust speed based on the bumpers. Idea from Robotic Doges
            if (gamepad1.left_bumper || gamepad2.left_bumper) {
                mod = 0.33;
            } else if (gamepad1.right_bumper || gamepad2.right_bumper) {
                mod = 1;
            } else {
                mod = 0.66;
            }

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = gamepad1.left_stick_y;
            double turn  = gamepad1.right_stick_x;
            leftPower  = Range.clip(drive + turn, -mod, mod);
            rightPower = Range.clip(drive - turn, -mod, mod);

            if (InitRobot.MODE_4x4) {
                params[0].getMovement().move2x4(leftPower, rightPower);
            } else {
                params[0].getMovement().move2x2(leftPower, rightPower);
            }
        }
        return "";
    }
}
```

This is called from the main class right before the main loop with this line:

```java
new AsyncBase().executeOnExecutor(AsyncTask.THREAD_POOL_EXECUTOR, robot);
```

This needs to be run only once.

### Driving Static Variables

1. DEADZONE: The controller deadzone to prevent accidental nudges to a joystick.
1. SERVO_POWER: The power to be sent to the servos when moving the arm

### Moving Around

The majority of the code for controlling user input is taken from the examples with some modifications.

```java
// Adjust speed based on the bumpers. Idea from Robotic Doges
if (gamepad1.left_bumper) {
    mod = 1;
} else if (gamepad1.right_bumper) {
    mod = 0.33;
} else {
    mod = 0.66;
}

// POV Mode uses left stick to go forward, and right stick to turn.
// - This uses basic math to combine motions and is easier to drive straight.
double drive = gamepad1.left_stick_y;
double turn  = gamepad1.right_stick_x;
leftPower    = Range.clip(drive + turn, -mod, mod);
rightPower   = Range.clip(drive - turn, -mod, mod);
```

As it is seen above, the bumpers on the gamepad control the robot's speed (defaults to 66% speed), so it is easier to make precise movements with the robot. Using the [Robot.java](#robotjava) class, we are able to move the robot around.

### Moving the Elevator

On our robot, we utilize a scissor lift as an elevator to move game items up and down. However, to prevent the scissor lift from breaking due to physical limits, we placed two color sensors on our robot with some red tape on the scissor lift. Then, in the programming, we added some code to use gamepad input and the sensors to prevent the robot from breaking.

```java
private class AsyncElevatorSpeed extends AsyncTask<Robot, String, String> {
    @Override
    protected String doInBackground(Robot... params) {
        while (opModeIsActive()) {
            // Check if the limit switch is hit either way, and set the movable direction.
            if ((sul == 0) && (gamepad2.left_stick_y < DEADZONE)) {
                // If we cannot go up, and the user tries to go up, we don't allow that to happen.
                elevatorSpeed = 0;
            } else if ((sdl == 0) && (gamepad2.left_stick_y > DEADZONE)) {
                // If we cannot go down, and the user tries to go down, we don't allow that to happen.
                elevatorSpeed = 0;
            } else if (Math.abs(gamepad2.left_stick_y) > DEADZONE) {
                // If the user moves the stick more than 10%, and none of the other conditions are fulfilled, we allow the scissor lift to move
                elevatorSpeed = gamepad2.left_stick_y;
            } else {
                // If the user is not doing anything, we don't allow the scissor lift to move
                elevatorSpeed = 0;
            }
            params[0].getMovement().moveElevator(elevatorSpeed);
        }
        return "";
    }
}
```

When creating this file, we ran into a few issues. The first one being that there was a lot of lag between pressing a button and the robot reacting. After some debugging, we found the root cause of the issue were the color sensors. Polling the color sensors took too much time in between, so we used an asynchronous function for both the elevator and the color sensors. This led to much less activation time. Another issue we had was that the joysticks were accidentally being hit by the drivers, so we added a deadzone to make sure the presses on the joysticks were intentional. Again, we see that the [Robot.java](#robotjava) class is used for moving the elevator.

### Detecting the Color Sensor

To properly use the elevator, we put the color sensor functionality in an asynchronous function because polling the color sensors is the part of the code which takes the longest time.

```java
private class AsyncColorSensor extends AsyncTask<Robot, String, String> {
    @Override
    protected String doInBackground(Robot... params) {
        while (opModeIsActive()) {
            sdl = params[0].getSensor().getRGB(0);
            sul = params[0].getSensor().getRGB(1);
        }
        return "";
    }
}
```

This function sets a number to the color sensor variables, `sul` and `sdl`.

## AutonomousMain.java

AutonomousMain contains two functions to move for both red and blue sides. This is the central Autonomous file. This contains many functions which contain instructions for moving around the robot autonomously based on the situation. Most functions are implemented in the [Miscellaneous Autonomous Files](#miscellaneous-autonomous-files) section as individual programs for ease of use during the competition.

## Miscellaneous Autonomous Files

### AutonomousBlueBlockBridge

This just calls [AutonomousMain](#autonomousmainjava)'s BlueBlock function. This function is a wrapper for the respective combined function which will do the grabbing of a block and parking near the bridge.

### AutonomousBlueBlockWall

This just calls [AutonomousMain](#autonomousmainjava)'s BlueBlock function. This function is a wrapper for the respective combined function which will do the grabbing of a block and parking near the wall.

### AutonomousBlueFoundationBridge

This just calls [AutonomousMain](#autonomousmainjava)'s BlueFoundation function. This function is a wrapper for the respective combined function which will do the foundation and park near the bridge.

### AutonomousBlueFoundationWall

This just calls [AutonomousMain](#autonomousmainjava)'s BlueFoundation function. This function is a wrapper for the respective combined function which will do the foundation and park near the bridge.

### AutonomousBluePark

This just calls [AutonomousMain](#autonomousmainjava)'s BluePark function. This function is a wrapper for the respective combined function which will just park.

### AutonomousRedBlockBridge

This just calls [AutonomousMain](#autonomousmainjava)'s RedBlockBridge function. This function is a wrapper for the respective combined function which will do the grabbing of a block and parking near the bridge side.

### AutonomousRedBlockWall

This just calls [AutonomousMain](#autonomousmainjava)'s RedBlock function. This function is a wrapper for the respective combined function which will do the grabbing of a block and parking near the wall side.

### AutonomousRedFoundationBridge

This just calls [AutonomousMain](#autonomousmainjava)'s RedFoundation function. This function is a wrapper for the respective combined function which will do the foundation and park near the bridge.

### AutonomousRedFoundationWall

This just calls [AutonomousMain](#autonomousmainjava)'s RedFoundation function. This function is a wrapper for the respective combined function which will do the foundation and park near the wall.

### AutonomousRedPark

This just calls [AutonomousMain](#autonomousmainjava)'s RedPark function. This function is a wrapper for the respective combined function which will just park.

## Miscellaneous Calibration Files

These files are used for finding values of sensors. Useful to ensure if the sensors are working right, and to find suitable values for configuration. With the exception of CalibrationPotentiometer.java, these files are all independent from the "Big Three".

### CalibrationBNO055IMU.java

This is used to calibrate the internal IMU (gyroscope) in the REV hub. It is required to run this file and generate the calibration file for this to ensure the gyroscope remains accurate. To use this, you must have the IMU configured as `imu` on the REV hubs. Press "a" on the gamepad to configure the IMU.

### CalibrationColorSensor.java

This is used to find suitable thresholds for the red, green, and blue detectors in [Sensor.java](#sensorjava). Use the returned values to find suitable thresholds for the sensor class.

### CalibrationPotentiometer.java

This is used to calibrate potentiometers. To use, run this OpMode. Turn the potentiometer all the way to one end, and note the number. Turn it to the other end and note down that number too. The smaller number (usually very close to zero) is `Vmin` in [Sensor.java](#sensorjava) and the higher number is `Vmax`. Along with this, you must also find the range of a potentiometer, usually 270°. The number should be on the spec sheet. This number is `POT_MAX`.

## Miscellaneous Test Files

These files are used for testing hardware, such as motors and servos. Useful for identifying the servo type and discarding broken hardware.

### TestMotor.java

Simple program to run all motors at 50% power when the user presses X on the gamepad. Used primarily for identifying broken motors.

### TestGyro.java

Checks the angles of the specified gyroscope.

### TestGyroDrive.java

This file contains an opmode to check if the gyroscope functionality and encoders are functioning properly by checking if the robot moves exactly 3 feet forward without drift.

### TestGyroTurn.java

This file contains an opmode to check if the gyroscope functionality is functioning properly by checking if the robot turns exactly 90 degrees based on the gyroscope.

### TestServo.java

Simple program to set all servos to a position when the user presses X on the gamepad. Used for identifying broken servos and distinguishing between continuous rotation servos and normal servos. The continuous rotation servos would spin in only one direction and the normal servos would go to a 90°, then move back to 0°.

## Conclusion

This README serves the purpose of educating those who wish to use our API in their robot. Because our code is modular and can be dropped into any other team's code folder and be used easily. The "Big Three" files [Sensor.java](#sensorjava), [Movement.java](#movementjava), and [Robot.java](#robotjava) are strong interfaces with the back-lying code. These files can be used to speed the development of the robot's software, and making switching to Android Studio easier.

*To view an interactive version of this README, visit [https://github.com/BotsBurgh/BOTSBURGH-FTC-2019-20/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/readme.md](https://github.com/BotsBurgh/BOTSBURGH-FTC-2019-20/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/readme.md).*
