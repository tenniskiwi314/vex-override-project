## TL;DR
RGB Template is a plug-and-play VEX template designed for quick prototyping and easy customization.
* **Get a pushbot running quickly**: Simply set the motor ports and drive mode in `robot-config.h`, and you're ready to go.
* **Customize button controls**: Add or modify button controls in the "Button Control" section of `robot-config.cpp`.
* **Write PID-based autonomous routines**: In `auton.cpp`, you only need to learn two basic APIs: drive and turn.
* **Build more advanced autonomous movements**: Advanced users can use the same two APIs with additional parameters to create swing turns, curved paths, chained movements, and distance-sensor-based movements.

## Unique Features
* **Test multiple autonomous routines without a field switch**: Run and debug different autonomous functions directly from the driver control, making iteration much faster.
* **Seamlessly switch between robots and drive mode**: The same code can be configured to accommodate multiple robots and drivetrain configurations ((44w, 55w, 66w motor drivetrains) with only one line of code.
* **Built-in motor health monitoring**: Monitor motor status during operation.
* **Built-in match and autonomous timers**: Easily track game and autonomous time without adding your own timing code.

## Prerequsite
### Knowledge of basic c++ syntax
* variable and statement
* arithmetic
* if
* while
* function
* object dot notation

### Understanding of VEX API
* motor operations
* button press states and events
* wait statement

## Software Installation
*   **Prerequisites:** Before you begin, make sure:
    - You have [VSCode with VEX extension](https://kb.vex.com/hc/en-us/categories/360002333191-V5?sc=vs-code-extension) installed.
    - (Optional) You have the [GitHub](https://github.com/) account and extension in VS Code installed.
    - (Optional) If you are using Mac with Apple silicon: [install Rosetta](https://support.apple.com/en-us/102527) to simulate Intel chip if errors show up with VEX extension.
*   **Download and Open the Project:** 
    *   Download the source code as a ZIP file. Unzip and extract the downloaded files to a local folder.
    *   In VSCode, go to `File > Open Folder` and select the extracted folder.
    *  (Optional) Or follow the instructions to [pull the repository](doc/how_to_clone_and_pull_with_vscode.md) from github using VSCode.
*   **(Optional) If the project cannot build correctly**: 
    *   [Create a new v5 project](https://kb.vex.com/hc/en-us/articles/20146074601748-Creating-VS-Code-Projects-for-V5#:~:text=Select%20the%20'New%20Project'%20button,and%20select%20the%20corresponding%20icon.). Copy the dowloaded `src` and the `include` folders to the new project.

## Project Structure

The project is organized into the following directories:

*   `src/`: Main source code
    *   `main.cpp`: Entry point of the program 
    *   `robot-config.cpp`: Configuration for drivetrain and other subsystems 
    *   `autons.cpp`: Autonomous routines
    *   `rgb-template/`: Library code
*   `include/`: Header files
*   `doc/`: Additional documentation


## Configuration
At the very least, set the correct motor port numbers for the drivetrain in `robot-config.h`.
For detailed step-by-step configuration instructions, see [Configuration Guide](doc/configuration_guide.md).

### (Optional) Button Controls ([robot-config.cpp](src/robot-config.cpp))

*   **Button Functions:** In the 'Button controls' section, complete the button functions for the buttons that are used.

## Test Sample Program
- **Build Project and Run Program:**
  - Download the program to the brain and use game controller to select the program slot and run the program.
- **Drive Mode:**
  - Double Arcade Drive (default): Use one stick to turn and the other stick to drive forward/backward
  - Single Arcade Drive: Use one stick to turn and drive forward/backward
  - Tank Drive: Use left stick for left side motors, right stick for right side motors  
  - Change drive mode for different drivers: Enter the test mode within 5 seconds of program startup and press the controller's `X button` to switch modes. Or set the `drive_mode` variable in the program directly.
- **Automatic Motor Health and Game Time Monitoring**: 
  - The controller will vibrate and display warning messages if any motors are disconnected or overheated (temperature limit: 50°C). Check motor connections and temperatures immediately when alerts occur.
  - The controller will vibrate and display the "end game" message near end game when field switch is plugged-in.

## Quick test of auton code
Navigate to the bottom of `src/robot-config.cpp`, inside the `quickTestCode` function, write a few lines of drive and turn commands for the drivetrain to test them out. Time is measured.

## Autonomous Routines ([autons.cpp](src/autons.cpp))

-   **Auton Functions:** Write your autonomous routines as separate functions.
-   **Auton Mappings:** Map auton functions to auton menu items in the `runAutonItem()` function 
-   **Auton Menu Text:** Add the names of your autonomous functions to the `autonMenuText` array to make them shown on the brain's and controller's screen.
- **(Optional) Test all autons during driver control**:
    - Hold the controller's `A button` within 5 seconds of program startup to enter test mode.
    - When in test mode, press the `A button` to run the selected auton or current step.
    - When in test mode, press the `Right buttons` to cycle through the list of autonomous routines
    - At any time, to abort the auton driving, simply move the joystick.


## Programming Interfaces of the library

### Drive APIs ([drivetrain.h](include/rgb-template/drivetrain.h))
The `Drivetrain` class provides a set of APIs to control the robot's movement using voltage.

### `driveWithVoltage(float leftVoltage, float rightVoltage)`
This is the most basic drive function. It sets the voltage (max is 12V) for the left and right sides of the drivetrain directly. 

**Example:**

```cpp
// Drive forward at half speed for 1 second
chassis.driveWithVoltage(6, 6);
wait(1000, msec);
// rotate slowly for 1 second
chassis.driveWithVoltage(-4, 4);
wait(1000, msec);
chassis.stop(coast);
```

### `turn(...)`

This API turns the robot to a specific heading using a PID controller.

**Variations:**

1.  `turn(float heading)`: Turns to the specified heading with default parameters.
2.  `turn(float heading, float turnMaxVoltage)`: Limits the maximum voltage for a slower, more accurate turn.

**Examples:**

```cpp
// Simple turn to 90 degrees
chassis.turn(90);

// Slower, more accurate turn
chassis.turn(-90, 6);
```

### `drive(...)`

This API drives the robot a specific distance using a PID controller, with options for turning to a heading.

**Variations:**

1.  `drive(float distance)`: Drives the specified distance with default parameters.
2.  `drive(float distance, float driveMaxVoltage)`: Limits the maximum voltage for slow driving.


**Examples:**

```cpp
// Drive backward 24 inches
chassis.drive(-24);

// Drive forward 24 inches with a slow speed, e.g. when intaking field game elements
chassis.drive(24, 4);

```

### `setHeading(...)`

This API set the robot to a specific heading, e.g. when the auton routine starts.

**Examples:**

```cpp
// set robot intial heading to 90 degrees
chassis.setHeading(90);

```
### Advanced Driving Features
- motion chaining using early exit, e.g. no stopping
- distance sensing using distance sensors
- swing turn
- curve drive

**Examples:**

```cpp

// turn to heading 90 with maxium speed, and move on to the next drive command when the robot is within 10 degree of the target heading
chassis.turn(90, 12, 10);

// Drive forward 40 inches while maintaining heading 90, and move on to the next drive command when the robot is within 5 inches of target 
chassis.drive(40, 90, 5);

// Drive towards the side wall, stopping at 12 inches away from the wall (need to install front distance sensor)
chassis.drive(12, 10, 90, 6, 1, true);  

// Hold the right side drivetrain and swing turn to heading 0. Complete stop when reaching the heading.
chassis.turn(0, 10, 1, 1); 

// Drive forward 30 inches while turning to heading 45. Complete stop when reaching the target
chassis.drive(30, 10, 45, 6, 1);

```

### Complete Set of Drive API

```cpp

  // Sets the current heading of the robot.
  void setHeading(float orientationDeg);
  // Drives the robot with a specific voltage for each side of the drivetrain.
  void driveWithVoltage(float leftVoltage, float rightVoltage);

  // Turns the robot to a specific heading.
  void turn(float heading);
  // turn slower with lower max voltage
  void turn(float heading, float turnMaxVoltage); 
  // earlyExitFactor for motion chaining: nonstopping if greater than 1. Maxium is 10.
  void turn(float heading, float turnMaxVoltage, float earlyExitFactor); 
  // only one side moves. swing = 1: right side swings, swing = -1: left side swings.
  void turn(float heading, float turnMaxVoltage, float earlyExitFactor, int swing);

  // Drives the robot a specific distance.
  void drive(float distance);
  // Drives the robot a specific distance with a maximum voltage.
  void drive(float distance, float driveMaxVoltage);
  // earlyExitFactor for motion chaining: nonstopping if greater than 1. Maxium is 10.
  void drive(float distance, float heading, float earlyExitFactor);
  // distance > 0: front sensor, distance < 0: back sensor
  void drive(float distance, float heading, float earlyExitFactor, bool distanceSensing); 
  // Curve drive with max voltage and heading correction
  void drive(float distance, float driveMaxVoltage, float heading, float headingMaxVoltage,  float earlyExitFactor);
  void drive(float distance, float driveMaxVoltage, float heading, float headingMaxVoltage,  float earlyExitFactor, bool distanceSensing);

    // Stops the drivetrain.
  void stop(vex::brakeType mode);

  // Gets the current heading of the robot.
  float getHeading();

```

## Author
Developed by Eric Xiao, from The Cavalry team 169X, formly RGB Bots in middle school.