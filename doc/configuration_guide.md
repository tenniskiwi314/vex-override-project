# Configuration Guide

This guide provides step-by-step instructions for configuring your VEX V5 robot code using RGB template.

---

## Drivetrain Configuration *(Required)*

### Configure Drivetrain Motor Ports
Find and update the port numbers in `robot-config.h`. Put negative number if the wheel turns the opposite direction of the motor (tip: hand roll the wheel and use V5 brain to check if the motor encoder number goes up or down). Note that if 5.5w motors are used in the setup, it can't be the second motor.

```cpp
#define LEFT_PORT1 -1
#define LEFT_PORT2 2
#define LEFT_PORT3 3
#define RIGHT_PORT1 -4
#define RIGHT_PORT2 5
#define RIGHT_PORT3 6
```

### Configure Inertial and Other Sensor Ports
```cpp
#define INERTIAL_PORT 9
```
### Set Default Drive Mode
```cpp
const int DRIVE_MODE = 1;  // 1 for double arcade, 0 for single arcade, 2 for tank
```

**Action:** Set the drive mode value:
- `1` for **Double Arcade** (right stick for forward/backward, left stick for turning)
- `0` for **Single Arcade** (left stick for both forward/backward and turning)
- `2` for **Tank Drive** (left stick for left side, right stick for right side)

### (optional) Set Scaling Factor for Inertial Sensor
IMU scaling is a number that the inertial sensor degree shown on the brain when the robot turns 360 degree. Update this number in `robot-config.h`. 
```cpp
const int INERTIAL_SCALE = 360;
```

### Configure Drivetrain Parameters, e.g. Wheel Size and Gear Ratio (for auton driving)

```cpp
const float WHEEL_DIAMETER = 2.75;
const float GEAR_RATIO = 0.75;

```

**Action:** 
- Set the wheel diameter to your actual wheel diameter in inches
- Set the gear ratio to your motor-to-wheel gear ratio:
  - `1.0` for direct drive (motor directly connected to wheel)
  - `0.75` for 36:48 gear ratio (36-tooth motor gear, 48-tooth wheel gear)
  - `2.0` for 2:1 gearbox (2 motor rotations = 1 wheel rotation)
  - `0.5` for 1:2 gearbox (0.5 motor rotations = 1 wheel rotation)

---

## Other Subsystems Configuration

### Open robot-config.cpp
Navigate to `src/robot-config.cpp` and locate the `other subsystems` section.

### Configure Subsystem Motors
Add the motor definitions for your other subsystems:

```cpp
// grabber motors
motor grabberMotor = motor(numberToPort(GRABBER_PORT), ratio18_1, port_reverse(GRABBER_PORT));
// Other motors as needed
```

**Action:** Add your motors with correct port numbers and gear ratios.

### Set Total Motor Count
Open `robot-config.h` Locate the motor count definition:

```cpp
const int NUMBER_OF_MOTORS = 9;
```

**Action:** Update to match your total number of motors.

**Motor Monitoring System:**
The program automatically monitors motor health and will alert the driver if:
- **Disconnected motors**: Controller vibrates with "---" pattern and displays "X motor is disconnected"
- **Overheated motors**: Controller vibrates with "---" pattern and displays "motor X is Y°C" (default temperature limit: 50°C)

**When alerts occur:**
- Controller will vibrate to get driver's attention
- Error message appears on controller screen
- Check motor connections and temperatures immediately


## Button Control Configuration 
### Step 1: Open robot-config.cpp
Navigate to `src/robot-config.cpp` and locate the 'button control' section.

### Step 2: Complete code for the button functions

```cpp
void buttonL1Function()
{
  // toggle grabber motor
}
```

---

## Autonomous Routines Configuration

### Step 1: Open autons.cpp
Navigate to `src/autons.cpp` and locate the auton functions.

### Step 2: Create Auton Functions
Add your autonomous routines:

```cpp
void auton1() {
    // Drive forward 24 inches
    chassis.drive(24);
    // Drive turn to 90 degrees field view
    chassis.turn(90);
}

void auton2() {
    // Simple forward movement (6V = 50% of max 12V)
    chassis.driveWithVoltage(6, 6);
    wait(1000, msec);
    chassis.stop(brake);
}
```

**Action:** Write your autonomous routines using the drive APIs.

### Step 3: Map Auton Functions
Find the `runAutonItem()` function:

```cpp
void runAutonItem() {
    switch (currentAutonSelection) {
    case 0:
        auton1();
        break;
    case 1:
        auton2();
        break;
    case 2:
        auton_skill();
        break;
    }
}
```

**Action:** Add your auton functions to the switch statement.

### Step 4: Add Menu Text
Find the `autonMenuText` array:

```cpp
char const * autonMenuText[] = {
    "auton1",
    "auton2",
    "auton_skill"
};
```

**Action:** Add names for your autonomous routines.

---

## Testing Your Configuration

### Step 1: Build the Project
1. Open the project in VSCode
2. Press `Ctrl+Shift+P` (or `Cmd+Shift+P` on Mac)
3. Type "VEX: Build" and select it
4. Check for any compilation errors

### Step 2: Download to Robot
1. Connect your VEX brain via USB
2. Press `Ctrl+Shift+P` (or `Cmd+Shift+P` on Mac)
3. Type "VEX: Download" and select it
4. Wait for the download to complete

### Step 3: Test Driver Control
1. Run the program on the brain
2. Test the set drive mode:
   - **Double Arcade (Mode 1)**: Use right stick for forward/backward, left stick for turning
   - **Single Arcade (Mode 0)**: Use left stick for both forward/backward and turning
   - **Tank Drive (Mode 2)**: Use left stick for left side motors, right stick for right side motors
3. Verify motors respond correctly

### (Optional) Step 4: Test Autonomous Routines During Driver Control
1. **Enter Test Mode**: Press and hold the `A button` within 5 seconds of program startup
2. **Navigate Autons**: Use `Right` to cycle through available autons
3. **Run Auton**: Press `A button` to execute the selected auton 
4. **View Results**: Check controller screen for execution time 

---

## Troubleshooting

### Common Issues:
1. **Motors not responding or in conflict**: Check port numbers
2. **Robot drives in wrong direction**: Reverse motor directions in robot-config.cpp
4. **Auton not running**: Check auton function names in runAutonItem()
5. **Test mode not working**: Ensure you press A button within 5 seconds of startup
6. **Turn sensitivity too high/low**: Adjust `kTurnDampingFactor` value in `setArcadeConstants()` - higher values increase sensitivity, lower values decrease sensitivity
7. **Controller vibrating with "---" pattern**: Check for disconnected or overheated motors - the system automatically monitors motor health


## (optional) Advanced Configuration Constants:
- **kBrake (0.5)**: Controls how quickly the robot stops when joysticks are released
- **kTurnBias (0.5)**: Controls the balance between forward/backward and turning movement
- **kTurnDampingFactor (0.85)**: **Controls turn sensitivity** - lower values make turning slower but more accurate, higher values make turning faster.


```cpp
// Sets the arcade drive constants for the chassis.
// These constants are used to control the arcade drive of the chassis.
chassis.setArcadeConstants(0.5, 0.5, 0.85);
```

**Action:** Adjust these values based on your tuning of the chassis driving behavior. 

```cpp
  // Sets the turn PID constants for the chassis.
  // kp, ki, kd and starti
  chassis.setTurnPID(0.2, .015, 1.5, 7.5);
  // Sets the exit conditions for the turn functions.
  // settle error, settle time, and time out
  chassis.setTurnExitConditions(1.5, 300, 3000);

```

**Action:** steps to tune PID consants
- Set kp, kd and ki all to zero
- Increase the kp until the drive overshoots its target a bit
- Increase the kd until the overshoot is corrected
- Increase the ki to speed up the drive (e.g. turn)
