# Programming the Clawbot

## Objective

This tutorial will guide you through basic programming of the VEX Clawbot.

## Intended Audience

This tutorial is intended for developers with some programming experience, but
with little to no experience with OkapiLib. If you haven't programmed before, we
recommend checking out all the "Introduction and Basic C++ Features" and
"Classes and Objects" sections of
[this tutorial series](https://www.studytonight.com/cpp/introduction-to-cpp.php).

## Goals

At the end of this tutorial you will have:

-  Understood the basic organization of an OkapiLib project
-  Programmed a basic chassis with "tank" control or "arcade" control
-  Programmed buttons to control the clawbot's lift
-  Programmed a joystick axis to control the clawbot's claw
-  Understood the standard subsystem module methodology
-  Programmed an encoder-based autonomous routine

You can follow VEX's tutorial for building this robot
[here](https://v5beta.vex.com/parent-wrapper.php?id=v5-with-clawbot).
For the purposes of this tutorial, we've plugged in our motors into the
following ports:

| Port   | Description    | Port   | Description   |
|--------|----------------|--------|---------------|
| 1      | Left Wheels    | 11     |               |
| 2      |                | 12     |               |
| 3      | Claw Motor     | 13     |               |
| 4      |                | 14     |               |
| 5      | Vision Sensor  | 15     |               |
| 6      |                | 16     |               |
| 7      |                | 17     |               |
| 8      | Arm Motor      | 18     |               |
| 9      |                | 19     |               |
| 10     | Right Wheels   | 20     |               |

Port 21: Radio

For the ADI:


| Port   | Description    | Port   | Description   |
|--------|----------------|--------|---------------|
| A      | Left Bumper    | E      |               |
| B      | Right Bumper   | F      |               |
| C      |                | G      |               |
| D      |                | H      | Arm Limit     |


To create, build, and upload a new project in PROS 3, run

```bash
prosv5 conduct new <path_to_project>
prosv5 make
prosv5 upload
prosv5 terminal
```

The last 3 commands (`make`, `upload`, `terminal`) can be simplified to
`prosv5 mut`.

## Tank/Arcade Control

OkapiLib uses something called a
[ChassisController](@ref okapi::ChassisController) to interact with a robot's
chassis. This interface lets you use open-loop control methods to drive the
robot around with a joystick, like tank and arcade control. It also provides
methods to move the robot programmatically, like driving in an arc or only
powering one side of the chassis. It also provides closed-loop control methods
to drive a specific distance or turn a specific angle.

There are two main subclasses we can use:
[ChassisControllerIntegrated](@ref okapi::ChassisControllerIntegrated) and
[ChassisControllerPID](@ref okapi::ChassisControllerPID).
[ChassisControllerIntegrated](@ref okapi::ChassisControllerIntegrated) uses the
V5 motor's built-in position and velocity control to move the robot around.
**This class is the easiest to use**, so we will use it for this tutorial. The
other class, [ChassisControllerPID](@ref okapi::ChassisControllerPID), uses
three PID controllers running on the V5 brain and sends velocity commands to the
motors. This class can give you more accurate chassis control, but requires
tuning the PID controllers.

We will be using
[ChassisControllerIntegrated](@ref okapi::ChassisControllerIntegrated) for this
tutorial. Let's initialize it now with our two motors in ports ``1`` and ``10``.
The motor in port ``10`` is negative because it is reversed.

```cpp
// Chassis Controller - lets us drive the robot around with open- or closed-loop control
auto drive = ChassisControllerBuilder().withMotors(1, -10).build();
```

Next, let's setup tank or arcade control.
[ChassisController](@ref okapi::ChassisController) provides methods for us to
use, we just need to pass in joystick values which have been scaled to be in the
range `[-1, 1]`. OkapiLib's [Controller](@ref okapi::Controller) returns analog
values in this range, so we don't need to do any scaling ourselves.

### Tank drive

```cpp
// Joystick to read analog values for tank or arcade control.
// Master controller by default.
Controller controller;

while (true) {
    // Tank drive with left and right sticks.
    drive->tank(controller.getAnalog(ControllerAnalog::leftY),
                controller.getAnalog(ControllerAnalog::rightY));

    // Wait and give up the time we don't need to other tasks.
    // Additionally, joystick values, motor telemetry, etc. all updates every 10 ms.
    pros::Task::delay(10);
}
```

### Arcade drive

```cpp
// Joystick to read analog values for tank or arcade control.
// Master controller by default.
Controller controller;

while (true) {
    // Arcade drive with the left stick.
    drive->arcade(controller.getAnalog(ControllerAnalog::leftY),
                  controller.getAnalog(ControllerAnalog::leftX));

    // Wait and give up the time we don't need to other tasks.
    // Additionally, joystick values, motor telemetry, etc. all updates every 10 ms.
    pros::Task::delay(10);
}
```

## Arm Control

This section will focus on controlling the clawbot's arm. There are two parts to
this: first, the arm has a limit switch at the bottom of its travel range, so we
should use that button to tell when we've hit a hard stop; second, the arm
should be user-controlled with two buttons on the controller.

First, let's focus on the limit switch at the bottom of the arm's travel range.
When the arm hits this button, the arm motor should stop trying to make the arm
move down. We can accomplish this using an if-statement that checks whether the
button is pressed.

We can define our button as an [ADIButton](@ref okapi::ADIButton):

```cpp
ADIButton armLimitSwitch('H');
```

And the arm motor:

```cpp
Motor armMotor = 8_rmtr;
```

The `_mtr` syntax is called a user-defined literal. It's a succinct way of
initializing a motor. For example,

```cpp
Motor foo = 1_mtr; // Motor in port 1
Motor foo(1);      // Motor in port 1

Motor bar = 1_rmtr; // Reversed motor in port 1
Motor bar(1, true); // Reversed motor in port 1
```

Then we can check if it's pressed and stop powering the arm motor:

```cpp
// Don't power the arm if it is all the way down
if (armLimitSwitch.isPressed()) {
    armMotor.move_voltage(0);
} else {
    // Normal arm control
}
```

Next, let's add the logic to make the arm user-controller with two buttons on
the controller. First, we need to define our two controller buttons as
[ControllerButton](@ref okapi::ControllerButton) instances:

```cpp
ControllerButton armUpButton(ControllerDigital::A);
ControllerButton armDownButton(ControllerDigital::B);
```

Then we can use them along with our limit switch logic from above to control the
arm:

```cpp
// Don't power the arm if it is all the way down
if (armLimitSwitch.isPressed()) {
    armMotor.move_voltage(0);
} else {
    // Else, the arm isn't all the way down
    if (armUpButton.isPressed()) {
        armMotor.move_voltage(127);
    } else if (armDownButton.isPressed()) {
        armMotor.move_voltage(-127);
    } else {
        armMotor.move_voltage(0);
    }
}
```

## Autonomous Routine

To illustrate the closed-loop control method that
[ChassisController](@ref okapi::ChassisController) has, let's make a simple
autonomous routine to drive in a square.

Writing an autonomous routine is much easier when distances and turns can be
done with physical units, so let's configure the
[ChassisController](@ref okapi::ChassisController) with the clawbot chassis's
dimensions. This will require that we specify two additional parameters. The
first is the gearset of the motors on the chassis, in this example we will use
the standard green cartridges. The second is a
[list](http://www.cplusplus.com/reference/initializer_list/initializer_list/)
containing the wheel diameter (`4` inches) and the width of the chassis (`11.5`
inches).

```cpp
// Chassis Controller - lets us drive the robot around with open- or closed-loop control
auto drive = ChassisControllerBuilder()
                .withMotors(1, -10)
                .withGearset(AbstractMotor::gearset::green)
                .withDimensions({{4_in, 11.5_in}, imev5GreenTPR})
                .build();
```

After this, you can move the chassis in physical units, such as inches and
degrees:

```cpp
for (int i = 0; i < 4; i++) {
    drive->moveDistance(12_in); // Drive forward 12 inches
    drive->turnAngle(90_deg);   // Turn in place 90 degrees
}
```

## Wrap Up

This is the final product from this tutorial.

### Tank drive

```cpp
#include "okapi/api.hpp"
using namespace okapi;

void opcontrol() {
    // Chassis Controller - lets us drive the robot around with open- or closed-loop control
    auto drive = ChassisControllerBuilder()
                    .withMotors(1, -10)
                    .withGearset(AbstractMotor::gearset::green)
                    .withDimensions({{4_in, 11.5_in}, imev5GreenTPR})
                    .build();

    // Joystick to read analog values for tank or arcade control
    // Master controller by default
    Controller controller;

    // Arm related objects
    ADIButton armLimitSwitch('H');
    ControllerButton armUpButton(ControllerDigital::A);
    ControllerButton armDownButton(ControllerDigital::B);
    Motor armMotor = 8_rmtr;

    // Button to run our sample autonomous routine
    ControllerButton runAutoButton(ControllerDigital::X);

    while (true) {
        // Tank drive with left and right sticks
        drive->tank(controller.getAnalog(ControllerAnalog::leftY),
                    controller.getAnalog(ControllerAnalog::rightY));

        // Don't power the arm if it is all the way down
        if (armLimitSwitch.isPressed()) {
            armMotor.move_voltage(0);
        } else {
            // else, the arm isn't all the way down
            if (armUpButton.isPressed()) {
                armMotor.move_voltage(127);
            } else if (armDownButton.isPressed()) {
                armMotor.move_voltage(-127);
            } else {
                armMotor.move_voltage(0);
            }
        }

        // Run the test autonomous routine if we press the button
        if (runAutoButton.changedToPressed()) {
            // Drive the robot in a square pattern using closed-loop control
            for (int i = 0; i < 4; i++) {
                drive->moveDistance(12_in); // Drive forward 12 inches
                drive->turnAngle(90_deg);   // Turn in place 90 degrees
            }
        }

        // Wait and give up the time we don't need to other tasks.
        // Additionally, joystick values, motor telemetry, etc. all updates every 10 ms.
        pros::Task::delay(10);
    }
}
```

### Arcade drive

```cpp
#include "okapi/api.hpp"
using namespace okapi;

void opcontrol() {
    // Chassis Controller - lets us drive the robot around with open- or closed-loop control
    auto drive = ChassisControllerBuilder()
                    .withMotors(1, -10)
                    .withGearset(AbstractMotor::gearset::green)
                    .withDimensions({{4_in, 11.5_in}, imev5GreenTPR})
                    .build();

    // Joystick to read analog values for tank or arcade control
    // Master controller by default
    Controller controller;

    // Arm related objects
    ADIButton armLimitSwitch('H');
    ControllerButton armUpButton(ControllerDigital::A);
    ControllerButton armDownButton(ControllerDigital::B);
    Motor armMotor = 8_rmtr;

    // Button to run our sample autonomous routine
    ControllerButton runAutoButton(ControllerDigital::X);

    while (true) {
        // Arcade drive with the left stick
        drive->arcade(controller.getAnalog(ControllerAnalog::leftY),
                      controller.getAnalog(ControllerAnalog::rightY));

        // Don't power the arm if it is all the way down
        if (armLimitSwitch.isPressed()) {
            armMotor.move_voltage(0);
        } else {
            // else, the arm isn't all the way down
            if (armUpButton.isPressed()) {
                armMotor.move_voltage(127);
            } else if (armDownButton.isPressed()) {
                armMotor.move_voltage(-127);
            } else {
                armMotor.move_voltage(0);
            }
        }

        // Run the test autonomous routine if we press the button
        if (runAutoButton.changedToPressed()) {
            // Drive the robot in a square pattern using closed-loop control
            for (int i = 0; i < 4; i++) {
                drive->moveDistance(12_in); // Drive forward 12 inches
                drive->turnAngle(90_deg);   // Turn in place 90 degrees
            }
        }

        // Wait and give up the time we don't need to other tasks.
        // Additionally, joystick values, motor telemetry, etc. all updates every 10 ms.
        pros::Task::delay(10);
    }
}
```
