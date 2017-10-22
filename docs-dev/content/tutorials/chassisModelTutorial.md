## Chassis Model

Okapi has a way to represent the most common robot chassis, skid steer and x-drive. With this model we can tell the robot how to move around in a general sense.

For example, a skid steer chassis with two motors per side and two quad encoders:

```c++
#include <chassis/chassisModel.h>

SkidSteerModel<2> model({2_m, 3_m, 4_m, 5_m}, //Left motors: 2 & 3, right motors: 4 & 5
                        QuadEncoder(1, 2, true), //Left encoder (reversed)
                        QuadEncoder(3, 4)); //Right encoder
```

Or you can have an x-drive with one motor per corner and two quad encoders:

```c++
#include <chassis/chassisModel.h>

XDriveModel<1> model({2_m, 3_m, 4_m, 5_m}, //Motors are ordered counter-clockwise from the top left
                        QuadEncoder(1, 2, true), //Top left encoder (reversed)
                        QuadEncoder(3, 4)); //Top right encoder
```

Then once we have a model, we can do everything a generic `ChassisModel` can do:

```c++
model.driveForward(100); //Drive forward at 100 power
model.turnClockwise(90); //Turn clockwise at 90 power
model.tank(joystickGetAnalog(1, 4), joystickGetAnalog(1, 3)); //Tank drive on the vertical channels
model.stop(); //Set all motors to 0
```

 If we wanted to use some sort of closed-loop control, we could either do it ourselves or use one of Okapi's structures instead, covered in the `ChassisController` or `OdomChassisController` tutorials.
