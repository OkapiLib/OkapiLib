## Chassis Controller

Okapi provides a way to control a `ChassisModel` using closed-loop control, like PID, so you don't have to rewrite a driveStraight() function every year. Rather than making a model directly, we can instead give `ChassisModelParams` to a `ChassisController`, which will instantiate the model for itself:

```c++
#include <chassis/chassisController.h>
ChassisControllerPid controller(SkidSteerModelParams<2>({2_m, 3_m, 4_m, 5_m}, //Skid steer chassis with two motors per side
                                                        QuadEncoder(1, 2, true), //Left encoder (reversed)
                                                        QuadEncoder(3, 4)), //Right encoder
                                PidParams(0.15, 0.05, 0.07), //Distance PID controller
                                PidParams(0.02, 0.01, 0)); //Angle PID controller
```

A `ChassisControllerPid` implements the `ChassisController` interface using PID control. It takes two `PidParams` to describe the controller gains for both controllers. The first controls distance to the target, but the second keeps the robot on a straight path while it drives to its target. Often times the second controller's gains will have lower kP, lower or no kD, and moderate kI because this controller just needs to make slight adjustments to the power the motors on each side of the chassis get.

Once we have our controller, we can control the robot more accurately than with just a model:

```c++
controller.driveStraight(100); //Drive in a straight line for 100 encoder ticks
controller.pointTurn(100); //Turn clockwise in place for 100 encoder ticks
```
