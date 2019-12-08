# 2D Motion Profiling

Motion Profiling is a controls algorithm that defines a movement as a series of
steps. In one dimension, it defines a motor's position, speed, acceleration,
etc. at every timestep during the movement. In two dimensions, it defines your
robot's position, speed, acceleration, etc. and heading at every timestep during
the movement.

There is a lot of interesting math that goes into calculating these values,
which you can learn about in examples like these:

- <http://www2.informatik.uni-freiburg.de/~lau/students/Sprunk2008.pdf>
- <https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathPlanning/QuinticPolynomialsPlanner/animation.gif>
- <https://www.dis.uniroma1.it/~deluca/rob1_en/13_TrajectoryPlanningJoints.pdf>

OkapiLib, through [Pathfinder](https://github.com/JacisNonsense/Pathfinder),
does this math for you and ties both the motion profiling and profile following
actions to the same
[async controllers](docs/tutorials/walkthrough/asyncAutonomousMovement.md) that are used
for other simpler movements. Because OkapiLib's motion profile generation uses
[Pathfinder](https://github.com/JacisNonsense/Pathfinder), there are a few open
issues you should know about:

- Pathfinder cannot generate negative velocities, so backward movements and very tight turns do not work
  - Moving backwards: <https://github.com/JacisNonsense/Pathfinder/issues/39>
  - Tight turns: <https://github.com/JacisNonsense/Pathfinder/issues/38>
- Very long movements (typically movements much longer than a VEX field) can potentially never reach maximum speed: <https://github.com/JacisNonsense/Pathfinder/issues/43>

First, let's initialize a [ChassisController](@ref okapi::ChassisController) to
pass into the
[AsyncMotionProfileControllerBuilder](@ref okapi::AsyncMotionProfileControllerBuilder):

> The \"linear\" properties of the chassis given to
> [AsyncMotionProfileControllerBuilder::withLimits](@ref okapi::AsyncMotionProfileControllerBuilder::withLimits)
> are for its forward/backwards movements, not the maximum properties for
> turning or other movements.


```cpp
using namespace okapi;

auto myChassis = ChassisControllerBuilder()
                   .withMotors({1, 2}, {-3, -4})
                   // Green gearset, 4 in wheel diam, 11.5 in wheel track
                   .withDimensions(AbstractMotor::gearset::green, {{4_in, 11.5_in}, imev5GreenTPR})
                   .build();

auto profileController = AsyncMotionProfileControllerBuilder()
                           .withLimits({1.0, 2.0, 10.0})
                           .withOutput(myChassis)
                           .buildMotionProfileController();
```

Next, let's create a motion profile. A profile is created with a list of points
and a name. Each of the points contains the desired coordinates and heading.
The points given to the controller form a path from the first given
point to the last, with the first point assumed to be the current position of
the robot. The path will be generated assuming that all of the points are
relative to the first, so if you pass in an x value of `-4_ft` for the first
point, then passing an x value of `0_ft` for the second point will result in a
forward movement of `4` feet.

> This function computes the set of steps for the desired profile, which may
> take some time.

```cpp
profileController->generatePath({
  {0_ft, 0_ft, 0_deg},  // Profile starting position, this will normally be (0, 0, 0)
  {3_ft, 0_ft, 0_deg}}, // The next point in the profile, 3 feet forward
  "A" // Profile name
);
```

After the profile is created, it is added to a map of available profiles
stored in the controller. You can then set a target using the name you
gave the profile.


```cpp
profileController->setTarget("A");
```

And then as with any [AsyncController](@ref okapi::AsyncController), you can
call
[AsyncController::waitUntilSettled](@ref okapi::AsyncController::waitUntilSettled)
to block program execution until the movement is finished and the robot reaches
the desired end point.

```cpp
profileController->waitUntilSettled();
```

## Wrap-up

In total, here is how to initialize and use a 2D motion profiling controller:

```cpp
using namespace okapi;

auto myChassis = ChassisControllerBuilder()
                   .withMotors({1, 2}, {-3, -4})
                   // Green gearset, 4 in wheel diam, 11.5 in wheel track
                   .withDimensions(AbstractMotor::gearset::green, {{4_in, 11.5_in}, imev5GreenTPR})
                   .build();

auto profileController = AsyncMotionProfileControllerBuilder()
                           .withLimits({1.0, 2.0, 10.0})
                           .withOutput(myChassis)
                           .buildMotionProfileController();

void opcontrol() {
  profileController->generatePath({{0_ft, 0_ft, 0_deg}, {3_ft, 0_ft, 0_deg}}, "A");
  profileController->setTarget("A");
  profileController->waitUntilSettled();
}
```
