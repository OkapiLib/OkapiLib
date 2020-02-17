# Moving Autonomously

Arguably the most fundamental task with regard to creating a good autonomous
routine is ensuring consistent and accurate movement of the chassis. Robotic
autonomous movement is an unsolved problem even among professional engineers,
so it is obviously a difficult task. OkapiLib makes it easy to get reasonably
accurate autonomous movements.

The basis for this autonomous movement is the
[ChassisController](@ref okapi::ChassisController) class. Take a look at its API
for more detailed info on it. We'll use a
[ChassisControllerIntegrated](@ref okapi::ChassisControllerIntegrated) for this
tutorial; using the V5 motors' onboard PID makes setup a much quicker and easier
process (No PID tuning needed!).

Let's start by creating the
[ChassisControllerIntegrated](@ref okapi::ChassisControllerIntegrated) with
drive motors in ports 1 and 2:

```cpp
std::shared_ptr<ChassisController> chassis =
  ChassisControllerBuilder()
    .withMotors(1, -2)
    // Green gearset, 4 in wheel diam, 11.5 in wheel track
    .withDimensions(AbstractMotor::gearset::green, {{4_in, 11.5_in}, imev5GreenTPR})
    .build();
```

Now that we've created a ChassisController, let's start moving around. There are
two fundamental movement types:
[moveDistance](@ref okapi::ChassisController::moveDistance) and
[turnAngle](@ref okapi::ChassisController::turnAngle), for moving
forward/backward and turning on a point.

```cpp
// Move 1 meter to the first goal
chassis->moveDistance(1_m);
// Turn 90 degrees to face second goal
chassis->turnAngle(90_deg);
// Drive 1 and a half feet toward second goal
chassis->moveDistance(1.5_ft);
```
