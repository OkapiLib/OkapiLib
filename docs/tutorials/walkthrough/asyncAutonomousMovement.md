# Asynchronous Autonomous Movement

Oftentimes the fastest way to move in autonomous involves actuating multiple
subsystems at once (i.e. driving and raising/lowering a lift). This is made
possible with Async Controllers.

To create a [ChassisController](@ref okapi::ChassisController) for a given system,
modify the below example to fit your subsystem.

```cpp
using namespace okapi;

auto driveController = ChassisControllerBuilder()
                        .withMotors(1, -2)
                        // Green gearset, 4 in wheel diam, 11.5 in wheel track
                        .withDimensions(AbstractMotor::gearset::green, {{4_in, 11.5_in}, imev5GreenTPR})
                        .build();
```

And then we'll add a lift subsystem as an Async Controller:

```cpp
const double liftkP = 0.001;
const double liftkI = 0.0001;
const double liftkD = 0.0001;

auto liftController = AsyncPosControllerBuilder()
                        .withMotor(3) // lift motor port 3
                        .withGains({liftkP, liftkI, liftkD})
                        .build();
```

Now that we have two subsystems to run, let's execute a few different movements.
If we want both systems to move, and the next movement in the autonomous routine
only depends on the drive completing its movement (and it doesn't care about the
lift's status), we'll run
[waitUntilSettled()](@ref okapi::ChassisController::waitUntilSettled) with just
the drive controller.

```cpp
// Begin movements
driveController->moveDistanceAsync(10_in); // Move 10 inches forward
liftController->setTarget(200); // Move 200 motor degrees upward
driveController->waitUntilSettled();

// Then the next movement will execute after the drive movement finishes
```

If blocking the next movement with regard only to the lift is desired, swap
`driveController` for `liftController` in the last line. If both movements need
to finish before executing the next movement, then call
[waitUntilSettled()](@ref okapi::ChassisController::waitUntilSettled) on both
controllers.

```cpp
// Begin movements
driveController->moveDistanceAsync(10_in); // Move 10 inches forward
liftController->setTarget(200); // Move 200 motor degrees upward
driveController->waitUntilSettled();
liftController->waitUntilSettled();

// Then the next movement will execute after both movements finish
```
