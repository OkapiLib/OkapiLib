Iterative and Async Controllers
===============================

OkapiLib provides two main types of feedback controllers, **Iterative
Controllers** and **Async Controllers**. The two both accomplish the
same end goal \-- controlling a given system \-- but in different
manners that makes them tailored for different applications. You can
achieve the same response from a system with either option, but one will
likely be better than the other for your particular preference with code
organization.

Iterative Controllers
---------------------

**Iterative Controllers** operate **discretely**, meaning that an
Iterative Controller will produce an output for the given input *at a
single point in time* (see  IterativeController#step()).
If you want to execute a full movement for a system with an Iterative
Controller, you will need to write a loop that runs
`IterativeController::step()` repeatedly until the movement is finished.

An example movement:

``` {.cpp}
using namespace okapi;

const double kP = 0.001;
const double kI = 0.0001;
const double kD = 0.0001;
const int MOTOR_PORT = 1;
const double TARGET = 100.0;

auto exampleController = IterativeControllerFactory::posPID(kP, kI, kD);
Motor exampleMotor(MOTOR_PORT);

void opcontrol() {
  // Execute the movement
  exampleController.setTarget(TARGET);
  while (!exampleController.isSettled()) {
    double newInput = exampleMotor.getPosition();
    double newOutput = exampleController.step(newInput);
    exampleMotor.controllerSet(newOutput);

    pros::delay(10); // Run the control loop at 10ms intervals
  }
}
```

Async Controllers
-----------------

**Async Controllers** were given their name because they can operate
**asynchronously** to one another, which means practically that you can
have multiple movements by multiple controllers operating at the same
time easily. Each Async Controller operates in its own task, so starting
a movement for one Async Controller won't prevent another Async
Controller (or Iterative Controller) from running immediately
thereafter.

You don't need to run a loop to generate and set the controller output
like with an Iterative Controller, and blocking further movements until
an Async Controller's movement is done is as simple as a call to
`AsyncController::waitUntilSettled()`.

An example movement:

``` {.cpp}
using namespace okapi;

const double kP = 1.0;
const double kI = 0.001;
const double kD = 0.1;
const int MOTOR_PORT = 1;
const double TARGET = 100.0;

auto exampleController = AsyncPosControllerBuilder()
                           .withMotor(MOTOR_PORT)
                           .withGains({kP, kI, kD})
                           .build();

void opcontrol() {
  // Execute the movement
  exampleController->setTarget(TARGET);
  exampleController->waitUntilSettled();
}
```

When Should I Use Which Controller?
-----------------------------------

Async Controllers are obviously the easiest to work with for normal
movements, as seen in the above example code. If you just want to run a
system or systems in the manner shown in the examples, that's probably
your best choice. If you want to collect extra data during the movement
(such as motor stats like current, temperature, etc.), then you will
need to use an Iterative Controller to collect that data. Additionally,
if you want to use the output from two controllers to set the output for
your system (i.e. running both a heading PID and a forward/backward PID
on a drivetrain), the using two Iterative Controllers would be the best
idea.

To conclude, for most applications an Async Controller should suffice,
but if you want more complex behavior, then use an Iterative Controller.
