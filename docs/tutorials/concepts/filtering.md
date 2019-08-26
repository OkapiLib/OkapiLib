Filtering
=========

OkapiLib makes it easy to use any one of a number of various types of
filters on sensors and controllers. The specifics of how each filter
works and should be initialized will be left to its API reference, but
this guide will help provide the general knowledge necessary to make the
most out of OkapiLib's filtering functionality.

Filtering Generic Sensor Input
------------------------------

It's possible with OkapiLib to filter any value that you want, which
makes it easy to filter sensors. The example below gives an example of
filtering a sensor value.

```cpp
using namespace okapi;

const int POTENTIOMETER_PORT = 1;
const int NUM_AVE_POINTS = 5;

Potentiometer exampleSensor(POTENTIOMETER_PORT);
AverageFilter<NUM_AVE_POINTS> exampleFilter;

void opcontrol() {
  while (true) {
    std::cout << "Current Sensor Reading: " << exampleFilter.filter(exampleSensor.get());
    pros::Task::delay(10);
  }
}
```

The above example will print out the average of the last five readings
of the potentiometer.

Adding a Filter to a Controller
-------------------------------

Velocity PID Controllers often benefit from filtering the velocity
reading. As a result, it is possible to pass in a filter as an argument
to the constructor for a Velocity PID Controller. Note \-- filtering
will not have a positive impact on position PID movements, and is not
supported as a result.

Using a filter with a velocity PID Controller can be done in the
following manner:

```cpp
using namespace okapi;

const double kP = 0.001;
const double kD = 0.0001;
const double kF = 0.0;
const double kSF = 0.0;
const int NUM_AVE_POINTS = 5;

auto exampleController = IterativeControllerFactory::velPID(
                           kP, kD, kF, kSF,
                           VelMathFactory::createPtr(
                             imev5GreenTPR,
                             std::make_unique<AverageFilter<NUM_AVE_POINTS>>()
                           )
                         );

void opcontrol() {
}
```

This will create a velocity PID controller which uses an
`AverageFilter<NUM_AVE_POINTS>`.
