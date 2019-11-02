# Using the ChassisControllerBuilder

The [ChassisControllerBuilder](@ref okapi::ChassisControllerBuilder) is a
versatile and simple way to make a
[ChassisController](@ref okapi::ChassisController). This tutorial will give an
overview of how to use it and customize it for your robot.

Please read the preface on where to use builders:
[Where to use builders](docs/tutorials/concepts/builders-and-tasks.md)

## Configuring motors

The only required parameter is the motor configuration. Both skid-steer and
x-drive layouts are supported.

### Two motors by ports:

```cpp
#include "okapi/api.hpp"
using namespace okapi;

void opcontrol() {
auto drive = ChassisControllerBuilder()
                .withMotors(1, -2) // Left motor is 1, right motor is 2 (reversed)
                .build();
}
```

### Two motors:

```cpp
#include "okapi/api.hpp"
using namespace okapi;

void opcontrol() {
Motor leftMotor(1);
Motor rightMotor(-2);

auto drive = ChassisControllerBuilder()
                .withMotors(leftMotor, rightMotor)
                .build();
}
```

### More than two motors by ports:

```cpp
#include "okapi/api.hpp"
using namespace okapi;

void opcontrol() {
auto drive = ChassisControllerBuilder()
                .withMotors(
                    {-1, -2}, // Left motors are 1 & 2 (reversed)
                    {3, 4}    // Right motors are 3 & 4
                )
                .build();
}
```

### More than two motors by objects:

```cpp
#include "okapi/api.hpp"
using namespace okapi;

void opcontrol() {
MotorGroup leftMotors({1, 2});
MotorGroup rightMotors({-3, -4});

auto drive = ChassisControllerBuilder()
                .withMotors(leftMotors, rightMotors)
                .build();
}
```

## Configuring your gearset and robot dimensions

You should also configure the gearset and chassis dimensions to ensure that the gearsets in the
motors are correct and to enable commanding the robot in real-life units (inches, degrees, etc.).

### Gearset and dimensions:

```cpp
#include "okapi/api.hpp"
using namespace okapi;

void opcontrol() {
auto drive = ChassisControllerBuilder()
                .withMotors(1, -2) // Left motor is 1, right motor is 2 (reversed)
                // Green gearset, 4 inch wheel diameter, 11.5 inch wheelbase
                .withDimensions(AbstractMotor::gearset::green, {4_in, 11.5_in})
                .build();
}
```

### Gearset with an external ratio and dimensions:

```cpp
#include "okapi/api.hpp"
using namespace okapi;

void opcontrol() {
auto drive = ChassisControllerBuilder()
                .withMotors(1, -2) // Left motor is 1, right motor is 2 (reversed)
                // Green gearset, external ratio of (2.0 / 3.0), 4 inch wheel diameter, 11.5 inch wheelbase
                .withDimensions(AbstractMotor::gearset::green * (2.0 / 3.0), {4_in, 11.5_in})
                .build();
}
```

### Gearset and raw scales:

```cpp
#include "okapi/api.hpp"
using namespace okapi;

void opcontrol() {
auto drive = ChassisControllerBuilder()
                .withMotors(1, -2) // Left motor is 1, right motor is 2 (reversed)
                // Green gearset, straight scale of 1127.8696, turn scale of 2.875
                .withDimensions(AbstractMotor::gearset::green, {1127.8696, 2.875})
                .build();
}
```

## Configuring your sensors

If you do not use the motors' built-in encoders (e.g., you might use ADI
encoders), then you will need to pass those in as well.

### Encoders:

```cpp
#include "okapi/api.hpp"
using namespace okapi;

void opcontrol() {
auto drive = ChassisControllerBuilder()
                .withMotors(1, -2) // Left motor is 1, right motor is 2 (reversed)
                .withSensors(
                    {'A', 'B'}, // Left encoder in ADI ports A & B
                    {'C', 'D', true}  // Right encoder in ADI ports C & D (reversed)
                )
                .build();
}
```

## Configuring PID gains

If you want to use OkapiLib's PID control instead of the built-in control, you
need to pass in two or three sets of PID gains.

### Two sets:

```cpp
#include "okapi/api.hpp"
using namespace okapi;

void opcontrol() {
auto drive = ChassisControllerBuilder()
                .withMotors(1, -2) // Left motor is 1, right motor is 2 (reversed)
                .withGains(
                    {0.001, 0, 0.0001}, // Distance controller gains
                    {0.001, 0, 0.0001}  // Turn controller gains
                )
                .build();
}
```

### Three sets:

```cpp
#include "okapi/api.hpp"
using namespace okapi;

void opcontrol() {
auto drive = ChassisControllerBuilder()
                .withMotors(1, -2) // Left motor is 1, right motor is 2 (reversed)
                .withGains(
                    {0.001, 0, 0.0001}, // Distance controller gains
                    {0.001, 0, 0.0001}, // Turn controller gains
                    {0.001, 0, 0.0001}  // Angle controller gains (helps drive straight)
                )
                .build();
}
```

## Configuring derivative filters

If you are using OkapiLib's PID control instead of the built-in control, you can
pass in derivative term filters. These are applied to the PID controllers'
derivative terms to smooth them. If you use OkapiLib's PID control but do not
specify any derivative filters, the default filter is a
[PassthroughFilter](@ref okapi::PassthroughFilter).

### One filter:

```cpp
#include "okapi/api.hpp"
using namespace okapi;

void opcontrol() {
auto drive = ChassisControllerBuilder()
                .withMotors(1, -2) // Left motor is 1, right motor is 2 (reversed)
                .withGains(
                    {0.001, 0, 0.0001}, // Distance controller gains
                    {0.001, 0, 0.0001}  // Turn controller gains
                )
                .withDerivativeFilters(
                    std::make_unique<AverageFilter<3>>() // Distance controller filter
                )
                .build();
}
```

### Two filters:

```cpp
#include "okapi/api.hpp"
using namespace okapi;

void opcontrol() {
auto drive = ChassisControllerBuilder()
                .withMotors(1, -2) // Left motor is 1, right motor is 2 (reversed)
                .withGains(
                    {0.001, 0, 0.0001}, // Distance controller gains
                    {0.001, 0, 0.0001}  // Turn controller gains
                )
                .withDerivativeFilters(
                    std::make_unique<AverageFilter<3>>(), // Distance controller filter
                    std::make_unique<AverageFilter<3>>()  // Turn controller filter
                )
                .build();
}
```

### Three filters:

```cpp
#include "okapi/api.hpp"
using namespace okapi;

void opcontrol() {
auto drive = ChassisControllerBuilder()
                .withMotors(1, -2) // Left motor is 1, right motor is 2 (reversed)
                .withGains(
                    {0.001, 0, 0.0001}, // Distance controller gains
                    {0.001, 0, 0.0001}, // Turn controller gains
                    {0.001, 0, 0.0001}  // Angle controller gains (helps drive straight)
                )
                .withDerivativeFilters(
                    std::make_unique<AverageFilter<3>>(), // Distance controller filter
                    std::make_unique<AverageFilter<3>>(), // Turn controller filter
                    std::make_unique<AverageFilter<3>>()  // Angle controller filter
                )
                .build();
}
```

## Configuring maximum velocity and voltage

You can change the default maximum velocity or voltage.

### Max velocity:

```cpp
#include "okapi/api.hpp"
using namespace okapi;

void opcontrol() {
auto drive = ChassisControllerBuilder()
                .withMotors(1, -2) // Left motor is 1, right motor is 2 (reversed)
                .withMaxVelocity(100)
                .build();
}
```

### Max voltage:

```cpp
#include "okapi/api.hpp"
using namespace okapi;

void opcontrol() {
auto drive = ChassisControllerBuilder()
                .withMotors(1, -2) // Left motor is 1, right motor is 2 (reversed)
                .withMaxVoltage(10000)
                .build();
}
```

## Configuring the controller settling behavior

You can change the [SettledUtil](@ref okapi::SettledUtil) that a
[ClosedLoopController](@ref okapi::ClosedLoopController) gets when it is created by the builder,
in order to change the settling behavior of the chassis.

```cpp
#include "okapi/api.hpp"
using namespace okapi;

void opcontrol() {
auto drive = ChassisControllerBuilder()
                .withMotors(1, -2) // Left motor is 1, right motor is 2 (reversed)
                .withClosedLoopControllerTimeUtil(50, 5, 250_ms)
                .build();
}
```

## Configuring the Logger

If you want the [ChassisController](@ref okapi::ChassisController) and the
classes it creates to log what they are doing, either for debugging or other
purposes, you can supply a [Logger](@ref okapi::Logger).

### Log to the PROS terminal:

```cpp
#include "okapi/api.hpp"
using namespace okapi;

void opcontrol() {
auto drive = ChassisControllerBuilder()
                .withMotors(1, -2) // Left motor is 1, right motor is 2 (reversed)
                .withLogger(
                    std::make_shared<Logger>(
                        TimeUtilFactory::createDefault().getTimer(), // It needs a Timer
                        "/ser/sout", // Output to the PROS terminal
                        Logger::LogLevel::debug // Most verbose log level
                    )
                )
                .build();
}
```

### Log to the SD card:

```cpp
#include "okapi/api.hpp"
using namespace okapi;

void opcontrol() {
auto drive = ChassisControllerBuilder()
                .withMotors(1, -2) // Left motor is 1, right motor is 2 (reversed)
                .withLogger(
                    std::make_shared<Logger>(
                        TimeUtilFactory::createDefault().getTimer(), // It needs a Timer
                        "/usd/test_logging.txt", // Output to a file on the SD card
                        Logger::LogLevel::debug  // Most verbose log level
                    )
                )
                .build();
}
```
