# Using the ChassisControllerBuilder

The [ChassisControllerBuilder](@ref okapi::ChassisControllerBuilder) is a
versatile and simple way to make a
[ChassisController](@ref okapi::ChassisController). This tutorial will give an
overview of how to use it and customize it for your robot.

Please read the preface on where to use builders:
[Where to use builders](docs/tutorials/concepts/builders-and-tasks.md)

## Configuring motors

You must configure the motors. Skid-steer, x-drive, and h-drive configurations are supported.

### Two motor skid-steer:

```cpp
ChassisControllerBuilder()
    .withMotors(1, -2) // Left motor is 1, right motor is 2 (reversed)
```

### Four motor skid-steer:

You may also use [MotorGroups](@ref okapi::MotorGroup). You are not required to have an equal
number of motors per side. There is no limit on the number of motors per side.

```cpp
ChassisControllerBuilder()
    .withMotors(
        {-1, -2}, // Left motors are 1 & 2 (reversed)
        {3, 4}    // Right motors are 3 & 4
    )
```

### X-Drive:

You may also use [MotorGroups](@ref okapi::MotorGroup).

```cpp
ChassisControllerBuilder()
    .withMotors(
        1,  // Top left
        -2, // Top right (reversed)
        -3, // Bottom right (reversed)
        4   // Bottom left
    )
```

### H-Drive:

You may also use [MotorGroups](@ref okapi::MotorGroup).

```cpp
ChassisControllerBuilder()
    .withMotors(
        1,  // Left side
        -2, // Right side (reversed)
        3   // Middle
    )
```

## Configuring your gearset and robot dimensions

You must configure the gearset and chassis dimensions to ensure that the gearsets in the
motors are correct and to enable commanding the robot in real-life units (inches, degrees, etc.).
If you specified non-integrated encoders using `withSensors` then `withDimensions` will refer to the
wheels those encoders are attached to. Otherwise, `withDimensions` will refer to the driven wheels.

### Gearset and dimensions:

```cpp
ChassisControllerBuilder()
    // Green gearset, 4 inch wheel diameter, 11.5 inch wheelbase
    .withDimensions(AbstractMotor::gearset::green, {{4_in, 11.5_in}, imev5GreenTPR})
```

### Gearset with an external ratio and dimensions:

```cpp
ChassisControllerBuilder()
    // Green gearset, external ratio of (2.0 / 3.0), 4 inch wheel diameter, 11.5 inch wheelbase
    .withDimensions(AbstractMotor::gearset::green, {{4_in, 11.5_in}, imev5GreenTPR * (2.0 / 3.0)})
```

### Gearset and raw scales:

```cpp
ChassisControllerBuilder()
    // Green gearset, straight scale of 1127.8696, turn scale of 2.875
    .withDimensions(AbstractMotor::gearset::green, {{1127.8696, 2.875}, imev5GreenTPR})
```

## Configuring your sensors

If you do not use the motors' built-in encoders (e.g., you might use ADI encoders or rotation sensors), then you will
need to pass those in as well. These sensors will not affect the
[ChassisControllerIntegrated](@ref okapi::ChassisControllerIntegrated) controls because it uses the
integrated control (and therefore, the encoders built-in to the motors).

### Rotation Sensors

```cpp
ChassisControllerBuilder()
    .withSensors(
        RotationSensor{1}, // Left encoder in V5 port 1
        RotationSensor{2, true}  // Right encoder in V5 port 2 (reversed)
    )
```

### ADI Encoders

```cpp
ChassisControllerBuilder()
    .withSensors(
        ADIEncoder{'A', 'B'}, // Left encoder in ADI ports A & B
        ADIEncoder{'C', 'D', true}  // Right encoder in ADI ports C & D (reversed)
    )
```

## Configuring odometry

OkapiLib supports odometry for all chassis configurations.
For more information about odometry, read the [Odometry Tutorial](docs/tutorials/walkthrough/odometry.md).

### Odometry using integrated encoders:

If you don't have external sensors, don't pass an extra [ChassisScales](@ref okapi::ChassisScales)
to [withOdometry](@ref okapi::ChassisControllerBuilder::withOdometry).

```cpp
ChassisControllerBuilder()
    .withMotors(1, -2) // Left motor is 1, right motor is 2 (reversed)
    // Green gearset, 4 inch wheel diameter, 11.5 inch wheelbase
    .withDimensions(AbstractMotor::gearset::green, {{4_in, 11.5_in}, imev5GreenTPR})
    .withOdometry() // Use the same scales as the chassis (above)
    .buildOdometry()
```

### Odometry using external encoders:

If you have external sensors, you need to pass an extra [ChassisScales](@ref okapi::ChassisScales)
to [withOdometry](@ref okapi::ChassisControllerBuilder::withOdometry) to specify the dimensions
for the tracking wheels. If you are using a
[ChassisControllerPID](@ref okapi::ChassisControllerPID), these dimensions will be the same as
the ones given to [withDimensions](@ref okapi::ChassisControllerBuilder::withDimensions), so you do
not need to pass any dimensions to [withOdometry](@ref okapi::ChassisControllerBuilder::withOdometry).
If you are using a [ChassisControllerIntegrated](@ref okapi::ChassisControllerIntegrated), these
dimensions will be different than the ones given to
[withDimensions](@ref okapi::ChassisControllerBuilder::withDimensions).

```cpp
ChassisControllerBuilder()
    .withMotors(1, -2) // Left motor is 1, right motor is 2 (reversed)
    // Green gearset, 4 inch wheel diameter, 11.5 inch wheelbase
    .withDimensions(AbstractMotor::gearset::green, {{4_in, 11.5_in}, imev5GreenTPR})
    .withSensors(
        ADIEncoder{'A', 'B'}, // Left encoder in ADI ports A & B
        ADIEncoder{'C', 'D', true}  // Right encoder in ADI ports C & D (reversed)
    )
    // Specify the tracking wheels diam (2.75 in), track (7 in), and TPR (360)
    .withOdometry({{2.75_in, 7_in}, quadEncoderTPR})
    .buildOdometry()
```

## Configuring PID gains

If you want to use OkapiLib's PID control instead of the built-in control, you
need to pass in two or three sets of PID gains.

### Two sets:

```cpp
ChassisControllerBuilder()
    .withGains(
        {0.001, 0, 0.0001}, // Distance controller gains
        {0.001, 0, 0.0001}  // Turn controller gains
    )
```

### Three sets:

```cpp
ChassisControllerBuilder()
    .withGains(
        {0.001, 0, 0.0001}, // Distance controller gains
        {0.001, 0, 0.0001}, // Turn controller gains
        {0.001, 0, 0.0001}  // Angle controller gains (helps drive straight)
    )
```

## Configuring derivative filters

If you are using OkapiLib's PID control instead of the built-in control, you can
pass in derivative term filters. These are applied to the PID controllers'
derivative terms to smooth them. If you use OkapiLib's PID control but do not
specify any derivative filters, the default filter is a
[PassthroughFilter](@ref okapi::PassthroughFilter).

### One filter:

```cpp
ChassisControllerBuilder()
    .withDerivativeFilters(
        std::make_unique<AverageFilter<3>>() // Distance controller filter
    )
```

### Two filters:

```cpp
ChassisControllerBuilder()
    .withDerivativeFilters(
        std::make_unique<AverageFilter<3>>(), // Distance controller filter
        std::make_unique<AverageFilter<3>>()  // Turn controller filter
    )
```

### Three filters:

```cpp
ChassisControllerBuilder()
    .withDerivativeFilters(
        std::make_unique<AverageFilter<3>>(), // Distance controller filter
        std::make_unique<AverageFilter<3>>(), // Turn controller filter
        std::make_unique<AverageFilter<3>>()  // Angle controller filter
    )
```

## Configuring maximum velocity and voltage

You can change the default maximum velocity or voltage.

### Max velocity:

The default max velocity depends on the [gearset](@ref okapi::AbstractMotor::gearset).

```cpp
ChassisControllerBuilder()
    .withMaxVelocity(100)
```

### Max voltage:

The default max voltage is `12000`.

```cpp
ChassisControllerBuilder()
    .withMaxVoltage(10000)
```

## Configuring the controller settling behavior

You can change the [SettledUtil](@ref okapi::SettledUtil) that a
[ClosedLoopController](@ref okapi::ClosedLoopController) gets when it is created by the builder,
in order to change the settling behavior of the chassis.

```cpp
ChassisControllerBuilder()
    .withClosedLoopControllerTimeUtil(50, 5, 250_ms)
```

## Configuring the Logger

If you want the [ChassisController](@ref okapi::ChassisController) and the classes it creates to log
what they are doing, either for debugging or other purposes, you can supply a
[Logger](@ref okapi::Logger). You can also
[enable logging globally](docs/tutorials/concepts/logging.md).

### Log to the PROS terminal:

```cpp
ChassisControllerBuilder()
    .withLogger(
        std::make_shared<Logger>(
            TimeUtilFactory::createDefault().getTimer(), // It needs a Timer
            "/ser/sout", // Output to the PROS terminal
            Logger::LogLevel::debug // Most verbose log level
        )
    )
```

### Log to the SD card:

```cpp
ChassisControllerBuilder()
    .withLogger(
        std::make_shared<Logger>(
            TimeUtilFactory::createDefault().getTimer(), // It needs a Timer
            "/usd/test_logging.txt", // Output to a file on the SD card
            Logger::LogLevel::debug  // Most verbose log level
        )
    )
```
