# Odometry

Odometry is the act of tracking a robot's absolute position. 
This information can be used by motion algorithms to drive to positions or turn to absolute angles.

An excellent overview of the odometry tracking algorithm can be found [here](https://www.vexforum.com/t/team-5225-introduction-to-position-tracking-document/49640).

## Configuring Odometry

OkapiLib supports odometry for all chassis configurations, and is configured using 
[ChassisControllerBuilder](@ref okapi::ChassisControllerBuilder).

There are three general types of odometry supported by OkapiLib, though most feasible configurations are possible:
-  **Two Integrated Encoder**: Works out of the box with most chassis configurations and uses integrated motor encoders. 
Using the integrated encoders makes the odometry prone to wheel slip and inexact turning causing this odometry method to have limited accuracy.
This method cannot track sideways movement.
-  **Two External Encoder**: Requires two external tracking wheels which track the ground. Minimizes wheel slip and often improves turning accuracy. 
This method cannot track sideways movement.
-  **Three External Encoder**: Requires three external tracking wheels to measure the robot's movement in all directions. 
Recommended for a holonomic chassis or any skid-steer chassis using omni wheels. This method will have the highest accuracy.

## Odometry using integrated encoders:

If you are using integrated encoders, then the odometry scales are the same as your chassis scales.
Don't pass an extra [ChassisScales](@ref okapi::ChassisScales) to [withOdometry](@ref okapi::ChassisControllerBuilder::withOdometry). 
Here is an example using [ChassisControllerIntegrated](@ref okapi::ChassisControllerIntegrated):

```cpp
ChassisControllerBuilder()
    .withMotors(1, -2) // left motor is 1, right motor is 2 (reversed)
    // green gearset, 4 inch wheel diameter, 11.5 inch wheelbase
    .withDimensions(AbstractMotor::gearset::green, {{4_in, 11.5_in}, imev5GreenTPR})
    .withOdometry() // use the same scales as the chassis (above)
    .buildOdometry(); // build an odometry chassis
```

Here is the same example using [ChassisControllerPID](@ref okapi::ChassisControllerPID):

```cpp
ChassisControllerBuilder()
    .withMotors(1, -2) // left motor is 1, right motor is 2 (reversed)
    .withGains(
        {0.001, 0, 0.0001}, // Distance controller gains
        {0.001, 0, 0.0001}, // Turn controller gains
        {0.001, 0, 0.0001}  // Angle controller gains (helps drive straight)
    )
    // green gearset, 4 inch wheel diameter, 11.5 inch wheelbase
    .withDimensions(AbstractMotor::gearset::green, {{4_in, 11.5_in}, imev5GreenTPR})
    .withOdometry() // use the same scales as the chassis (above)
    .buildOdometry(); // build an odometry chassis
```

## Odometry using external encoders:

If you are using a [ChassisControllerIntegrated](@ref okapi::ChassisControllerIntegrated), 
the chassis dimensions will be different than the odometry scales given to 
[withDimensions](@ref okapi::ChassisControllerBuilder::withDimensions). This is because 
[ChassisControllerIntegrated](@ref okapi::ChassisControllerIntegrated) still requires scales for
the powered wheels, while the tracking wheels have a different set of scales. You will need to pass an extra 
[ChassisScales](@ref okapi::ChassisScales) to [withOdometry](@ref okapi::ChassisControllerBuilder::withOdometry) 
to specify the scales for the tracking wheels. 

```cpp
ChassisControllerBuilder()
    .withMotors(1, -2) // left motor is 1, right motor is 2 (reversed)
    // green gearset, 4 inch wheel diameter, 11.5 inch wheelbase
    .withDimensions(AbstractMotor::gearset::green, {{4_in, 11.5_in}, imev5GreenTPR})
    .withSensors(
        {'A', 'B'}, // left encoder in ADI ports A & B
        {'C', 'D', true}  // right encoder in ADI ports C & D (reversed)
    )
    // specify the tracking wheels diameter (3 in), track (7 in), and TPR (360)
    .withOdometry({{3_in, 7_in}, quadEncoderTPR})
    .buildOdometry();
```

If you are using a [ChassisControllerPID](@ref okapi::ChassisControllerPID) with external sensors, the odometry scales will be the same as
the ones given to [withDimensions](@ref okapi::ChassisControllerBuilder::withDimensions).

```cpp
ChassisControllerBuilder()
    .withMotors(1, -2) // left motor is 1, right motor is 2 (reversed)
    .withGains(
        {0.001, 0, 0.0001}, // distance controller gains
        {0.001, 0, 0.0001}, // turn controller gains
        {0.001, 0, 0.0001}  // angle controller gains (helps drive straight)
    )
    .withSensors(
        {'A', 'B'}, // left encoder in ADI ports A & B
        {'C', 'D', true}  // right encoder in ADI ports C & D (reversed)
    )
    // green gearset, 4 inch wheel diameter, 11.5 inch wheelbase
    .withDimensions(AbstractMotor::gearset::green, {{4_in, 11.5_in}, imev5GreenTPR})
    .withOdometry() // use the same scales as the chassis (above)
    .buildOdometry(); // build an odometry chassis
```

## Odometry using three tracking wheels:

[ThreeEncoderOdometry](@ref okapi::ThreeEncoderOdometry) is enabled by providing a third sensor to 
[withSensors](@ref okapi::ChassisControllerBuilder::withSensors). The scales then need to be expanded to incorporate the third sensor.
Here is an example using [ChassisControllerIntegrated](@ref okapi::ChassisControllerIntegrated): 

```cpp
ChassisControllerBuilder()
    .withMotors(1, -2) // left motor is 1, right motor is 2 (reversed)
    // green gearset, 4 inch wheel diameter, 11.5 inch wheelbase
    .withDimensions(AbstractMotor::gearset::green, {{4_in, 11.5_in}, imev5GreenTPR})
    .withSensors(
        {'A', 'B'}, // left encoder in ADI ports A & B
        {'C', 'D', true}  // right encoder in ADI ports C & D (reversed)
        {'E', 'F'}  // middle encoder in ADI ports E & F
    )
    // specify the tracking wheels diameter (3 in), track (7 in), and TPR (360)
    // specify the middle encoder distance (1 in) and diameter (2.75 in)
    .withOdometry({{3_in, 7_in, 1_in, 2.75_in}, quadEncoderTPR})
    .buildOdometry();
```

Here is the same example using [ChassisControllerPID](@ref okapi::ChassisControllerPID):

```cpp
ChassisControllerBuilder()
    .withMotors(1, -2) // left motor is 1, right motor is 2 (reversed)
    .withGains(
        {0.001, 0, 0.0001}, // distance controller gains
        {0.001, 0, 0.0001}, // turn controller gains
        {0.001, 0, 0.0001}  // angle controller gains (helps drive straight)
    )
    .withSensors(
        {'A', 'B'}, // left encoder in ADI ports A & B
        {'C', 'D', true}  // right encoder in ADI ports C & D (reversed)
        {'E', 'F'}  // middle encoder in ADI ports E & F
    )
    // green gearset, 4 inch wheel diameter, 11 inch wheelbase
    // 1 inch middle encoder distance, and 2.75 inch middle wheel diameter
    .withDimensions(AbstractMotor::gearset::green, {{4_in, 11_in, 1_in, 2.75_in}, imev5GreenTPR})
    .withOdometry() // use the same scales as the chassis (above)
    .buildOdometry(); // build an odometry chassis
```

## Using Odometry

The odometry controller built by [buildOdometry](@ref okapi::ChassisControllerBuilder::buildOdometry) is a 
[DefaultOdomChassisController](@ref okapi::DefaultOdomChassisController), which inherits from 
[OdomChassisController](@ref okapi::OdomChassisController). It supports all methods from 
[ChassisController](@ref okapi::ChassisController) such as 
[moveDistance](@ref okapi::DefaultOdomChassisController::moveDistance) or 
[turnAngle](@ref okapi::DefaultOdomChassisController::turnAngle), and adds odometry commands such as 
[driveToPoint](@ref okapi::DefaultOdomChassisController::driveToPoint), 
[turnToPoint](@ref okapi::DefaultOdomChassisController::turnToPoint), and 
[turnToAngle](@ref okapi::DefaultOdomChassisController::turnToAngle).

### Coordinates

OkapiLib supports two methods of representing a coordinate. which are defined in [StateMode](@ref okapi::StateMode). 
The default is [StateMode::FRAME_TRANSFORMATION](@ref okapi::StateMode::FRAME_TRANSFORMATION), 
where positive x is forward and positive y is to the right. You can specify the 
[StateMode](@ref okapi::StateMode) to be used for odometry as a parameter to 
[withOdometry](@ref okapi::ChassisControllerBuilder::withOdometry). 

To get and set the odometry state, use [getState](@ref okapi::OdomChassisController::getState) and 
[setState](@ref okapi::OdomChassisController::setState).

### Moving

OkapiLib moves by calculating necessary chassis movement given a desired state and an odometry state, and 
then forwarding those commands to a [ChassisController](@ref okapi::ChassisController).
For example, typing

```cpp
chassis->driveToPoint({1_ft, 1_ft}); // assume starting position of {0, 0, 0}
```

is equivalent to typing
```cpp
chassis->turnAngle(45_deg);
chassis->moveDistance(1.4_ft); // sqrt(1^2 + 1^2)
```

The advantage of using these algorithms is that if the odometry state is slightly different than expected, 
OkapiLib will try to compensate so that the next position is accurate. Programming using odometry is much nicer, 
as the user can visualize the field and changing a movement will not affect the following movements.

## Full Example:

Here is is a full example of odometry using [ChassisControllerIntegrated](@ref okapi::ChassisControllerIntegrated) and two tracking wheels: 

```cpp
auto chassis =
  ChassisControllerBuilder()
    .withMotors(1, -2) // left motor is 1, right motor is 2 (reversed)
    // green gearset, 4 inch wheel diameter, 11.5 inch wheelbase
    .withDimensions(AbstractMotor::gearset::green, {{4_in, 11.5_in}, imev5GreenTPR})
    // left encoder in ADI ports A & B, right encoder in ADI ports C & D (reversed)
    .withSensors({'A', 'B'}, {'C', 'D', true})
    // specify the tracking wheels diameter (3 in), track (7 in), and TPR (360)
    .withOdometry({{3_in, 7_in}, quadEncoderTPR}, StateMode::FRAME_TRANSFORMATION)
    .buildOdometry();

// set the state to zero
chassis->setState({0_in, 0_in, 0_deg});
// turn 45 degrees and drive approximately 1.4 ft
chassis->driveToPoint({1_ft, 1_ft});
// turn approximately 45 degrees to end up at 90 degrees
chassis->turnToAngle(90_deg);
// turn approximately -90 degrees to face the point
chassis->turnToPoint({5_ft, 0_ft});
```

