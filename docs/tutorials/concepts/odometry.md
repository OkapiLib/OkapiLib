# Odometry

Odometry is the act of tracking a robot's absolute position. 
This information can be used by motion algorithms to drive to positions or turn to absolute angles.

An excellent overview of the odometry tracking algorithm can be found [here](https://www.vexforum.com/t/team-5225-introduction-to-position-tracking-document/49640).

## Configuring Odometry

OkapiLib supports odometry for all chassis configurations, and is configured using the 
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
    .buildOdometry() // build an odometry chassis
```

Here is the same example using [ChassisControllerPID](@ref okapi::ChassisControllerPID).

```cpp
ChassisControllerBuilder()
    .withMotors(1, -2) // left motor is 1, right motor is 2 (reversed)
    .withGains(
        {0.001, 0, 0.0001}, // Distance controller gains
        {0.001, 0, 0.0001}, // Turn controller gains
        {0.001, 0, 0.0001}  // Angle controller gains (helps drive straight)
    )
    // green gearset, 4 inch wheel diameter, 11.5 inch wheelbasey
    .withDimensions(AbstractMotor::gearset::green, {{4_in, 11.5_in}, imev5GreenTPR})
    .withOdometry() // use the same scales as the chassis (above)
    .buildOdometry() // build an odometry chassis
```

