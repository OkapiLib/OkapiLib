## ChassisModel (abstract)

The `ChassisModel` class is an interface to a robot's chassis: it provides methods to control the chassis and to read from standard sensors placed on most chassis (i.e., quadrature encoders).

### Constructor

```c++
//Signature
ChassisModel()
```

The constructor does not take any parameters.

### driveForward

```c++
//Signature
virtual void driveForward(const int power) = 0
```

Drives the chassis forwards by setting all motors to the input power. A positive value for `power` should cause all chassis wheels to move the robot forward in a straight line.

Parameter | Description
----------|------------
power | The raw motor power sent directly to the chassis' motors

### turnClockwise

```c++
//Signature
virtual void turnClockwise(const int power) = 0
```

Turns the robot clockwise by setting the left side motors to the input power and the right side motors to the negative of the input power. A positive value for `power` should cause all chassis wheels to turn the robot clockwise on a point.

Parameter | Description
----------|------------
power | The raw motor power sent directly to the chassis' motors

### driveVector

```c++
//Signature
virtual void driveVector(const int distPower, const int anglePower) = 0

driveVector(127, 0) //Same as driveForward(127)
driveVector(0, 127) //Same as turnClockwise(127)

//Make a moderate swing turn clockwise
//left motors get 110 power, right motors get 70 power
driveVector(90, 20)
```

Drive the chassis along a curved path. Calling `driveVector(127, 0)` should be equivalent to calling `driveForward(127)`; calling `driveVector(0, 127)` should be equivalent to calling `turnClockwise(127)`. A mix between the two will cause the robot to make a swing turn.

Parameter | Description
----------|------------
distPower | The motor power making up the "straight" component of the final motor power
anglePower | The motor power making up the "turn" component of the final motor power

### tank

```c++
//Signature
virtual void tank(const int leftVal, const int rightVal) = 0;
```

Power the motors like tank drive.

Parameter | Description
----------|------------
leftVal | Motor power for the left side motors
rightVal | Motor power for the right side motors

### arcade

```c++
//Signature
virtual void arcade(const int verticalVal, const int horizontalVal) = 0;
```

Power the motors like arcade drive.

Parameter | Description
----------|------------
verticalVal | Motor power for the vertical component of movement
horizontalVal | Motor power for the horizontal component of movement

### left

```c++
//Signature
virtual void left(const int val) = 0;
```

Power the left side motors.

Parameter | Description
----------|------------
val | Motor power

### right

```c++
//Signature
virtual void right(const int val) = 0;
```

Power the right side motors.

Parameter | Description
----------|------------
val | Motor power

### getEncoderVals

Reads the encoders given to the chassis model at construction time and returns them in the format `{left encoder value, right encoder value}`. Return type is a `std::valarray` because it features operator overrides for common math operations making encoder math easy.

```c++
//Signature
virtual std::valarray<int> getEncoderVals() const = 0
```
