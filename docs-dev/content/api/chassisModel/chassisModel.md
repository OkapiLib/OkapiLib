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

Drives the chassis forwards by setting all motors to the input power. A positive value for `power` should cause all chassis wheels to move the robot forward in a straight line. Uses truespeed.

Parameter | Description
----------|------------
power | The raw motor power sent directly to the chassis' motors

### turnClockwise

```c++
//Signature
virtual void turnClockwise(const int power) = 0
```

Turns the robot clockwise by setting the left side motors to the input power and the right side motors to the negative of the input power. A positive value for `power` should cause all chassis wheels to turn the robot clockwise on a point. Uses truespeed.

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

Drive the chassis along a curved path. Calling `driveVector(127, 0)` should be equivalent to calling `driveForward(127)`; calling `driveVector(0, 127)` should be equivalent to calling `turnClockwise(127)`. A mix between the two will cause the robot to make a swing turn. Uses truespeed.

Parameter | Description
----------|------------
distPower | The motor power making up the "straight" component of the final motor power
anglePower | The motor power making up the "turn" component of the final motor power

### tank

```c++
//Signature
virtual void tank(const int leftVal, const int rightVal, const int threshold = 0) = 0;
```

Power the motors like tank drive.

Parameter | Description
----------|------------
leftVal | Motor power for the left side motors
rightVal | Motor power for the right side motors
threshold | Motor power below this threshold will become zero

### arcade

```c++
//Signature
virtual void arcade(int verticalVal, int horizontalVal, const int threshold = 0) = 0;
```

Power the motors like arcade drive.

Parameter | Description
----------|------------
verticalVal | Motor power for the vertical component of movement
horizontalVal | Motor power for the horizontal component of movement
threshold | Motor power below this threshold will become zero

### left

```c++
//Signature
virtual void left(const int val) = 0;
```

Power the left side motors.

Parameter | Description
----------|------------
val | Motor power

### leftTS

```c++
//Signature
virtual void leftTS(const int val) = 0;
```

Power the left side motors using truespeed.

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

### rightTS

```c++
//Signature
virtual void rightTS(const int val) = 0;
```

Power the right side motors using truespeed.

Parameter | Description
----------|------------
val | Motor power

### getEncoderVals

```c++
//Signature
virtual std::valarray<int> getSensorVals() = 0
```

Reads the sensors given to the chassis model at construction time and returns them in the format `{left sensor value, right sensor value}`. Return type is a `std::valarray` because it features operator overrides for common math operations making encoder math easy.

### resetSensors

```c++
//Signature
virtual void resetSensors() = 0
```

Reset the sensors to zero.
