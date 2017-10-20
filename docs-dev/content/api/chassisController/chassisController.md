## ChassisController (abstract)

The `ChassisController` class an interface for controlling a robot's chassis: it provides methods that build upon the basic methods `ChassisModel` has for more accurate control.

### Constructor

```c++
//Signature
ChassisController(const ChassisModelParams& imodelParams)
ChassisController(std::shared_ptr<ChassisModel> imodel)
```

Parameter | Description
----------|------------
imodelParams | `ChassisModelParams` (used to make a new `ChassisModel`)
imodel | An existing `ChassisModel`

### driveStraight

```c++
//Signature
virtual void driveStraight(const int itarget) = 0
```

Drive the robot straight for a distance of `itarget` in the units of `itarget`.

Parameter | Description
----------|------------
itarget | Distance for the robot to travel

### pointTurn

```c++
//Signature
virtual void pointTurn(const float idegTarget) = 0
```

Turn the robot in place for an angle of `idegTarget`. The units of the angle travel is most often the difference in encoder ticks between the two sides of the chassis.

### driveForward

```c++
//Signature
void driveForward(const int power)
```

Passthrough function to call `driveForward` on the internal `ChassisModel`.

### driveVector

```c++
//Signature
void driveVector(const int distPower, const int anglePower)
```

Passthrough function to call `driveVector` on the internal `ChassisModel`.

### turnClockwise

```c++
//Signature
void turnClockwise(const int power)
```

Passthrough function to call `turnClockwise` on the internal `ChassisModel`.

### tank

```c++
//Signature
void tank(const int leftVal, const int rightVal, const int threshold = 0)
```

Passthrough function to call `tank` on the internal `ChassisModel`

### arcade

```c++
//Signature
void arcade(int verticalVal, int horizontalVal, const int threshold = 0)
```

Passthrough function to call `arcade` on the internal `ChassisModel`

### left

```c++
//Signature
void left(const int val);
```

Passthrough function to call `left` on the internal `ChassisModel`.

### right

```c++
//Signature
void right(const int val);
```

Passthrough function to call `right` on the internal `ChassisModel`.

### getEncoderVals

```c++
//Signature
std::valarray<int> getEncoderVals() const
```

Passthrough function to call `getEncoderVals` on the internal `ChassisModel`.
