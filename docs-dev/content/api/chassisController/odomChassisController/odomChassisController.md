## OdomChassisController (abstract)

The `OdomChassisController` class inherits from `ChassisController`. It extends the `ChassisController` interface to add odometry-based functionality.

### Constructor

```c++
//Signature
OdomChassisController(OdomParams iparams)
```

Parameter | Description
----------|------------
iparams | `OdomParams` (used to make a new `Odometry`)

<aside class="notice">
This class creates a new task inside its constructor and references a static class (Odometry). Be careful and remember to only make one.
</aside>

### driveToPoint

```c++
//Signature
virtual void driveToPoint(const float ix, const float iy, const bool ibackwards = false, const float ioffset = 0) = 0
```

Parameter | Description
----------|------------
ix | X coordinate of destination
iy | Y coordinate of destination
ibackwards | Whether to drive to the destination backwards (default false)
ioffset | How far back from the destination to stop (default 0)

Drive to the point (`ix`, `iy`) in the field frame. If required, the robot will first turn to face the destination point.

### turnToAngle

```c++
//Signature
virtual void turnToAngle(const float iangle) = 0
```

Parameter | Description
----------|------------
iangle | Angle to face

Turn to the angle `iangle` in the field frame.
