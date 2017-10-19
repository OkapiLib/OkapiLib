## OdomMath

The `OdomMath` class provides static implementations for common odometry operations.

### computeDistanceToPoint

```c++
//Signature
static float computeDistanceToPoint(const float ix, const float iy, const OdomState& istate)
```

Calculate the distance from the robot to a point, (`ix`, `iy`).

Parameter | Description
----------|------------
ix | X coordinate
iy | Y coordinate
istate | Odometry state

### computeAngleToPoint

```c++
//Signature
static float computeAngleToPoint(const float ix, const float iy, const OdomState& istate)
```

Calculate the angle from the robot to a point, (`ix`, `iy`).

Parameter | Description
----------|------------
ix | X coordinate
iy | Y coordinate
istate | Odometry state

### computeDistanceAndAngleToPoint

```c++
//Signature
static DistanceAndAngle computeDistanceAndAngleToPoint(const float ix, const float iy, const OdomState& istate)
```

Calculate the distance and the angle from the robot to a point, (`ix`, `iy`).

Parameter | Description
----------|------------
ix | X coordinate
iy | Y coordinate
istate | Odometry state
