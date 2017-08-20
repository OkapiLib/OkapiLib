---
weight: 190
title: OdomMath
---

# OdomMath

The `OdomMath` class provides static implementations for common odometry operations.

## computeDistanceToPoint

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

## computeAngleToPoint

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

## computeDistanceAndAngleToPoint

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

# DistanceAndAngle

The `DistanceAndAngle` class is a simple container for the two parameters returned by `OdomMath::computeDistanceAndAngleToPoint`.

Member | Description
-------|------------
length | Distance to point
theta | Angle to point

## Constructor

```c++
//Signature
DistanceAndAngle(const float ilength, const float itheta)
DistanceAndAngle()
```

Parameter | Description
----------|------------
ilength | Distance to point
itheta | Angle to point
