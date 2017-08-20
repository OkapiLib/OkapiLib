---
weight: 170
title: MPGenerator
---

# MPGenerator

The `MPGenerator` class is a motion profile generator. It accepts position, velocity, and acceleration bounds and produces a motion profile within those bounds.

## Constructor

```c++
//Signature
MPGenerator(const float iaccel, const float imaxVel)
MPGenerator(const float iaccel, const float imaxVel, const float itargetPos)
MPGenerator(const float imaxAccel, const float iminAccel, const float imaxVel, const float itargetPos)
MPGenerator(const float imaxAccel, const float iminAccel, const float imaxVel, const float itargetPos, const float istartVel, const float iendVel)
MPGenerator(MPGenParams iparams)
```

Parameter | Description
----------|------------
imaxAccel | Maximum acceleration
iminAccel | Minimum acceleration (i.e., maximum deceleration, make sure this value is negative)
imaxVel | Maximum velocity
itargetPos | Target position (how far should the robot travel)
istartVel | Starting velocity (normally 0)
iendVel | Ending velocity (normally 0)

## generateProfile

```c++
//Signature
MotionProfile generateProfile(const float idt)
```

Generate and return a complete motion profile.

Parameter | Description
----------|------------
idt | Timestep between targets

## getNextVelTarget

```c++
//Signature
MPTarget getNextVelTarget(const float itime)
```

Calculate and return the next velocity target in the motion profile.

## isComplete

```c++
//Signature
bool isComplete() const
```

Return whether the motion profile has been followed start to finish and is done.
