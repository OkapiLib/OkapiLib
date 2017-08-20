---
weight: 140
title: MotionProfile
---

# MotionProfile

The `MotionProfile` class encapsulates the series of velocity and acceleration targets that make up a motion profile.

Member | Description
-------|------------
data | A vector of velocity and acceleration targets
distance | The distance the motion profile travels
dt | The timestep between targets

## Constructor

```c++
//Signature
MotionProfile(const float idistance, const float idt)
```

Parameter | Description
----------|------------
idistance | The distance the motion profile travels
idt | The timestep between targets

# MPTarget

The `MPTarget` class is designed to encapsulate a pair of velocity and acceleration targets for a motion profile.

Member | Description
-------|------------
vel | A velocity target
accel | An acceleration target

## Constructor

```c++
//Signature
MPTarget(const float velocity, const float acceleration)
MPTarget()
```

Parameter | Description
----------|------------
vel | A velocity target
accel | An acceleration target
