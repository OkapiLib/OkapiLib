---
weight: 180
title: MPGeneratorParams
---

# MPGeneratorParams

The `MPGeneratorParams` class encapsulates the parameters an `MPGenerator` takes.

## Constructor

```c++
//Signature
MPGenParams(const float imaxAccel, const float iminAccel, const float imaxVel, const float itargetPos, const float istartVel, const float iendVel)
MPGenParams(const float imaxAccel, const float iminAccel, const float imaxVel, const float itargetPos)
MPGenParams(const float iaccel, const float imaxVel, const float itargetPos)
MPGenParams(const MPGenParams &other)
```

Parameter | Description
----------|------------
imaxAccel | Maximum acceleration
iminAccel | Minimum acceleration (i.e., maximum deceleration, make sure this value is negative)
imaxVel | Maximum velocity
itargetPos | Target position (how far should the robot travel)
istartVel | Starting velocity (normally 0)
iendVel | Ending velocity (normally 0)
