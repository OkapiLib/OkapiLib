## PidParams

The `PidParams` class encapsulates the parameters a `Pid` takes.

Parameter | Description
----------|------------
kP | Proportional gain
kI | Integral gain
kD | Derivative gain
kBias | Controller bias (this value added to output)

### Constructor

```c++
//Signature
PidParams(const float ikP, const float ikI, const float ikD, const float ikBias = 0)
```

Parameter | Description
----------|------------
ikP | Proportional gain
ikI | Integral gain
ikD | Derivative gain
ikBias | Controller bias (this value added to output, default 0)
