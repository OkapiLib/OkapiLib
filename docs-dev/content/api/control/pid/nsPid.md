## NsPid

### Constructor

```c++
//Signature
NsPid(const PidParams& iparams, const VelMathParams& ivelParams, const float iminVel, const float iscale = 0.1)
```

Parameter | Description
----------|------------
iparams | `PidParams` to make the internal PID controller
ivelParams | `VelMathParams` for the velocity calculations
iminVel | Minimum velocity at which the controller will start reducing the output power
iscale | Scale to reduce the output power by

### step

```c++
//Signature
virtual float step(const float inewReading) override
```

Do one iteration of Pid math to compute a new motor power. This needs to be called every so many milliseconds (15 ms works fine).

Calls `step` from class `Pid`, and may return a reduced power if the velocity of the process is sufficiently low. The purpose of a low power mode is to prevent motors from stalling once they have reached their target (or if they can't quite reach their target).

Parameter | Description
----------|------------
inewReading | New sensor reading
