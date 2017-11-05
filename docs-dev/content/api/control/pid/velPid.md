## VelPid

The `VelPid` class implements the Pid algorithm for the velocity domain, with some quality-of-life changes to support online tuning. Inherits from `ControlObject`.

### Constructor

```c++
//Signature
VelPid(const float ikP, const float ikD)
VelPid(const VelPidParams& params)
```

Parameter | Description
----------|------------
ikP | Proportional gain
ikI | Integral gain
ikD | Derivative gain
ikBias | Controller bias (this value added to output, default 0)
params | `VelPidParams`

### stepVel

```c++
//Signature
virtual float stepVel(const float inewReading)
```

Do one iteration of velocity math to compute a new filtered velocity. This is only meant to be used separately from `step` if you only want to compute a new velocity. Don't call `stepVel` if you are already calling `step` because `step` will call `stepVel` on its own.

Parameter | Description
----------|------------
inewReading | New sensor reading

### step

```c++
//Signature
virtual float step(const float inewReading)
```

Do one iteration of Pid math to compute a new motor power. This needs to be called every so many milliseconds (15 ms works fine).

Parameter | Description
----------|------------
inewReading | New sensor reading

### setTarget

```c++
//Signature
void setTarget(const float itarget)
```

Set the target value.

Parameter | Description
----------|------------
itarget | New target value

### getOutput

```c++
//Signature
float getOutput() const
```

Return the most recent controller output.

### getError

```c++
//Signature
virtual float getError() const = 0
```

Return the most recent controller error.

### setSampleTime

```c++
//Signature
void setSampleTime(const int isampleTime)
```

Set the timestep (in ms) between calls to `step`.

Parameter | Description
----------|------------
isampleTime | Timestep between calls to `step` in ms

### setOutputLimits

```c++
//Signature
void setOutputLimits(float imax, float imin)
```

Set the max and min value for the controller output.

Parameter | Description
----------|------------
imax | Max output
imin | Min output

### reset

```c++
//Signature
void reset()
```

Reset the controller so it will start from zero again.

### flipDisable

```c++
//Signature
void flipDisable()
```

Change whether the controller is on or off. A controller which is off will output 0.

### setGains

```c++
//Signature
void setGains(const float ikP, const float ikD)
```

Set new controller gains and bias.

Parameter | Description
----------|------------
ikP | Proportional gain
ikD | Derivative gain

### setFilterGains

```c++
//Signature
void setFilterGains(const float alpha, const float beta)
```

Set new gains for the `DemaFilter`

Parameter | Description
----------|------------
alpha | Alpha gain
beta | Beta gain

### setTicksPerRev

```c++
//Signature
void setTicksPerRev(const float tpr)
```

Set the number of measurement units per revolution. Default is 360 (quadrature encoder).
