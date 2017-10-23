## ControlObject (abstract)

The `ControlObject` class is an interface to closed-loop controllers. It requires implementation of step, target, and output functions.

### step

```c++
//Signature
virtual float step(const float ireading) = 0
```

Do one iteration of the control math to compute a new motor power. Normally called in a loop.

Parameter | Description
----------|------------
ireading | New sensor reading

### setTarget

```c++
//Signature
virtual void setTarget(const float itarget) = 0
```

Set the target value.

Parameter | Description
----------|------------
itarget | New target value

### getOutput

```c++
//Signature
virtual float getOutput() const = 0
```

Return the most recent controller output.

### setSampleTime

```c++
//Signature
virtual void setSampleTime(const int isampleTime)
```

Set the timestep (in ms) between calls to `step`.

Parameter | Description
----------|------------
isampleTime | Timestep between calls to `step` in ms

### setOutputLimits

```c++
//Signature
virtual void setOutputLimits(float imax, float imin)
```

Set the max and min value for the controller output.

Parameter | Description
----------|------------
imax | Max output
imin | Min output

### reset

```c++
//Signature
virtual void reset()
```

Reset the controller so it will start from zero again.

### flipDisable

```c++
//Signature
virtual void flipDisable()
```

Change whether the controller is on or off. A controller which is off will output 0.
