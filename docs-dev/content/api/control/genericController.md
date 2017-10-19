## GenericController

The `GenericController` class combines motors and a `ControlObject` into one package that uses the controller to control the motors as a group (all motors get the same controller output).

### Constructor

```c++
//Signature
GenericController(const std::array<Motor, motorNum> &imotorList, const std::shared_ptr<ControlObject> &iptr)
```

Parameter | Description
----------|------------
imotorList | `std::array` of `Motor` that will be controlled
iptr | `ControlObject` used to control the motors

### loop

```c++
//Signature
void loop(const float ireading)
```

Have the `ControlObject` do one iteration and then power the motors with the output. This needs to be called every so many milliseconds (15 ms works fine).

Parameter | Description
----------|------------
ireading | New sensor reading

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
void getOutput() const
```

Return the most recent controller output.

### setSampleTime

```c++
//Signature
void setSampleTime(const int isampleTime)
```

Set the timestep (in ms) between calls to `loop`.

Parameter | Description
----------|------------
isampleTime | Timestep between calls to `loop` in ms

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
