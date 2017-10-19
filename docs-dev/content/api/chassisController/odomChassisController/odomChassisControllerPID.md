## OdomChassisControllerPid

The `OdomChassisControllerPid` class inherits from `OdomChassisController` and from `ChassisControllerPid`. It implements the `OdomChassisController` interface using PID control.

### Constructor

```c++
//Signature
OdomChassisControllerPid(const OdomParams& params, const PidParams& idistanceParams, const PidParams& iangleParams)

//Make a new OdomChassisControllerPid using a skid steer model with two motors per side
OdomChassisControllerPid foo(
  OdomParams(
    SkidSteerModelParams<2>({1,3,2,4}, //The four motor ports
                            encoderInit(1,2,false), //Left encoder
                            encoderInit(3,4,true)), //Right encoder
    1.345,     //Distance scale (encoder ticks to mm)
    12.88361), //Turn scale (encoder ticks to deg)
  PidParams(0,0,0),  //Distance PID controller
  PidParams(0,0,0)); //Angle PID controller
```

Parameter | Description
----------|------------
params | `OdomParams` (used to make a new `Odometry`)
idistanceParams | `PidParams` for the distance PID controller
iangleParams | `PidParams` for the angle PID controller
