---
weight: 24
title: OdomChassisControllerPIDMP
---

# OdomChassisControllerPid

The `OdomChassisControllerPid` class inherits from `OdomChassisController` and from `ChassisControllerPid`. It implements the `OdomChassisController` interface using PID control.

## Constructor

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

# OdomChassisControllerMP

The `OdomChassisControllerMP` class inherits from `OdomChassisController` and from `ChassisControllerMP`. It implements the `OdomChassisController` interface using motion profiling.

## Constructor

```c++
//Signature
OdomChassisControllerMP(const OdomParams& params, const MPControllerParams& iconparams)

//Make a new ChassisControllerMP using a skid steer model with two motors per side
OdomChassisControllerMP foo(
  OdomParams(
    SkidSteerModelParams<2>({1,3,2,4}, //The four motor ports
                            encoderInit(1,2,false), //Left encoder
                            encoderInit(3,4,true)), //Right encoder
    1.345,     //Distance scale (encoder ticks to mm)
    12.88361), //Turn scale (encoder ticks to deg)
  MPControllerParams(
    MPGenParams(1, 15, 1000),        //MPGenerator params; max & min accel: 1, max vel: 15, target pos: 1000
    MPConsumerParams(6, 1.2, 0.5))); //MPConsumer params; kV: 6, kA: 1.2, kP: 0.5
```

Parameter | Description
----------|------------
params | `OdomParams` (used to make a new `Odometry`)
igenParams | `MPGenParams` (used to make a new `MPGenerator`)
icParams | `MPConsumerParams` (used to make a new `MPConsumer`)
