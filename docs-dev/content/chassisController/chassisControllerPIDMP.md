---
weight: 22
title: ChassisControllerPIDMP
---

# ChassisControllerPid

The `ChassisControllerPid` class inherits from `ChassisController` and implements its interface using PID control.

## Constructor

```c++
//Signature
ChassisControllerPid(const ChassisModelParams& imodelParams, const PidParams& idistanceParams, const PidParams& iangleParams)
ChassisControllerPid(const std::shared_ptr<ChassisModel>& imodel, const PidParams& idistanceParams, const PidParams& iangleParams)

//Make a new ChassisControllerPid using a skid steer model with two motors per side
ChassisControllerPid foo(
  SkidSteerModelParams<2>({1, 3, 2, 4}, //The four motor ports
                          encoderInit(1, 2, false), //Left encoder
                          encoderInit(3, 4, true)), //Right encoder
  PidParams(2, 0.1, 0.4),    //Distance PID controller
  PidParams(0.3, 1.2, 0.1)); //Angle PID controller
```

Parameter | Description
----------|------------
imodelParams | `ChassisModelParams` (used to make a new `ChassisModel`)
imodel | An existing `ChassisModel`
idistanceParams | `PidParams` for the distance PID controller
iangleParams | `PidParams` for the angle PID controller

<aside class="notice">
Most users should not call this constructor with a std::shared_ptr&lt;ChassisModel&gt;. Instead, pass a ChassisModelParams and Okapi will figure out what to do.
</aside>

# ChassisControllerMP

The `ChassisControllerMP` class inherits from `ChassisController` and implements its interface using motion profiling.

## Constructor

```c++
//Signature
ChassisControllerMP(const ChassisModelParams& imodelParams, const MPControllerParams& iparams)
ChassisControllerMP(const std::shared_ptr<ChassisModel>& imodel, const MPControllerParams& iparams)

//Make a new ChassisControllerMP using a skid steer model with two motors per side
ChassisControllerMP foo(
  SkidSteerModelParams<2>({1, 3, 2, 4}, //The four motor ports
                          encoderInit(1, 2, false), //Left encoder
                          encoderInit(3, 4, true)), //Right encoder
  MPControllerParams(
    MPGenParams(1, 15, 1000),        //MPGenerator params; max & min accel: 1, max vel: 15, target pos: 1000
    MPConsumerParams(6, 1.2, 0.5))); //MPConsumer params; kV: 6, kA: 1.2, kP: 0.5
```

Parameter | Description
----------|------------
imodelParams | `ChassisModelParams` (used to make a new `ChassisModel`)
imodel | An existing `ChassisModel`
igenParams | `MPGenParams` (used to make a new `MPGenerator`)
icParams | `MPConsumerParams` (used to make a new `MPConsumer`)

<aside class="notice">
Most users should not call this constructor with a std::shared_ptr&lt;ChassisModel&gt;. Instead, pass a ChassisModelParams and Okapi will figure out what to do.
</aside>
