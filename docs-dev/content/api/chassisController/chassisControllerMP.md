## ChassisControllerMP

The `ChassisControllerMP` class inherits from `ChassisController` and implements its interface using motion profiling.

### Constructor

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
