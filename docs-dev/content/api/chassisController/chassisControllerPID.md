## ChassisControllerPid

The `ChassisControllerPid` class inherits from `ChassisController` and implements its interface using PID control.

### Constructor

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
