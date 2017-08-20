---
weight: 21
title: ChassisController
---

# ChassisController (abstract)

The `ChassisController` class an interface for controlling a robot's chassis: it provides methods that build upon the basic methods `ChassisModel` has for more accurate control.

## Constructor

```c++
//Signature
ChassisController(const ChassisModelParams& imodelParams)
ChassisController(std::shared_ptr<ChassisModel> imodel)
```

Parameter | Description
----------|------------
imodelParams | `ChassisModelParams` (used to make a new `ChassisModel`)
imodel | An existing `ChassisModel`

## driveStraight

```c++
//Signature
virtual void driveStraight(const int itarget) = 0
```

Drive the robot straight for a distance of `itarget` in the units of `itarget`.

Parameter | Description
----------|------------
itarget | Distance for the robot to travel

## pointTurn

```c++
//Signature
virtual void pointTurn(const float idegTarget) = 0
```

Turn the robot in place for an angle of `idegTarget`. The units of the angle travel is most often the difference in encoder ticks between the two sides of the chassis.
