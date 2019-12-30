# Controller Inputs and Outputs

Fundamentally, a feedback control system needs both an input and an
output. In OkapiLib, Iterative Controllers do not require Controller
Inputs and Outputs to be passed into the constructor like Async
Controllers do, but they are still necessary for making use of the
controller. This tutorial will explain more about what Controller Inputs
and Outputs are, and what classes can be used for such purposes.

## General Usage

Using Controller Inputs and Outputs is quite simple, they each only have
one function. Controller Inputs will return their current state with the
[controllerGet](@ref okapi::ControllerInput::controllerGet) function, and
Controller Outputs can be given a desired output with
[controllerSet](@ref okapi::ControllerOutput::controllerSet). The exact 
implementation of these functions varies depending on what device you are using,
but all Controller Outputs will accept a range of `[-1, 1]` as the input to
[controllerSet](@ref okapi::ControllerOutput::controllerSet) for the sake of
standardization across multiple different devices and configurations (i.e.
motors with different gearings).

## What Classes are of Each Type?

**Controller Inputs:**

- [ADIButton](@ref okapi::ADIButton)
- [ADIEncoder](@ref okapi::ADIEncoder)
- [ADIGyro](@ref okapi::ADIGyro)
- [ADIUltrasonic](@ref okapi::ADIUltrasonic)
- [ControllerButton](@ref okapi::ControllerButton)
- [FilteredControllerInput](@ref okapi::FilteredControllerInput)
- [IntegratedEncoder](@ref okapi::IntegratedEncoder)
- [OffsetableControllerInput](@ref okapi::OffsetableControllerInput)
- [Potentiometer](@ref okapi::Potentiometer)

**Controller Outputs:**

- [ADIMotor](@ref okapi::ADIMotor)
- [Motor](@ref okapi::Motor)
- [MotorGroup](@ref okapi::MotorGroup)
