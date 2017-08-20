---
weight: 50
title: SkidSteerModel
---

# SkidSteerModel

The `SkidSteerModel` class inherits from `ChassisModel` and takes a template parameter `size_t motorsPerSide` (the number of motors per each of the two sides of the chassis). It is a model for a skid steer drive (also called a tank drive).

## Constructor

```c++
//Signature
SkidSteerModel(const std::initializer_list<unsigned char>& imotorList, Encoder ileftEnc, Encoder irightEnc)
SkidSteerModel(const SkidSteerModelParams<motorsPerSide>& iparams)
SkidSteerModel(const SkidSteerModel<motorsPerSide>& other)

//Construct a SkidSteerModel with four motors (two per side) and two encoders
//Left side motors are ports 1 and 3
//Right side motors are ports 2 and 4
//Right side encoder is reversed because it is a mirror of the left side
SkidSteerModel<2> foo({1, 3, 2, 4}, encoderInit(1, 2, false), encoderInit(3, 4, true))
```

Parameter | Description
----------|------------
imotorList | The left and right side motors for the drive in the format, `{left motors, right motors}`
ileftEnc | The quadrature encoder for the left side
irightEnc | The quadrature encoder for the right side
