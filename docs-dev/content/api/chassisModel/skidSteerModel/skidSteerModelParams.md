## SkidSteerModelParams

The `SkidSteerModelParams` class inherits from `ChassisModelParams`. It encapsulates the parameters a `SkidSteerModel` takes.

### Constructor

```c++
//Signature
SkidSteerModelParams(const std::initializer_list<unsigned char>& imotorList, Encoder ileftEnc, Encoder irightEnc)
```

Parameter | Description
----------|------------
imotorList | The left and right side motors for the drive in the format, `{left motors, right motors}`
ileftEnc | The quadrature encoder for the left side
irightEnc | The quadrature encoder for the right side
