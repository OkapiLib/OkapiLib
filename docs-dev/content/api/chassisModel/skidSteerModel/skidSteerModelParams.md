## SkidSteerModelParams

The `SkidSteerModelParams` class inherits from `ChassisModelParams`. It encapsulates the parameters a `SkidSteerModel` takes.

### Constructor

```c++
//Signature
SkidSteerModelParams(const std::array<Motor, motorsPerSide * 2>& imotorList, const QuadEncoder& ileftEnc, const QuadEncoder& irightEnc):
SkidSteerModelParams(const std::array<Motor, motorsPerSide * 2>& imotorList, const IME& ileftIME, const IME& irightIME)
```

Parameter | Description
----------|------------
imotorList | The left and right side motors for the drive in the format, `{left motors, right motors}`
ileftEnc | The quadrature encoder for the left side
irightEnc | The quadrature encoder for the right side
ileftIME | The IME for the left side
irightIME | The IME for the right side
