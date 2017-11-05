## XDriveModelParams

The `XDriveModelParams` class inherits from `ChassisModelParams`. It encapsulates the parameters an `XDriveModel` takes.

### Constructor
```c++
//Signature
XDriveModelParams(const std::array<unsigned char, motorsPerCorner * 4>& imotorList, const QuadEncoder& ileftEnc, const QuadEncoder& irightEnc):
XDriveModelParams(const std::array<unsigned char, motorsPerCorner * 4>& imotorList, const IME& ileftIME, const IME& irightIME)
```

Parameter | Description
----------|------------
imotorList | The chassis motors for the drive in the clockwise format, `{top left motors, top right motors, bottom right motors, bottom left motors}`
ileftEnc | The quadrature encoder for the left side
irightEnc | The quadrature encoder for the right side
ileftIME | The IME for the left side
irightIME | The IME for the right side
