---
weight: 80
title: XDriveModelParams
---

# XDriveModelParams

The `XDriveModelParams` class inherits from `ChassisModelParams`. It encapsulates the parameters an `XDriveModel` takes.

## Constructor
 ```c++
 //Signature
 XDriveModelParams(const std::initializer_list<unsigned char>& imotorList, Encoder ileftEnc, Encoder irightEnc)
 ```

 Parameter | Description
 ----------|------------
 imotorList | The chassis motors for the drive in the clockwise format, `{top left motors, top right motors, bottom right motors, bottom left motors}`
 ileftEnc | The quadrature encoder for the left side
 irightEnc | The quadrature encoder for the right side
