## SlewMotor

The `SlewMotor` class adds a slew rate to the `Motor` class.

### Constructor

```c++
//Signature
SlewMotor(const Motor& imotor, const float islewRate)
```

Parameter | Description
----------|------------
imotor | Motor to slew
islewRate | Slew rate

### set

```c++
//Signature
virtual void set(const int val) const
```

Slew the power of this motor up to `val`.

Parameter | Description
----------|------------
val | Target motor power

### setTS

```c++
//Signature
virtual void set(const int val) const
```

Slew the power of this motor up to `val` using trueSpeed.

Parameter | Description
----------|------------
val | Target motor power
