## CubicMotor

The `CubicMotor` class provides a cubic-control based implementation of the `set` function `Motor` defines. This means that the motor power from `set` follows a cubic curve instead of the default identity function. The `setTS` function is not overridden and so provides the same behavior as in `Motor`.

### Constructor

```c++
//Signature
explicit constexpr CubicMotor()
explicit constexpr CubicMotor(const unsigned char iport, const int isign)
```

Parameter | Description
----------|------------
iport | Motor port
isign | `1` for forward, `-1` for reversed

This class also has literals available:

Literal | Motor Value
--------|------------
`n_m3` | `CubicMotor(m, 1)`
`n_rm3` | `CubicMotor(m, -1)`

### set

```c++
//Signature
virtual void set(const int val) const
```

Set the power of this motor using a cubic function. The exact math for the output of this function is `(val * val * val) / (127 * 127)`.

Parameter | Description
----------|------------
val | Motor power
