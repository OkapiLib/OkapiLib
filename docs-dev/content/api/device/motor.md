## Motor

The `Motor` class provides a wrapper around the default motor utilities.

### Constructor

```c++
//Signature
explicit constexpr Motor()
explicit constexpr Motor(const unsigned char iport, const int isign)
```

Parameter | Description
----------|------------
iport | Motor port
isign | `1` for forward, `-1` for reversed

This class also has literals available:

Literal | Motor Value
--------|------------
`n_m` | `Motor(m, 1)`
`n_rm` | `Motor(m, -1)`

### set

```c++
//Signature
virtual void set(const int val) const
```

Set the power of this motor.

Parameter | Description
----------|------------
val | Motor power

### setTS

```c++
//Signature
virtual void set(const int val) const
```

Set the power of this motor using trueSpeed.

Parameter | Description
----------|------------
val | Motor power
