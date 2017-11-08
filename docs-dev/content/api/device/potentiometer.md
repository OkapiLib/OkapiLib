## Potentiometer

### Constructor

```c++
//Signature
explicit constexpr Potentiometer()
explicit constexpr Potentiometer(const unsigned char iport)
explicit constexpr Potentiometer(const unsigned char iport, const bool iinverted)
```

Parameter | Description
----------|------------
iport | Analog input port
iinverted | Whether the potentiometer is inverted (range is 0->4095 or 4095->0)

This class also has literals available:

Literal | Potentiometer Value
--------|--------------------
`n_p` | `Potentiometer(n, false)`
`n_ip` | `Potentiometer(n, true)`

### get

```c++
//Signature
int get() const
```

Return the value of this potentiometer.
