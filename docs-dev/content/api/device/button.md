## Button

The `Button` class is a sample wrapper around a digital input port that represents a button.

### Constructor

```c++
//Signature
explicit constexpr Button()
explicit constexpr Button(const unsigned long long int iport)
explicit constexpr Button(const unsigned long long int iport, const bool iinverted)
```

Parameter | Description
----------|------------
iport | Digital port
iinverted | Whether the button is inverted (default open vs. default closed)


This class also has literals available:

Literal | Button Value
--------|-------------
`n_b` | `Button(n, false)`
`n_ib` | `Button(n, true)`
