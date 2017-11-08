## Button

The `Button` class is a sample wrapper around a digital input port that represents a button.

### Constructor

```c++
//Signature
explicit constexpr Button()
explicit constexpr Button(const unsigned char iport, const bool iinverted = false)
explicit constexpr Button(const unsigned char ijoystick, const unsigned char ibuttonGroup, const unsigned char ibutton, const bool iinverted = false)
explicit constexpr Button(PROS_FILE* ilcdPort, const unsigned char ilcdButton, const bool iinverted = false)
```

Button or limit switch constructor:

Parameter | Description
----------|------------
iport | Digital port
iinverted | Whether the button is inverted (default open vs. default closed)

Joystick button constructor:

Parameter | Description
----------|------------
ijoystick | Joystick number (primary (1) or partner (2))
ibuttonGroup | Joystick button group
ibutton | Button in button group
iinverted | Whether the button is inverted (default open vs. default closed)

LCD button constructor:

Parameter | Description
----------|------------
ilcdPort | LCD port (uart1 or uart2)
ilcdbutton | LCD button mask
iinverted | Whether the button is inverted (default open vs. default closed)

This class also has literals available:

Literal | Button Value
--------|-------------
`n_b` | `Button(n, false)`
`n_ib` | `Button(n, true)`

### isPressed

```c++
//Signature
bool isPressed() const
```

Return whether this button is pressed or not. This takes into account whether the button is inverted or not.

### edge

```c++
//Signature
bool edge()
```

Return `true` if a rising or falling edge is detected. This takes into account whether the button is inverted or not.

### risingEdge

```c++
//Signature
bool risingEdge()
```

Return `true` if a rising edge is detected. This takes into account whether the button is inverted or not.

### fallingEdge

```c++
//Signature
bool fallingEdge()
```

Return `true` if a falling edge is detected. This takes into account whether the button is inverted or not.
