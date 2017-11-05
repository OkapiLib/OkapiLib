## QuadEncoder

The `QuadEncoder` class is a simple container for an encoder. Inherits from `RotarySensor`.

### Constructor

```c++
//Signature
QuadEncoder(const unsigned char iportTop, const unsigned char iportBottom)
QuadEncoder(const unsigned char iportTop, const unsigned char iportBottom, const bool ireversed)
```

Parameter | Description
----------|------------
iportTop | Top digital in port
iportBottom | Bottom digital in port
ireversed | Whether the encoder is reversed or not (clockwise turn increases vs. decreases ticks)

### get

```c++
//Signature
int get() override
```

Return the current tick count.

### reset

```c++
//Signature
void reset() override
```

Reset the tick count to zero.
