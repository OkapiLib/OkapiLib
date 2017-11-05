## IME

The `IME` class is a simple container for an IME. Inherits from `RotarySensor`.

### Constructor

```c++
//Signature
IME(const unsigned int iindex)
IME(const unsigned int iindex, const bool ireversed)
```

Parameter | Description
----------|------------
iindex | IME index in the chain
ireversed | Whether the IME is reversed or not (clockwise turn increases vs. decreases ticks)

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
