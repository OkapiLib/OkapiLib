## RotarySensor (abstract)

The `RotarySensor` class is a simple container for a sensor which spins indefinitely to measure rotation, like a `QuadEncoder` or an `IME`.

### get

```c++
//Signature
virtual int get() = 0
```

Return the current tick count.

### reset

```c++
//Signature
virtual void reset() = 0
```

Reset the tick count to zero.
