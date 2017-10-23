## VelMath

### Constructor

```c++
//Signature
VelMath(const float iticksPerRev, const float ialpha = 0.19, const float ibeta = 0.041)
VelMath(const VelMathParams& iparams)
```

Parameter | Description
----------|------------
iticksPerRev | Encoder ticks per one revolution
ialpha | `DemaFilter` alpha gain
ibeta | `DemaFilter` beta gain

### step

```c++
//Signature
float step(const float inewPos)
```

Calculate, filter, and return a new velocity. This need to be called every so many milliseconds (not any faster than 15 ms).

### setGains

```c++
//Signature
void setGains(const float ialpha, const float ibeta)
```

Set new filter gains.

Parameter | Description
----------|------------
ialpha | Alpha gain
ibeta | Beta gain

### setTicksPerRev


```c++
//Signature
void setTicksPerRev(const float iTPR)
```

Set a new ticks per rev value.

Parameter | Description
----------|------------
iTPR | Encoder ticks per one revolution

### getOutput

```c++
//Signature
float getOutput() const
```

Return the most recent velocity.

### getDiff

```c++
//Signature
float getDiff() const
```

Return the difference between the last and second to last velocity. Dividing this value by the sample time would give an acceleration.
