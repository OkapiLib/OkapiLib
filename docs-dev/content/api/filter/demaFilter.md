## DemaFilter

The `DemaFilter` class inherits from `Filter`. It is an infinite impulse response (IIR) filter that implements the double exponential moving average algorithm (EMA). This is very similar to the normal EMA, except it can pick up on trends in data and follow the current trend until a new trend starts. This filter performs better than a single EMA when data frequently follows a trend.

### Constructor

```c++
//Signature
DemaFilter(const float ialpha, const float ibeta)
```

Parameter | Description
----------|------------
ialpha | Alpha gain
ibeta | Beta gain

### setGains

```c++
//Signature
void setGains(const float ialpha, const float ibeta)
```

Set new gains for this filter

Parameter | Description
----------|------------
ialpha | New alpha gain
ibeta | New beta gain
