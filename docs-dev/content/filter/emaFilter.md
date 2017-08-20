---
weight: 110
title: EmaFilter
---

# EmaFilter

The `EmaFilter` class inherits from `Filter`. It is an infinite impulse response (IIR) filter that implements the exponential moving average algorithm (EMA).

## Constructor

```c++
//Signature
EmaFilter(const float ialpha, const float ibeta)
```

Parameter | Description
----------|------------
ialpha | Alpha gain
ibeta | Beta gain

## setGains

```c++
//Signature
void setGains(const float ialpha, const float ibeta)
```

Set new gains for this filter

Parameter | Description
----------|------------
ialpha | New alpha gain
ibeta | New beta gain
