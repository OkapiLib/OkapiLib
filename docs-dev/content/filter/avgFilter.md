---
weight: 100
title: AvgFilter
---

# AvgFilter

The `AvgFilter` class inherits from `Filter` and takes a template parameter `std::size_t n>` (the number of inputs to average). It is a finite impulse response (FIR) filter that averages the last `n` inputs.

## Constructor

```c++
//Signature
AvgFilter()
```

The constructor does not take any parameters.
