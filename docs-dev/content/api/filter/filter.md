## Filter (abstract)

The `Filter` class is an interface for data filtering.

### Constructor

```c++
//Signature
Filter()
```

The constructor does not take any parameters.

### filter

```c++
//Signature
virtual float filter(const float ireading) = 0;
```

Filter an input and return the filtered output.


### getOutput

```c++
//Signature
virtual float getOutput() const = 0;
```

Return the previous output.
