## ChassisModelParams (Abstract)

The `ChassisModelParams` class encapsulates the parameters a `ChassisModel` takes.

### Constructor

```c++
//Signature
ChassisModelParams()
```

The constructor does not take any parameters.

### make

Allocate a new `ChassisModel` and return a `std::shared_ptr` to it. Most users will not have to call this, classes that take a `ChassisModel` will figure out the memory model themselves.

```c++
//Signature
virtual std::shared_ptr<ChassisModel> make() const = 0
```
