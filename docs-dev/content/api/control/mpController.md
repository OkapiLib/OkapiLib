## MPController

The `MPController` class packages together an `MPConsumer` and an `MPGenerator` into one class which both generates and follows a motion profile.

### Constructor

```c++
//Signature
MPController(const MPGenParams& igenParams, const MPConsumerParams& iconParams)
MPController(const MPControllerParams& iparams)

//Basic MPController construction
MPController foo(MPGenParams(1, 15, 1000), MPConsumerParams(6, 1.2, 0.5));
```

Parameter | Description
----------|------------
igenParams | `MPGenParams`
iconParams | `MPConsumerParams`
iparams | `MPControllerParams`

### step

```c++
//Signature
virtual float step(const float inewReading) override
```

Step the controller once over a new measurement and return the new response power.

### setTarget

```c++
//Signature
void setTarget(const int pos) override
```

Set a new target for the controller, causing the internal motion profile to be regenerated.

### getOutput

```c++
//Signature
float getOutput() const override
```

Return the most recent controller output.

### reset

```c++
//Signature
void reset() override
```

Reset the controller so it can follow another profile.

### isComplete

```c++
//Signature
bool isComplete() const
```

Return whether the motion profile has been followed start to finish and is done.

## MPControllerParams

The `MPControllerParams` class encapsulates the parameters an `MPController` takes.

### Constructor

```c++
//Signature
MPControllerParams(const MPGenParams& igenParams, const MPConsumerParams& iconParams)
```

Parameter | Description
----------|------------
igenParams | `MPGenParams`
iconParams | `MPConsumerParams`
