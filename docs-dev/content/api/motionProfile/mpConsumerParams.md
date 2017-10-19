## MPConsumerParams

The `MPConsumerParams` class encapsulates the parameters an `MPConsumer` takes.

### Constructor

```c++
//Signature
MPConsumerParams(const float ikV, const float ikA, const float ikP = 0)
```

Parameter | Description
----------|------------
ikV | Feed-forward velocity gain
ikA | Feed-forward acceleration gain
ikP | Feedback proportional gain (default = 0)
