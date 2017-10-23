## MPConsumer

The `MPConsumer` class is a feed-forward closed-loop controller that follows a pre-generated motion profile.

### Constructor

```c++
//Signature
MPConsumer(const float ikV, const float ikA, const float ikP = 0)
MPConsumer(const MPConsumerParams& iparams)
```

Parameter | Description
----------|------------
iprofile | A motion profile generated using `MPGenerator`
ikV | Feed-forward velocity gain
ikA | Feed-forward acceleration gain
ikP | Feedback proportional gain (default = 0)

### step

```c++
//Signature
virtual float step(const MotionProfile& profile, const float newReading)
```

Step the controller once over a new measurement and return the new response power.

### isComplete

```c++
//Signature
bool isComplete() const
```

Return whether the motion profile has been followed start to finish and is done.

### reset

```c++
//Signature
void reset()
```

Reset the controller so it can follow another profile.
