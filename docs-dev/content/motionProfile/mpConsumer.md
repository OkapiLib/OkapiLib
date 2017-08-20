---
weight: 150
title: MPConsumer
---

# MPConsumer

The `MPConsumer` class is a feed-forward closed-loop controller that follows a pre-generated motion profile.

## Constructor

```c++
//Signature
MPConsumer(MotionProfile &iprofile, const float ikV, const float ikA, const float ikP = 0)
MPConsumer(MotionProfile &iprofile, const MPConsumerParams& iparams)
```

Parameter | Description
----------|------------
iprofile | A motion profile generated using `MPGenerator`
ikV | Feed-forward velocity gain
ikA | Feed-forward acceleration gain
ikP | Feedback proportional gain (default = 0)

## loop

```c++
//Signature
virtual float loop(const float newReading)
```
Loop the controller once over a new measurement and return the new response power.

## isComplete

```c++
//Signature
bool isComplete() const
```

Return whether the motion profile has been followed start to finish and is done.

## newProfile

```c++
//Signature
void newProfile(MotionProfile& iprofile)
```

Reinitialize this instance with a new motion profile. Also resets the internal `VelPid` controller.
