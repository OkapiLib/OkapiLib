## Odometry

The `Odometry` class tracks the robot as it moves, computing its position in the field frame. It is a singleton so its main method, `step`, can be called in a task.

### setParams

```c++
//Signature
static void setParams(OdomParams& iparams)
```

Set the model, scale, and turnScale parameters.

Parameter | Description
----------|------------
iparams | `OdomParams`

### setScales

```c++
//Signature
static void setScales(const float iscale, const float iturnScale)
```

Set the scale and turnScale parameters.

Parameter | Description
----------|------------
iscale | Driving scale (encoder ticks to mm)
iturnScale | Turning scale (encoder ticks to degrees)

### guessScales

```c++
//Signature
static void guessScales(const float chassisDiam, const float wheelDiam, const float ticksPerRev = 360.0)
```

Attempt to guess the two odometry scales from chassis and wheel diameter. This might get you close, but it serves only as a quick and temporary fix. The two odometry scales must be found experimentally for your specific robot.

Parameter | Description
----------|------------
chassisDiam | Center-to-center wheel base in inches
wheelDiam | Edge-to-edge wheel diameter in inches
ticksPerRev | Quadrate encoder ticks per one wheel revolution (default 360)

### step

```c++
//Signature
static OdomState step()
```

Do one iteration of odometry math to compute the new position of the robot. This needs to be called every so many milliseconds (15 ms seems to work fine).

### getState

```c++
//Signature
static OdomState getState()
```

Return the last calculated position of the robot.

## OdomParams

The `OdomParams` class encapsulates the parameters an `Odometry` takes.

Member | Description
-------|------------
model | `ChassisModel`
scale | Driving scale (encoder ticks to mm)
turnScale | Turning scale (encoder ticks to degrees)

### Constructor

```c++
//Signature
OdomParams(const ChassisModelParams& iparams, const float iscale, const float iturnScale)
```

Parameter | Description
----------|------------
iparams | `ChassisModelParams` (used to make a new `ChassisModel`)
iscale | Driving scale (encoder ticks to mm)
iturnScale | Turning scale (encoder ticks to degrees)

## OdomState

The `OdomState` class is a simple container for the position of the robot tracked by `Odometry`.

Member | Description
-------|------------
x | X coordinate of robot
y | Y coordinate of robot
theta | Theta of robot

### Constructor

```c++
//Signature
OdomState(const float ix, const float iy, const float itheta)
OdomState()
```

Parameter | Description
----------|------------
ix | X coordinate of robot
iy | Y coordinate of robot
itheta | Theta of robot
