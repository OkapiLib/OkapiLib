---
title: Getting started
type: index
---

## Programming the Clawbot

This tutorial is a step-by-step guide to programming this simple skills robot:

{{< figure src="/images/skillsRobotPicture.png#center" >}}

This robot has motors and sensors plugged into the following ports:

Motor Port | Description | Motor Port | Description
-----|-------------|------|------------
1 |                   | 6  | Right middle motor
2 | Left front motor  | 7  | Right bottom motor
3 | Left middle motor | 8  | 
4 | Left bottom motor | 9  | 
5 | Right top motor   | 10 | 

Digital Port | Description | Digital Port | Description
------------|-------------|-------------|------------
1 | Left encoder top wire     | 7  | 
2 | Left encoder bottom wire  | 8  | 
3 | Right encoder top wire    | 9  | 
4 | Right encoder bottom wire | 10 | 
5 |                           | 11 | 
6 |                           | 12 | 

Analog Port | Description | Analog Port | Description
------------|-------------|-------------|------------
1 | Lift potentiometer | 5 | 
2 |                    | 6 | 
3 |                    | 7 | 
4 |                    | 8 | 

## Project Configuration

After installing OkapiLib through the PROS Conductor tool, `okapilib.a` will be copied into `firmware/` and OkapiLib's header files will be copied into `include/`. In order to properly compile your project, you need to modify a few files:

1. The `src/` folder contains three files, `auto.c`, `init.c`, and `opcontrol.c`. The `.c` extension of these files needs to be changed to `.cpp` so the compiler knows they contain C++ code.

2. The `src/init.cpp` file needs to be changed so it instead contains the following code which calls a special internal function `__libc_init_array()`:

```c++
#include "main.h"

extern "C" {
  void __libc_init_array();
}

void initializeIO() {
  __libc_init_array();
}

void initialize() {
}
```

3. Finally, the `common.mk` file needs to have the `CPPFLAGS` variable modified to contain the flag `-std=c++14` so the compiler knows the correct C++ standard to use:

This line: `CPPFLAGS:=$(CCFLAGS)-fno-exceptions -fno-rtti -felide-constructors`

Should become this: `CPPFLAGS:=$(CCFLAGS) -std=c++14 -fno-exceptions -fno-rtti -felide-constructors`
