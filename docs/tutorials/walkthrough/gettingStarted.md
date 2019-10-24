# Getting Started with OkapiLib

## Introduction

OkapiLib is installed in all new PROS projects by default. If you are unsure if
OkapiLib is installed, you can check the output of `prosv5 conduct info-project`.
Additionally, OkapiLib's header files reside in ` include/okapi`. Once you know
that OkapiLib is installed, you can start using it by uncommenting OkapiLib's
API header include statement in the file `include/main.h` :

``` cpp
#include "api.h"

/**
 * You should add more #includes here
 */
#include "okapi/api.hpp" // <-- UNCOMMENT THIS LINE
//#include "pros/api_legacy.h"
```

All okapi methods are located in the okapi namespace. 
To avoid typing `okapi::` in front of every command, uncomment the `using` statement in the file `include/main.h` :

``` cpp
/**
 * If you find doing pros::Motor() to be tedious and you'd prefer just to do
 * Motor, you can use the namespace with the following commented out line.
 *
 * IMPORTANT: Only the okapi or pros namespace may be used, not both
 * concurrently! The okapi namespace will export all symbols inside the pros
 * namespace.
 */
// using namespace pros;
// using namespace pros::literals;
using namespace okapi; // <-- UNCOMMENT THIS LINE
```

If you don't want to use the okapi namespace, but you want to use unit literals such as `1_in`, add the following line to the file `include/main.h` :

``` cpp
using namespace okapi::literals;
```

## Troubleshooting

If OkapiLib is not getting downloaded from GitHub correctly during
installation, you can
[download it manually here](https://github.com/OkapiLib/OkapiLib/releases) and
then install it by running `prosv5 conduct fetch path/to/okapilib.zip`. Once
OkapiLib is installed, try creating a PROS project again.
