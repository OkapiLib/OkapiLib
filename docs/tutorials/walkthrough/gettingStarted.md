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

## Troubleshooting

If OkapiLib is not getting downloaded from GitHub correctly duringinstallation,
you can
[download it manuallyhere](https://github.com/OkapiLib/OkapiLib/releases) and
then install it byrunning `prosv5 conduct fetch path/to/okapilib.zip`. Once
OkapiLib isinstalled, try creating a PROS project again.
