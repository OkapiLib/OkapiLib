#ifndef PATHFINDER_MOD_TANK_H_DEF
#define PATHFINDER_MOD_TANK_H_DEF

#include "pathfinder/lib.h"

CAPI void pathfinder_modify_tank(Segment *original, int length, Segment *left, Segment *right, double wheelbase_width);

#endif