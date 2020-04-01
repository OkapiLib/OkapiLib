#ifndef PATHFINDER_FIT_H_DEF
#define PATHFINDER_FIT_H_DEF

#include "okapi/pathfinder/include/pathfinder/lib.h"
#include "okapi/pathfinder/include/pathfinder/structs.h"

CAPI void pf_fit_hermite_pre(Waypoint a, Waypoint b, Spline *s);
CAPI void pf_fit_hermite_cubic(Waypoint a, Waypoint b, Spline *s);
CAPI void pf_fit_hermite_quintic(Waypoint a, Waypoint b, Spline *s);

#define FIT_HERMITE_CUBIC   &pf_fit_hermite_cubic
#define FIT_HERMITE_QUINTIC &pf_fit_hermite_quintic

#endif