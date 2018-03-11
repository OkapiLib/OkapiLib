#include "okapi/control/velMath.h"
#include "api.h"

namespace okapi {
  float VelMath::step(const float inewPos) {
    const long now = pros::millis();

    vel = static_cast<float>((1000 / (now - lastTime))) * (inewPos - lastPos) * (60 / ticksPerRev);
    vel = filter.filter(vel);

    lastPos = inewPos;
    lastTime = now;

    return vel;
  }
}
