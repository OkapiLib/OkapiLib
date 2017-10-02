#include "control/velMath.h"
#include <API.h>

namespace okapi {
  float VelMath::loop(const float inewPos) {
    const long now = millis();

    vel = (1000 / (now - lastTime)) * (inewPos - lastPos) * (60 / ticksPerRev);
    vel = filter.filter(vel);

    lastPos = inewPos;
    lastTime = now;

    return vel;
  }
}
