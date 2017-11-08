#include "control/velMath.h"
#include <API.h>
#include "PAL/PAL.h"

namespace okapi {
  float VelMath::step(const float inewPos) {
    const long now = PAL::millis();

    vel = static_cast<float>((1000 / (now - lastTime))) * (inewPos - lastPos) * (60 / ticksPerRev);
    vel = filter.filter(vel);

    lastPos = inewPos;
    lastTime = now;

    return vel;
  }
}
