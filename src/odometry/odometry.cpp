#include <cmath>
#include "odometry/odometry.h"
#include "util/mathUtil.h"
#include <API.h>

namespace okapi {
  void Odometry::guessScales(const float chassisDiam, const float wheelDiam, const float ticksPerRev) {
    scale = (wheelDiam * pi * inchToMM) / ticksPerRev;
    turnScale = 1.0 / (chassisDiam * inchToMM);
  }

  OdomState Odometry::loop() {
    unsigned long now = millis();
    std::valarray<int> newTicks{0, 0}, tickDiff{0, 0};

    while (true) {
      newTicks = model->getEncoderVals();
      tickDiff = newTicks - lastTicks;
      mm = ((tickDiff[0] + tickDiff[1]) / 2.0) * scale;
      lastTicks = newTicks;

      state.theta += (tickDiff[1] - tickDiff[0]) * turnScale;
      if (state.theta > 180)
        state.theta -= 360;
      else if (state.theta < -180)
        state.theta += 360;

      state.x += mm * std::cos(state.theta) * radianToDegree;
      state.y += mm * std::sin(state.theta) * radianToDegree;

      taskDelayUntil(&now, 15);
    }
  }
}
