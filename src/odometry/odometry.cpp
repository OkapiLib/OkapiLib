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

    while (true) {
      const auto newTicks = model->getEncoderVals();
      currentTicks = newTicks - lastTicks;
      mm = (((float)currentTicks[0] * scale) + ((float)currentTicks[1] * scale)) / 2.0;
      lastTicks = newTicks;

      state.theta += (currentTicks[1] - currentTicks[0]) * turnScale;
      if (state.theta > 180)
        state.theta -= 360;
      else if (state.theta <= -180)
        state.theta += 360;

      state.x += mm * std::cos(state.theta) * radianToDegree;
      state.y += mm * std::sin(state.theta) * radianToDegree;

      taskDelayUntil(&now, 15);
    }
  }
}
