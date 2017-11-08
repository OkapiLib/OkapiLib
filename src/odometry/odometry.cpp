#include <cmath>
#include "odometry/odometry.h"
#include "util/mathUtil.h"
#include "PAL/PAL.h"

namespace okapi {
  void Odometry::guessScales(const float chassisDiam, const float wheelDiam, const float ticksPerRev) {
    scale = ((wheelDiam * pi * inchToMM) / ticksPerRev) * 0.9945483364; //This scale is usually off by this amount
    turnScale = (1.0 / (chassisDiam * inchToMM)) * degreeToRadian;
  }

  OdomState Odometry::loop() {
    unsigned long now = PAL::millis();
    std::valarray<int> newTicks{0, 0}, tickDiff{0, 0};

    while (true) {
      newTicks = model->getSensorVals();
      tickDiff = newTicks - lastTicks;
      mm = (static_cast<float>(tickDiff[1] + tickDiff[0]) / 2.0) * scale;
      lastTicks = newTicks;

      state.theta += (static_cast<float>(tickDiff[1] - tickDiff[0]) / 2.0) * turnScale;
      if (state.theta > 180)
        state.theta -= 360;
      else if (state.theta < -180)
        state.theta += 360;

      state.x += mm * std::cos(state.theta);
      state.y += mm * std::sin(state.theta);

      PAL::taskDelayUntil(&now, 15);
    }
  }
}
