#include <cmath>
#include "odometry/odometry.h"
#include "util/mathUtil.h"

namespace okapi {
  std::shared_ptr<ChassisModel> Odometry::model;
  OdomState Odometry::state;
  float Odometry::scale = 0, Odometry::turnScale = 0;
  std::valarray<int> Odometry::lastTicks(0, 0), Odometry::currentTicks(0, 0);
  float Odometry::mm = 0;

  void Odometry::guessScales(const float chassisDiam, const float wheelDiam, const float ticksPerRev) {
    scale = (wheelDiam * pi * inchToMM) / ticksPerRev;
    turnScale = 1.0 / (chassisDiam * inchToMM);
  }

  OdomState Odometry::loop() {
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

    return state;
  }
}
