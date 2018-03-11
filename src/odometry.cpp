#include <cmath>
#include "okapi/odometry/odometry.h"
#include "okapi/util/mathUtil.h"

namespace okapi {
  void Odometry::loop() {
    uint32_t now = pros::millis();
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

      task_delay_until(&now, 15);
    }
  }
}
