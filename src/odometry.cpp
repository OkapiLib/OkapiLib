/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#include <cmath>
#include "okapi/odometry/odometry.hpp"
#include "okapi/util/mathUtil.hpp"

namespace okapi {
  Odometry::Odometry(const ChassisModelParams& imodelParams, const float iscale, const float iturnScale):
    model(imodelParams.make()),
    scale(iscale),
    turnScale(iturnScale),
    lastTicks{0, 0},
    mm(0) {}

  Odometry::Odometry(const OdometryParams& iparams):
    model(iparams.model),
    scale(iparams.scale),
    turnScale(iparams.turnScale),
    lastTicks{0, 0},
    mm(0) {}

  Odometry::~Odometry() = default;

  void Odometry::setScales(const float iscale, const float iturnScale) {
    scale = iscale;
    turnScale = iturnScale;
  }

  void Odometry::loop() {
    uint32_t now = pros::millis();
    std::valarray<int> newTicks{0, 0}, tickDiff{0, 0};

    while (true) {
      newTicks = model.getSensorVals();
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

  void Odometry::trampoline(void *context) {
    static_cast<Odometry*>(context)->loop();
  }

  OdomState Odometry::getState() {
    return state;
  }
}
