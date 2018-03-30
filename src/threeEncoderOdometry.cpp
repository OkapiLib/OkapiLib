/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/odometry/threeEncoderOdometry.hpp"

namespace okapi {
ThreeEncoderOdometryArgs::ThreeEncoderOdometryArgs(const SkidSteerModel &iparams,
                                                   const double iscale, const double iturnScale,
                                                   const double imiddleScale)
  : OdometryArgs(iparams, iscale, iturnScale), middleScale(imiddleScale) {
}

ThreeEncoderOdometryArgs::~ThreeEncoderOdometryArgs() = default;

ThreeEncoderOdometry::ThreeEncoderOdometry(const ThreeEncoderSkidSteerModel &imodel,
                                           const double iscale, const double iturnScale,
                                           const double imiddleScale)
  : Odometry(imodel, iscale, iturnScale), middleScale(imiddleScale), model(imodel) {
}

ThreeEncoderOdometry::~ThreeEncoderOdometry() = default;

void ThreeEncoderOdometry::loop() {
  uint32_t now = millis();
  std::valarray<int> newTicks{0, 0, 0}, tickDiff{0, 0, 0};

  while (true) {
    newTicks = model.getSensorVals();
    tickDiff = newTicks - lastTicks;
    mm = (static_cast<double>(tickDiff[1] + tickDiff[0]) / 2.0) * scale;
    lastTicks = newTicks;

    state.theta += (static_cast<double>(tickDiff[1] - tickDiff[0]) / 2.0) * turnScale;
    if (state.theta > 180)
      state.theta -= 360;
    else if (state.theta < -180)
      state.theta += 360;

    state.x += mm * std::cos(state.theta) + (tickDiff[2] * middleScale) * std::sin(state.theta);
    state.y += mm * std::sin(state.theta) + (tickDiff[2] * middleScale) * std::cos(state.theta);

    task_delay_until(&now, 15);
  }
}

void ThreeEncoderOdometry::trampoline(void *context) {
  static_cast<ThreeEncoderOdometry *>(context)->loop();
}
} // namespace okapi
