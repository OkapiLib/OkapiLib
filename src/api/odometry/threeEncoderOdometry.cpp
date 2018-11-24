/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/odometry/threeEncoderOdometry.hpp"

namespace okapi {
ThreeEncoderOdometry::ThreeEncoderOdometry(std::shared_ptr<ReadOnlyChassisModel> imodel,
                                           const ChassisScales &ichassisScales,
                                           const TimeUtil &itimeUtil)
  : Odometry(imodel, ichassisScales, itimeUtil), model(imodel), rate(itimeUtil.getRate()) {
}

void ThreeEncoderOdometry::step() {
  newTicks = model->getSensorVals();
  tickDiff = newTicks - lastTicks;
  lastTicks = newTicks;

  mm = (static_cast<double>(tickDiff[1] + tickDiff[0]) / 2.0) * chassisScales.straight * meter;

  state.theta += (((tickDiff[0] - tickDiff[1]) / 2.0) * chassisScales.turn) * degree;
  if (state.theta > 180_deg)
    state.theta -= 360_deg;
  else if (state.theta < -180 * degree)
    state.theta += 360_deg;

  state.x += mm * std::cos(state.theta.convert(radian)) +
             (tickDiff[2] * chassisScales.middle * meter) * std::sin(state.theta.convert(radian));
  state.y += mm * std::sin(state.theta.convert(radian)) +
             (tickDiff[2] * chassisScales.middle * meter) * std::cos(state.theta.convert(radian));
}

void ThreeEncoderOdometry::trampoline(void *context) {
  static_cast<ThreeEncoderOdometry *>(context)->loop();
}
} // namespace okapi
