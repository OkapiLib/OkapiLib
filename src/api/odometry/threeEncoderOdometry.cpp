/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/odometry/threeEncoderOdometry.hpp"

namespace okapi {
ThreeEncoderOdometry::ThreeEncoderOdometry(
  std::shared_ptr<ReadOnlyChassisModel> imodel, const ChassisScales &ichassisScales,
  const Supplier<std::unique_ptr<AbstractRate>> &irateSupplier)
  : Odometry(imodel, ichassisScales, irateSupplier.get()),
    model(imodel),
    rate(irateSupplier.get()) {
}

void ThreeEncoderOdometry::loop() {
  std::valarray<std::int32_t> newTicks{0, 0, 0}, tickDiff{0, 0, 0};

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"
  while (true) {
    newTicks = model->getSensorVals();
    tickDiff = newTicks - lastTicks;
    lastTicks = newTicks;

    mm = (static_cast<double>(tickDiff[1] + tickDiff[0]) / 2.0) * chassisScales.straight;

    state.theta += (static_cast<double>(tickDiff[1] - tickDiff[0]) / 2.0) * chassisScales.turn;
    if (state.theta > 180)
      state.theta -= 360;
    else if (state.theta < -180)
      state.theta += 360;

    state.x +=
      mm * std::cos(state.theta) + (tickDiff[2] * chassisScales.middle) * std::sin(state.theta);
    state.y +=
      mm * std::sin(state.theta) + (tickDiff[2] * chassisScales.middle) * std::cos(state.theta);

    rate->delayUntil(10);
  }
#pragma clang diagnostic pop
}

void ThreeEncoderOdometry::trampoline(void *context) {
  static_cast<ThreeEncoderOdometry *>(context)->loop();
}
} // namespace okapi
