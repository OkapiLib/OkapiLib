/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/odometry/odometry.hpp"
#include "okapi/api/util/mathUtil.hpp"
#include <cmath>

namespace okapi {
OdomState::OdomState() = default;

OdomState::OdomState(const double ix, const double iy, const double itheta)
  : x(ix), y(iy), theta(itheta) {
}

OdomState::~OdomState() = default;

OdometryArgs::OdometryArgs(std::shared_ptr<ReadOnlyChassisModel> imodel, const ChassisScales &ichassisScales)
  : model(imodel), chassisScales(ichassisScales) {
}

OdometryArgs::~OdometryArgs() = default;

Odometry::Odometry(std::shared_ptr<ReadOnlyChassisModel> imodel, const ChassisScales &ichassisScales, std::unique_ptr<AbstractRate> irate)
  : model(imodel),
    rate(std::move(irate)),
    chassisScales(ichassisScales),
    lastTicks{0, 0},
    mm(0) {
}

Odometry::Odometry(const OdometryArgs &iparams)
  : model(iparams.model), chassisScales(iparams.chassisScales) {
}

Odometry::~Odometry() = default;

void Odometry::setScales(const ChassisScales &ichassisScales) {
  chassisScales = ichassisScales;
}

void Odometry::loop() {
  std::valarray<std::int32_t> newTicks{0, 0}, tickDiff{0, 0};

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"
  while (true) {
    newTicks = model->getSensorVals();
    tickDiff = newTicks - lastTicks;
    lastTicks = newTicks;

    mm = (static_cast<double>(tickDiff[1] + tickDiff[0]) / 2.0) * chassisScales.straight;

    state.theta += (tickDiff[1] - tickDiff[0]) * chassisScales.turn;
    if (state.theta > 180)
      state.theta -= 360;
    else if (state.theta < -180)
      state.theta += 360;

    state.x += mm * std::cos(state.theta * degreeToRadian);
    state.y += mm * std::sin(state.theta * degreeToRadian);

    rate->delayUntil(10);
  }
#pragma clang diagnostic pop
}

void Odometry::trampoline(void *context) {
  static_cast<Odometry *>(context)->loop();
}

OdomState Odometry::getState() const {
  return state;
}

void Odometry::setState(const OdomState &istate) {
  state = istate;
  lastTicks[0] = 0;
  lastTicks[1] = 0;
}

} // namespace okapi
