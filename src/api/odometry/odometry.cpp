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
Odometry::Odometry(std::shared_ptr<ReadOnlyChassisModel> imodel,
                   const ChassisScales &ichassisScales, std::unique_ptr<AbstractRate> irate)
  : logger(Logger::instance()),
    model(imodel),
    rate(std::move(irate)),
    chassisScales(ichassisScales) {
  logger->info("Odometry: Straight scale: " + std::to_string(chassisScales.straight) +
               ", Turn scale: " + std::to_string(chassisScales.turn));
}

Odometry::~Odometry() {
  dtorCalled = true;
}

void Odometry::setScales(const ChassisScales &ichassisScales) {
  chassisScales = ichassisScales;
}

void Odometry::loop() {
  while (!dtorCalled) {
    step();
    rate->delayUntil(10_ms);
  }
}

void Odometry::step() {
  newTicks = model->getSensorVals();
  tickDiff = newTicks - lastTicks;
  lastTicks = newTicks;

  mm = (static_cast<double>(tickDiff[1] + tickDiff[0]) / 2.0) / chassisScales.straight * meter;

  state.theta += (((tickDiff[0] - tickDiff[1]) / 2.0) / chassisScales.turn) * degree;
  if (state.theta > 180_deg)
    state.theta -= 360_deg;
  else if (state.theta < -180 * degree)
    state.theta += 360_deg;

  state.x += mm * std::cos(state.theta.convert(radian));
  state.y += mm * std::sin(state.theta.convert(radian));
}

void Odometry::trampoline(void *context) {
  if (context) {
    static_cast<Odometry *>(context)->loop();
  }
}

OdomState Odometry::getState() const {
  return state;
}

void Odometry::setState(const OdomState &istate) {
  state = istate;
  lastTicks[0] = 0;
  lastTicks[1] = 0;
}

void Odometry::stopLooping() {
  dtorCalled = true;
}
} // namespace okapi
