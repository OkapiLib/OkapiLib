/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/odometry/odometry.hpp"
#include "okapi/api/units/QAngularSpeed.hpp"
#include "okapi/api/util/mathUtil.hpp"
#include <cmath>

namespace okapi {
Odometry::Odometry(const std::shared_ptr<ReadOnlyChassisModel> &imodel,
                   const ChassisScales &ichassisScales,
                   const TimeUtil &itimeUtil,
                   const std::shared_ptr<Logger> &ilogger)
  : logger(ilogger),
    model(imodel),
    rate(itimeUtil.getRate()),
    timer(itimeUtil.getTimer()),
    chassisScales(ichassisScales) {
}

Odometry::~Odometry() {
  dtorCalled.store(true, std::memory_order_release);
}

void Odometry::setScales(const ChassisScales &ichassisScales) {
  chassisScales = ichassisScales;
}

void Odometry::loop() {
  while (!dtorCalled.load(std::memory_order_acquire)) {
    step();
    rate->delayUntil(10_ms);
  }
}

void Odometry::step() {
  const auto deltaT = timer->getDt();

  if (deltaT.getValue() != 0) {
    newTicks = model->getSensorVals();
    tickDiff = newTicks - lastTicks;
    lastTicks = newTicks;

    const auto Sl = (tickDiff[0] / chassisScales.straight) * meter;
    const auto Sr = (tickDiff[1] / chassisScales.straight) * meter;
    const auto Vl = Sl / deltaT;
    const auto Vr = Sr / deltaT;
    const auto b = chassisScales.wheelbaseWidth;
    auto turnRadius = (b * (Vr + Vl)) / (2 * (Vr - Vl));

    QLength deltaX;
    QLength deltaY;
    QAngle deltaTheta;

    if ((Sr - Sl).abs() < 0.0000001_mm) {
      turnRadius = (Sr + Sl) / 2;
      deltaTheta = 0_deg;
    } else {
      deltaTheta = (((Vr - Vl) * deltaT) / b) * radian;

      if (isnan(deltaTheta.getValue())) {
        deltaTheta = 0_deg;
      }
    }

    deltaX = turnRadius * std::sin((state.theta + deltaTheta).convert(radian));
    deltaY = turnRadius * std::cos((state.theta + deltaTheta).convert(radian));

    if (isnan(deltaX.getValue())) {
      deltaX = 0_m;
    }

    if (isnan(deltaY.getValue())) {
      deltaY = 0_m;
    }

    state.x += deltaX;
    state.y += deltaY;
    state.theta += deltaTheta;
  }
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
  dtorCalled.store(true, std::memory_order_release);
}
} // namespace okapi
