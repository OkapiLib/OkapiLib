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
Odometry::Odometry(const TimeUtil &itimeUtil,
                   const std::shared_ptr<ReadOnlyChassisModel> &imodel,
                   const ChassisScales &ichassisScales,
                   const QSpeed &iwheelVelDelta,
                   const std::shared_ptr<Logger> &ilogger)
  : rate(itimeUtil.getRate()),
    timer(itimeUtil.getTimer()),
    model(imodel),
    chassisScales(ichassisScales),
    wheelVelDelta(iwheelVelDelta),
    logger(ilogger) {
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

    const auto [newState, sinTheta, cosTheta] = odomMathStep(tickDiff, deltaT);

    state.x += newState.x;
    state.y += newState.y;
    state.theta += newState.theta;
  }
}

std::tuple<OdomState, double, double> Odometry::odomMathStep(std::valarray<std::int32_t> &tickDiff,
                                                             const QTime &deltaT) {
  const auto Sl = (tickDiff[0] / chassisScales.straight) * meter;
  const auto Sr = (tickDiff[1] / chassisScales.straight) * meter;
  const auto Vl = Sl / deltaT;
  const auto Vr = Sr / deltaT;
  const auto b = chassisScales.wheelbaseWidth;
  auto turnRadius = (b * (Vr + Vl)) / (2 * (Vr - Vl));

  QLength deltaX;
  QLength deltaY;
  QAngle deltaTheta;
  double sinTheta;
  double cosTheta;

  if ((Vr - Vl).abs() < wheelVelDelta) {
    turnRadius = (Sr + Sl) / 2;
    deltaTheta = 0_deg;

    sinTheta = std::sin(state.theta.convert(radian));
    cosTheta = std::cos(state.theta.convert(radian));
  } else {
    deltaTheta = (((Vr - Vl) * deltaT) / b) * radian;

    if (isnan(deltaTheta.getValue())) {
      deltaTheta = 0_deg;
    }

    sinTheta = std::sin((Vr - Vl).convert(mps) * deltaT.convert(second) / b.convert(meter));
    cosTheta = std::cos((Vr - Vl).convert(mps) * deltaT.convert(second) / b.convert(meter)) - 1;
  }

  deltaX = turnRadius * sinTheta;
  deltaY = turnRadius * cosTheta;

  if (isnan(deltaX.getValue())) {
    deltaX = 0_m;
  }

  if (isnan(deltaY.getValue())) {
    deltaY = 0_m;
  }

  return std::make_tuple(OdomState{deltaX, deltaY, deltaTheta}, sinTheta, cosTheta);
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
}

void Odometry::stopLooping() {
  dtorCalled.store(true, std::memory_order_release);
}
} // namespace okapi
