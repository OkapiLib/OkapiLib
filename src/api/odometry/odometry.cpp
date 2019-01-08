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

    const auto newState = odomMathStep(tickDiff, deltaT);

    state.x += newState.x;
    state.y += newState.y;
    state.theta += newState.theta;
  }
}

OdomState Odometry::odomMathStep(std::valarray<std::int32_t> &tickDiff, const QTime &) {
  const double deltaL = tickDiff[0] / chassisScales.straight;
  const double deltaR = tickDiff[1] / chassisScales.straight;

  double deltaTheta = (deltaL - deltaR) / chassisScales.wheelbaseWidth.convert(meter);
  double localOffX, localOffY;

  if (deltaTheta != 0) {
    localOffX = 2 * sin(deltaTheta / 2) * chassisScales.middleWheelDistance.convert(meter);
    localOffY = 2 * sin(deltaTheta / 2) *
                (deltaR / deltaTheta + chassisScales.wheelbaseWidth.convert(meter) / 2);
  } else {
    localOffX = 0;
    localOffY = deltaR;
  }

  double avgA = state.theta.convert(radian) + (deltaTheta / 2);

  double polarR = sqrt(localOffX * localOffX + localOffY * localOffY);
  double polarA = atan2(localOffY, localOffX) - avgA;

  double dX = sin(polarA) * polarR;
  double dY = cos(polarA) * polarR;

  if (isnan(dX)) {
    dX = 0;
  }

  if (isnan(dY)) {
    dY = 0;
  }

  if (isnan(deltaTheta)) {
    deltaTheta = 0;
  }

  return OdomState{dX * meter, dY * meter, deltaTheta * radian};
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
