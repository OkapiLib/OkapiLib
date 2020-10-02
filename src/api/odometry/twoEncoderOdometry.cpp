/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/odometry/twoEncoderOdometry.hpp"
#include "okapi/api/units/QAngularSpeed.hpp"
#include "okapi/api/util/mathUtil.hpp"
#include <cmath>

namespace okapi {
TwoEncoderOdometry::TwoEncoderOdometry(const TimeUtil &itimeUtil,
                                       const std::shared_ptr<ReadOnlyChassisModel> &imodel,
                                       const ChassisScales &ichassisScales,
                                       const std::shared_ptr<Logger> &ilogger)
  : logger(ilogger),
    rate(itimeUtil.getRate()),
    timer(itimeUtil.getTimer()),
    model(imodel),
    chassisScales(ichassisScales) {
}

void TwoEncoderOdometry::setScales(const ChassisScales &ichassisScales) {
  chassisScales = ichassisScales;
}

void TwoEncoderOdometry::step() {
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

OdomState TwoEncoderOdometry::odomMathStep(const std::valarray<std::int32_t> &itickDiff,
                                           const QTime &) {
  if (itickDiff.size() < 2) {
    LOG_ERROR_S("TwoEncoderOdometry: itickDiff did not have at least two elements.");
    return OdomState{};
  }

  for (auto &&elem : itickDiff) {
    if (std::abs(elem) > maximumTickDiff) {
      LOG_ERROR("TwoEncoderOdometry: A tick diff (" + std::to_string(elem) +
                ") was greater than the maximum allowable diff (" +
                std::to_string(maximumTickDiff) + "). Skipping this odometry step.");
      return OdomState{};
    }
  }

  const double deltaL = itickDiff[0] / chassisScales.straight;
  const double deltaR = itickDiff[1] / chassisScales.straight;

  double deltaTheta = (deltaL - deltaR) / chassisScales.wheelTrack.convert(meter);
  double localOffX, localOffY;

  if (deltaTheta != 0) {
    localOffX = 2 * std::sin(deltaTheta / 2) * chassisScales.middleWheelDistance.convert(meter);
    localOffY = 2 * std::sin(deltaTheta / 2) *
                (deltaR / deltaTheta + chassisScales.wheelTrack.convert(meter) / 2);
  } else {
    localOffX = 0;
    localOffY = deltaR;
  }

  double avgA = state.theta.convert(radian) + (deltaTheta / 2);

  double polarR = std::sqrt(localOffX * localOffX + localOffY * localOffY);
  double polarA = std::atan2(localOffY, localOffX) - avgA;

  double dX = std::sin(polarA) * polarR;
  double dY = std::cos(polarA) * polarR;

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

OdomState TwoEncoderOdometry::getState(const StateMode &imode) const {
  if (imode == StateMode::FRAME_TRANSFORMATION) {
    return state;
  } else {
    return OdomState{state.y, state.x, state.theta};
  }
}

void TwoEncoderOdometry::setState(const OdomState &istate, const StateMode &imode) {
  LOG_DEBUG("State set to: " + istate.str());
  if (imode == StateMode::FRAME_TRANSFORMATION) {
    state = istate;
  } else {
    state = OdomState{istate.y, istate.x, istate.theta};
  }
}

std::shared_ptr<ReadOnlyChassisModel> TwoEncoderOdometry::getModel() {
  return model;
}

ChassisScales TwoEncoderOdometry::getScales() {
  return chassisScales;
}
} // namespace okapi
