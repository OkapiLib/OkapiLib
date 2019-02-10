/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/odometry/threeEncoderOdometry.hpp"
#include "okapi/api/units/QSpeed.hpp"
#include <math.h>

namespace okapi {
ThreeEncoderOdometry::ThreeEncoderOdometry(const TimeUtil &itimeUtil,
                                           std::shared_ptr<ReadOnlyChassisModel> imodel,
                                           const ChassisScales &ichassisScales,

                                           const QSpeed &iwheelVelDelta,
                                           const std::shared_ptr<Logger> &ilogger)
  : Odometry(itimeUtil, imodel, ichassisScales, iwheelVelDelta, ilogger),
    logger(ilogger),
    model(imodel),
    rate(itimeUtil.getRate()) {
  if (ichassisScales.middle == 0) {
    std::string msg = "ThreeEncoderOdometry: Middle scale cannot be zero.";
    LOG_ERROR(msg);
    throw std::invalid_argument(msg);
  }

  if (ichassisScales.middleWheelDistance == 0_m) {
    std::string msg = "ThreeEncoderOdometry: Middle wheel distance cannot be zero.";
    LOG_ERROR(msg);
    throw std::invalid_argument(msg);
  }
}

OdomState ThreeEncoderOdometry::odomMathStep(std::valarray<std::int32_t> &tickDiff, const QTime &) {
  const double deltaL = tickDiff[0] / chassisScales.straight;
  const double deltaR = tickDiff[1] / chassisScales.straight;

  double deltaTheta = (deltaL - deltaR) / chassisScales.wheelbaseWidth.convert(meter);
  double localOffX, localOffY;

  const auto deltaM = static_cast<const double>(
    tickDiff[2] / chassisScales.middle -
    ((deltaTheta / 2_pi) * 1_pi * chassisScales.middleWheelDistance.convert(meter)));

  if (deltaL == deltaR) {
    localOffX = deltaM;
    localOffY = deltaR;
  } else {
    localOffX = 2 * sin(deltaTheta / 2) *
                (deltaM / deltaTheta + chassisScales.middleWheelDistance.convert(meter));
    localOffY = 2 * sin(deltaTheta / 2) *
                (deltaR / deltaTheta + chassisScales.wheelbaseWidth.convert(meter) / 2);
  }

  double avgA = state.theta.convert(radian) + (deltaTheta / 2);

  double polarR = sqrt((localOffX * localOffX) + (localOffY * localOffY));
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

void ThreeEncoderOdometry::trampoline(void *context) {
  static_cast<ThreeEncoderOdometry *>(context)->loop();
}
} // namespace okapi
