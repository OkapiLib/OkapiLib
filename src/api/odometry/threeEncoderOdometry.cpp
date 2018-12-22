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
    logger->error(msg);
    throw std::invalid_argument(msg);
  }

  if (ichassisScales.middleWheelDistance == 0_m) {
    std::string msg = "ThreeEncoderOdometry: Middle wheel distance cannot be zero.";
    logger->error(msg);
    throw std::invalid_argument(msg);
  }
}

std::tuple<OdomState, double, double>
ThreeEncoderOdometry::odomMathStep(std::valarray<std::int32_t> &tickDiff, const QTime &deltaT) {
  const auto [superState, superSinTheta, superCosTheta] = Odometry::odomMathStep(tickDiff, deltaT);

  const auto Sm = (tickDiff[2] / chassisScales.middle) * meter;
  const auto mB = chassisScales.middleWheelDistance;
  const auto deltaMiddle = Sm + ((superState.theta / 360_deg) * 1_pi * mB);

  return std::make_tuple(OdomState{superState.x + deltaMiddle * superSinTheta,
                                   superState.y + deltaMiddle * superCosTheta,
                                   superState.theta},
                         superSinTheta,
                         superCosTheta);
}

void ThreeEncoderOdometry::trampoline(void *context) {
  static_cast<ThreeEncoderOdometry *>(context)->loop();
}
} // namespace okapi
