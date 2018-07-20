/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/chassis/controller/odomChassisControllerIntegrated.hpp"
#include "okapi/api/odometry/odomMath.hpp"
#include "okapi/api/util/timeUtil.hpp"
#include <cmath>

namespace okapi {
OdomChassisControllerIntegrated::OdomChassisControllerIntegrated(const TimeUtil &itimeUtil,
  std::shared_ptr<SkidSteerModel> imodel, std::unique_ptr<Odometry> iodometry,
  const AsyncPosIntegratedControllerArgs &ileftControllerParams,
  const AsyncPosIntegratedControllerArgs &irightControllerParams, const double imoveThreshold)
  : ChassisController(imodel),
    OdomChassisController(imodel, std::move(iodometry), imoveThreshold),
    ChassisControllerIntegrated(itimeUtil, imodel, ileftControllerParams,
                                irightControllerParams) {
}

void OdomChassisControllerIntegrated::driveToPoint(const double ix, const double iy,
                                                   const bool ibackwards, const double ioffset) {
  DistanceAndAngle daa = OdomMath::computeDistanceAndAngleToPoint(ix, iy, odom->getState());

  if (ibackwards) {
    daa.theta += 180;
    daa.length *= -1;
  }

  if (std::fabs(daa.theta) > 1) {
    ChassisControllerIntegrated::turnAngle(daa.theta);
  }

  if (std::fabs(daa.length - ioffset) > moveThreshold) {
    ChassisControllerIntegrated::moveDistance(static_cast<int>(daa.length - ioffset));
  }
}

void OdomChassisControllerIntegrated::turnToAngle(const double iangle) {
  ChassisControllerIntegrated::turnAngle(iangle - odom->getState().theta);
}
} // namespace okapi
