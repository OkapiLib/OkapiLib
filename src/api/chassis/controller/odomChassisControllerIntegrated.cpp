/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/chassis/controller/odomChassisControllerIntegrated.hpp"
#include "okapi/api/odometry/odomMath.hpp"
#include <cmath>

namespace okapi {
OdomChassisControllerIntegrated::OdomChassisControllerIntegrated(
  const TimeUtil &itimeUtil,
  std::shared_ptr<SkidSteerModel> imodel,
  std::unique_ptr<Odometry> iodometry,
  std::unique_ptr<AsyncPosIntegratedController> ileftController,
  std::unique_ptr<AsyncPosIntegratedController> irightController,
  const AbstractMotor::GearsetRatioPair igearset,
  const ChassisScales &iscales,
  const QLength imoveThreshold,
  const QAngle iturnThreshold)
  : ChassisController(imodel),
    OdomChassisController(imodel, std::move(iodometry), imoveThreshold, iturnThreshold),
    ChassisControllerIntegrated(itimeUtil,
                                imodel,
                                std::move(ileftController),
                                std::move(irightController),
                                igearset,
                                iscales),
    logger(Logger::instance()) {
}

void OdomChassisControllerIntegrated::driveToPoint(const QLength ix,
                                                   const QLength iy,
                                                   const bool ibackwards,
                                                   const QLength ioffset) {
  DistanceAndAngle daa = OdomMath::computeDistanceAndAngleToPoint(ix, iy, odom->getState());

  if (ibackwards) {
    daa.theta += 180_deg;
    daa.length *= -1;
  }

  logger->info("OdomChassisControllerIntegrated: Computed length of " +
               std::to_string(daa.length.convert(meter)) + " meters and angle of " +
               std::to_string(daa.theta.convert(degree)) + " degrees");

  if (daa.theta.abs() > turnThreshold) {
    ChassisControllerIntegrated::turnAngle(daa.theta);
  }

  if ((daa.length - ioffset).abs() > moveThreshold) {
    ChassisControllerIntegrated::moveDistance(daa.length - ioffset);
  }
}

void OdomChassisControllerIntegrated::turnToAngle(const QAngle iangle) {
  const auto angleDiff = iangle - odom->getState().theta;
  if (angleDiff > turnThreshold) {
    ChassisControllerIntegrated::turnAngle(angleDiff);
  }
}
} // namespace okapi
