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
  const QAngle iturnThreshold,
  const std::shared_ptr<Logger> &ilogger)
  : ChassisController(imodel, imodel->getMaxVelocity(), imodel->getMaxVoltage()),
    OdomChassisController(imodel, std::move(iodometry), imoveThreshold, iturnThreshold),
    ChassisControllerIntegrated(itimeUtil,
                                imodel,
                                std::move(ileftController),
                                std::move(irightController),
                                igearset,
                                iscales),
    logger(ilogger) {
}

void OdomChassisControllerIntegrated::driveToPoint(const QLength ix,
                                                   const QLength iy,
                                                   const bool ibackwards,
                                                   const QLength ioffset) {
  auto [length, angle] = OdomMath::computeDistanceAndAngleToPoint(ix, iy, odom->getState());

  if (ibackwards) {
    length *= -1;
    angle += 180_deg;
  }

  LOG_INFO("OdomChassisControllerIntegrated: Computed length of " +
           std::to_string(length.convert(meter)) + " meters and angle of " +
           std::to_string(angle.convert(degree)) + " degrees");

  if (angle.abs() > turnThreshold) {
    LOG_INFO("OdomChassisControllerIntegrated: Turning " + std::to_string(angle.convert(degree)) +
             " degrees");
    ChassisControllerIntegrated::turnAngle(angle);
  }

  if ((length - ioffset).abs() > moveThreshold) {
    LOG_INFO("OdomChassisControllerIntegrated: Driving " +
             std::to_string((length - ioffset).convert(meter)) + " meters");
    ChassisControllerIntegrated::moveDistance(length - ioffset);
  }
}

void OdomChassisControllerIntegrated::turnToAngle(const QAngle iangle) {
  const auto angleDiff = iangle - odom->getState().theta;
  if (angleDiff.abs() > turnThreshold) {
    LOG_INFO("OdomChassisControllerIntegrated: Turning " +
             std::to_string(angleDiff.convert(degree)) + " degrees");
    ChassisControllerIntegrated::turnAngle(angleDiff);
  }
}
} // namespace okapi
