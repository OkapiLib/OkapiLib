/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/chassis/controller/odomChassisControllerPid.hpp"
#include "okapi/api/odometry/odomMath.hpp"
#include <cmath>

namespace okapi {
OdomChassisControllerPID::OdomChassisControllerPID(
  const TimeUtil &itimeUtil,
  const std::shared_ptr<SkidSteerModel> &imodel,
  std::unique_ptr<Odometry> iodometry,
  std::unique_ptr<IterativePosPIDController> idistanceController,
  std::unique_ptr<IterativePosPIDController> iangleController,
  std::unique_ptr<IterativePosPIDController> iturnController,
  const AbstractMotor::GearsetRatioPair &igearset,
  const ChassisScales &iscales,
  const QLength &imoveThreshold,
  const QAngle &iturnThreshold,
  const std::shared_ptr<Logger> &ilogger)
  : ChassisController(imodel, imodel->getMaxVelocity(), imodel->getMaxVoltage()),
    OdomChassisController(imodel, std::move(iodometry), imoveThreshold, iturnThreshold),
    ChassisControllerPID(itimeUtil,
                         imodel,
                         std::move(idistanceController),
                         std::move(iangleController),
                         std::move(iturnController),
                         igearset,
                         iscales),
    logger(ilogger) {
}

void OdomChassisControllerPID::driveToPoint(const QLength ix,
                                            const QLength iy,
                                            const bool ibackwards,
                                            const QLength ioffset) {
  DistanceAndAngle daa = OdomMath::computeDistanceAndAngleToPoint(ix, iy, odom->getState());

  if (ibackwards) {
    daa.theta += 180_deg;
    daa.length *= -1;
  }

  logger->info("OdomChassisControllerPID: Computed length of " +
               std::to_string(daa.length.convert(meter)) + " meters and angle of " +
               std::to_string(daa.theta.convert(degree)) + " degrees");

  if (daa.theta.abs() > turnThreshold) {
    ChassisControllerPID::turnAngle(daa.theta);
  }

  if ((daa.length - ioffset).abs() > moveThreshold) {
    ChassisControllerPID::moveDistance(daa.length - ioffset);
  }
}

void OdomChassisControllerPID::turnToAngle(const QAngle iangle) {
  const auto angleDiff = iangle - odom->getState().theta;
  if (angleDiff > turnThreshold) {
    ChassisControllerPID::turnAngle(iangle - odom->getState().theta);
  }
}
} // namespace okapi
