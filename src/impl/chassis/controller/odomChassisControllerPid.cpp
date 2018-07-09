/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/impl/chassis/controller/odomChassisControllerPid.hpp"
#include "okapi/impl/odometry/odomMath.hpp"
#include <cmath>

namespace okapi {
OdomChassisControllerPID::OdomChassisControllerPID(
  Motor ileftSideMotor, Motor irightSideMotor, const double iscale, const double iturnScale,
  const IterativePosPIDControllerArgs &idistanceArgs,
  const IterativePosPIDControllerArgs &iangleArgs, const float imoveThreshold)
  : OdomChassisControllerPID(std::make_shared<Motor>(ileftSideMotor),
                             std::make_shared<Motor>(irightSideMotor), iscale, iturnScale,
                             idistanceArgs, iangleArgs, imoveThreshold) {
}

OdomChassisControllerPID::OdomChassisControllerPID(
  MotorGroup ileftSideMotor, MotorGroup irightSideMotor, const double iscale,
  const double iturnScale, const IterativePosPIDControllerArgs &idistanceArgs,
  const IterativePosPIDControllerArgs &iangleArgs, const float imoveThreshold)
  : OdomChassisControllerPID(std::make_shared<MotorGroup>(ileftSideMotor),
                             std::make_shared<MotorGroup>(irightSideMotor), iscale, iturnScale,
                             idistanceArgs, iangleArgs, imoveThreshold) {
}

OdomChassisControllerPID::OdomChassisControllerPID(
  std::shared_ptr<AbstractMotor> ileftSideMotor, std::shared_ptr<AbstractMotor> irightSideMotor,
  const double iscale, const double iturnScale, const IterativePosPIDControllerArgs &idistanceArgs,
  const IterativePosPIDControllerArgs &iangleArgs, const float imoveThreshold)
  : OdomChassisControllerPID(std::make_shared<SkidSteerModel>(ileftSideMotor, irightSideMotor),
                             iscale, iturnScale, idistanceArgs, iangleArgs, imoveThreshold) {
}

OdomChassisControllerPID::OdomChassisControllerPID(
  std::shared_ptr<SkidSteerModel> imodel, const double iscale, const double iturnScale,
  const IterativePosPIDControllerArgs &idistanceArgs,
  const IterativePosPIDControllerArgs &iangleArgs, const float imoveThreshold)
  : ChassisController(imodel),
    OdomChassisController(OdometryArgs(imodel, iscale, iturnScale), imoveThreshold),
    ChassisControllerPID(imodel, idistanceArgs, iangleArgs) {
}

void OdomChassisControllerPID::driveToPoint(const float ix, const float iy, const bool ibackwards,
                                            const float ioffset) {
  DistanceAndAngle daa = OdomMath::computeDistanceAndAngleToPoint(ix, iy, odom.getState());

  if (ibackwards) {
    daa.theta += 180;
    daa.length *= -1;
  }

  if (std::abs(daa.theta) > 1) {
    ChassisControllerPID::turnAngle(daa.theta);
  }

  if (std::abs(daa.length - ioffset) > moveThreshold) {
    ChassisControllerPID::moveDistance(static_cast<int>(daa.length - ioffset));
  }
}

void OdomChassisControllerPID::turnToAngle(const float iangle) {
  ChassisControllerPID::turnAngle(iangle - odom.getState().theta);
}
} // namespace okapi
