/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/chassis/controller/odomChassisControllerIntegrated.hpp"
#include "okapi/odometry/odomMath.hpp"
#include <cmath>

namespace okapi {
OdomChassisControllerIntegrated::OdomChassisControllerIntegrated(Motor ileftSideMotor,
                                                                 Motor irightSideMotor,
                                                                 const double iscale,
                                                                 const double iturnScale,
                                                                 const float imoveThreshold)
  : OdomChassisControllerIntegrated(std::make_shared<Motor>(ileftSideMotor),
                                    std::make_shared<Motor>(irightSideMotor), iscale, iturnScale,
                                    imoveThreshold) {
}

OdomChassisControllerIntegrated::OdomChassisControllerIntegrated(MotorGroup ileftSideMotor,
                                                                 MotorGroup irightSideMotor,
                                                                 const double iscale,
                                                                 const double iturnScale,
                                                                 const float imoveThreshold)
  : OdomChassisControllerIntegrated(std::make_shared<MotorGroup>(ileftSideMotor),
                                    std::make_shared<MotorGroup>(irightSideMotor), iscale,
                                    iturnScale, imoveThreshold) {
}

OdomChassisControllerIntegrated::OdomChassisControllerIntegrated(
  std::shared_ptr<AbstractMotor> ileftSideMotor, std::shared_ptr<AbstractMotor> irightSideMotor,
  const double iscale, const double iturnScale, const float imoveThreshold)
  : OdomChassisControllerIntegrated(
      std::make_shared<SkidSteerModel>(ileftSideMotor, irightSideMotor), ileftSideMotor,
      irightSideMotor, iscale, iturnScale, imoveThreshold) {
}

OdomChassisControllerIntegrated::OdomChassisControllerIntegrated(
  std::shared_ptr<SkidSteerModel> imodel, const double iscale, const double iturnScale,
  const float imoveThreshold)
  : OdomChassisControllerIntegrated(imodel, imodel->getLeftSideMotor(), imodel->getRightSideMotor(),
                                    iscale, iturnScale, imoveThreshold) {
}

OdomChassisControllerIntegrated::OdomChassisControllerIntegrated(
  std::shared_ptr<SkidSteerModel> imodel, std::shared_ptr<AbstractMotor> ileftSideMotor,
  std::shared_ptr<AbstractMotor> irightSideMotor, const double iscale, const double iturnScale,
  const float imoveThreshold)
  : OdomChassisControllerIntegrated(
      imodel, iscale, iturnScale, AsyncPosIntegratedControllerArgs(ileftSideMotor),
      AsyncPosIntegratedControllerArgs(irightSideMotor), imoveThreshold) {
}

OdomChassisControllerIntegrated::OdomChassisControllerIntegrated(
  std::shared_ptr<SkidSteerModel> imodel, const double iscale, const double iturnScale,
  const AsyncPosIntegratedControllerArgs &ileftControllerParams,
  const AsyncPosIntegratedControllerArgs &irightControllerParams, const double imoveThreshold)
  : ChassisController(imodel),
    OdomChassisController(OdometryArgs(imodel, iscale, iturnScale), imoveThreshold),
    ChassisControllerIntegrated(imodel, ileftControllerParams, irightControllerParams) {
}

void OdomChassisControllerIntegrated::driveToPoint(const float ix, const float iy,
                                                   const bool ibackwards, const float ioffset) {
  DistanceAndAngle daa = OdomMath::computeDistanceAndAngleToPoint(ix, iy, odom.getState());

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

void OdomChassisControllerIntegrated::turnToAngle(const float iangle) {
  ChassisControllerIntegrated::turnAngle(iangle - odom.getState().theta);
}
} // namespace okapi
