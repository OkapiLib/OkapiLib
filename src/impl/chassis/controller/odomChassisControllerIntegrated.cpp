/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/impl/chassis/controller/odomChassisControllerIntegrated.hpp"
#include "okapi/api/odometry/odomMath.hpp"
#include <cmath>
#include "okapi/impl/control/util/settledUtilFactory.hpp"
#include "okapi/impl/util/rate.hpp"

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
      std::make_unique<SkidSteerModel>(ileftSideMotor, irightSideMotor), ileftSideMotor,
      irightSideMotor, iscale, iturnScale, imoveThreshold) {
}

OdomChassisControllerIntegrated::OdomChassisControllerIntegrated(
  std::unique_ptr<SkidSteerModel> imodel, const double iscale, const double iturnScale,
  const float imoveThreshold)
  : OdomChassisControllerIntegrated(std::move(imodel), imodel->getLeftSideMotor(), imodel->getRightSideMotor(),
                                    iscale, iturnScale, imoveThreshold) {
}

OdomChassisControllerIntegrated::OdomChassisControllerIntegrated(
  std::unique_ptr<SkidSteerModel> imodel, std::shared_ptr<AbstractMotor> ileftSideMotor,
  std::shared_ptr<AbstractMotor> irightSideMotor, const double iscale, const double iturnScale,
  const float imoveThreshold)
  : OdomChassisControllerIntegrated(
      std::move(imodel), iscale, iturnScale, AsyncPosIntegratedControllerArgs(ileftSideMotor),
      AsyncPosIntegratedControllerArgs(irightSideMotor), imoveThreshold) {
}

OdomChassisControllerIntegrated::OdomChassisControllerIntegrated(
  std::unique_ptr<SkidSteerModel> imodel, const double iscale, const double iturnScale,
  const AsyncPosIntegratedControllerArgs &ileftControllerParams,
  const AsyncPosIntegratedControllerArgs &irightControllerParams, const double imoveThreshold)
  : ChassisController(imodel),
    OdomChassisController(OdometryArgs(imodel, iscale, iturnScale), imoveThreshold),
    ChassisControllerIntegrated(Supplier<std::unique_ptr<SettledUtil>>([](){ return SettledUtilFactory::create(); }),
                                Supplier<std::unique_ptr<AbstractRate>>([](){ return std::make_unique<Rate>(); }),
                                imodel, ileftControllerParams, irightControllerParams) {
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
