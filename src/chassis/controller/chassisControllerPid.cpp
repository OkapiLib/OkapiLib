/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/chassis/controller/chassisControllerPid.hpp"
#include "okapi/util/timer.hpp"
#include <cmath>

namespace okapi {
ChassisControllerPID::ChassisControllerPID(Motor ileftSideMotor, Motor irightSideMotor,
                                           const IterativePosPIDControllerArgs &idistanceArgs,
                                           const IterativePosPIDControllerArgs &iangleArgs,
                                           const double istraightScale, const double iturnScale)
  : ChassisControllerPID(std::make_shared<Motor>(ileftSideMotor),
                         std::make_shared<Motor>(irightSideMotor), idistanceArgs, iangleArgs,
                         istraightScale, iturnScale) {
}

ChassisControllerPID::ChassisControllerPID(MotorGroup ileftSideMotor, MotorGroup irightSideMotor,
                                           const IterativePosPIDControllerArgs &idistanceArgs,
                                           const IterativePosPIDControllerArgs &iangleArgs,
                                           const double istraightScale, const double iturnScale)
  : ChassisControllerPID(std::make_shared<MotorGroup>(ileftSideMotor),
                         std::make_shared<MotorGroup>(irightSideMotor), idistanceArgs, iangleArgs,
                         istraightScale, iturnScale) {
}

ChassisControllerPID::ChassisControllerPID(Motor itopLeftMotor, Motor itopRightMotor,
                                           Motor ibottomRightMotor, Motor ibottomLeftMotor,
                                           const IterativePosPIDControllerArgs &idistanceArgs,
                                           const IterativePosPIDControllerArgs &iangleArgs,
                                           const double istraightScale, const double iturnScale)
  : ChassisControllerPID(
      std::make_shared<Motor>(itopLeftMotor), std::make_shared<Motor>(itopRightMotor),
      std::make_shared<Motor>(ibottomRightMotor), std::make_shared<Motor>(ibottomLeftMotor),
      idistanceArgs, iangleArgs, istraightScale, iturnScale) {
}

ChassisControllerPID::ChassisControllerPID(std::shared_ptr<AbstractMotor> ileftSideMotor,
                                           std::shared_ptr<AbstractMotor> irightSideMotor,
                                           const IterativePosPIDControllerArgs &idistanceArgs,
                                           const IterativePosPIDControllerArgs &iangleArgs,
                                           const double istraightScale, const double iturnScale)
  : ChassisController(std::make_shared<SkidSteerModel>(ileftSideMotor, irightSideMotor)),
    distancePid(idistanceArgs),
    anglePid(iangleArgs),
    straightScale(istraightScale),
    turnScale(iturnScale) {
  setEncoderUnits(E_MOTOR_ENCODER_COUNTS);
}

ChassisControllerPID::ChassisControllerPID(std::shared_ptr<AbstractMotor> itopLeftMotor,
                                           std::shared_ptr<AbstractMotor> itopRightMotor,
                                           std::shared_ptr<AbstractMotor> ibottomRightMotor,
                                           std::shared_ptr<AbstractMotor> ibottomLeftMotor,
                                           const IterativePosPIDControllerArgs &idistanceArgs,
                                           const IterativePosPIDControllerArgs &iangleArgs,
                                           const double istraightScale, const double iturnScale)
  : ChassisController(std::make_shared<XDriveModel>(itopLeftMotor, itopRightMotor,
                                                    ibottomRightMotor, ibottomLeftMotor)),
    distancePid(idistanceArgs),
    anglePid(iangleArgs),
    straightScale(istraightScale),
    turnScale(iturnScale) {
  setEncoderUnits(E_MOTOR_ENCODER_COUNTS);
}

ChassisControllerPID::ChassisControllerPID(std::shared_ptr<ChassisModel> imodel,
                                           const IterativePosPIDControllerArgs &idistanceArgs,
                                           const IterativePosPIDControllerArgs &iangleArgs,
                                           const double istraightScale, const double iturnScale)
  : ChassisController(imodel),
    distancePid(idistanceArgs),
    anglePid(iangleArgs),
    straightScale(istraightScale),
    turnScale(iturnScale) {
  setEncoderUnits(E_MOTOR_ENCODER_COUNTS);
}

void ChassisControllerPID::moveDistance(const int itarget) {
  distancePid.reset();
  anglePid.reset();

  const double newTarget = itarget * straightScale;
  distancePid.setTarget(newTarget);
  anglePid.setTarget(newTarget);

  std::uint32_t prevWakeTime = pros::millis();
  const auto encStartVals = model->getSensorVals();
  std::valarray<std::int32_t> encVals;
  double distanceElapsed = 0, angleChange = 0;

  while (!distancePid.isSettled() && !anglePid.isSettled()) {
    encVals = model->getSensorVals() - encStartVals;
    distanceElapsed = static_cast<double>((encVals[0] + encVals[1])) / 2.0;
    angleChange = static_cast<double>(encVals[1] - encVals[0]);
    model->driveVector(distancePid.step(distanceElapsed), anglePid.step(angleChange));
    pros::Task::delay_until(&prevWakeTime, 10);
  }

  model->stop();
}

void ChassisControllerPID::turnAngle(float idegTarget) {
  anglePid.reset();

  const double newTarget = idegTarget * turnScale;
  anglePid.setTarget(newTarget);

  std::uint32_t prevWakeTime = pros::millis();
  const auto encStartVals = model->getSensorVals();
  std::valarray<std::int32_t> encVals;
  double angleChange = 0;

  while (!anglePid.isSettled()) {
    encVals = model->getSensorVals() - encStartVals;
    angleChange = static_cast<double>(encVals[1] - encVals[0]);
    model->rotate(anglePid.step(angleChange));
    pros::Task::delay_until(&prevWakeTime, 10);
  }

  model->stop();
}
} // namespace okapi
