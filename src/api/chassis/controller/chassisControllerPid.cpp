/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/chassis/controller/chassisControllerPid.hpp"
#include "okapi/api/util/timer.hpp"
#include <cmath>

namespace okapi {
ChassisControllerPID::ChassisControllerPID(std::shared_ptr<AbstractMotor> ileftSideMotor,
                                           std::shared_ptr<AbstractMotor> irightSideMotor,
                                           const IterativePosPIDControllerArgs &idistanceArgs,
                                           const IterativePosPIDControllerArgs &iangleArgs,
                                           const AbstractMotor::motorGearset igearset,
                                           const ChassisScales &iscales)
  : ChassisController(std::make_shared<SkidSteerModel>(ileftSideMotor, irightSideMotor)),
    distancePid(idistanceArgs),
    anglePid(iangleArgs),
    straightScale(iscales.straight),
    turnScale(iscales.turn) {
  setGearing(igearset);
  setEncoderUnits(AbstractMotor::motorEncoderUnits::E_MOTOR_ENCODER_DEGREES);
}

ChassisControllerPID::ChassisControllerPID(std::shared_ptr<AbstractMotor> itopLeftMotor,
                                           std::shared_ptr<AbstractMotor> itopRightMotor,
                                           std::shared_ptr<AbstractMotor> ibottomRightMotor,
                                           std::shared_ptr<AbstractMotor> ibottomLeftMotor,
                                           const IterativePosPIDControllerArgs &idistanceArgs,
                                           const IterativePosPIDControllerArgs &iangleArgs,
                                           const AbstractMotor::motorGearset igearset,
                                           const ChassisScales &iscales)
  : ChassisController(std::make_shared<XDriveModel>(itopLeftMotor, itopRightMotor,
                                                    ibottomRightMotor, ibottomLeftMotor)),
    distancePid(idistanceArgs),
    anglePid(iangleArgs),
    straightScale(iscales.straight),
    turnScale(iscales.turn) {
  setGearing(igearset);
  setEncoderUnits(AbstractMotor::motorEncoderUnits::E_MOTOR_ENCODER_DEGREES);
}

ChassisControllerPID::ChassisControllerPID(std::shared_ptr<ChassisModel> imodel,
                                           const IterativePosPIDControllerArgs &idistanceArgs,
                                           const IterativePosPIDControllerArgs &iangleArgs,
                                           const AbstractMotor::motorGearset igearset,
                                           const ChassisScales &iscales)
  : ChassisController(imodel),
    distancePid(idistanceArgs),
    anglePid(iangleArgs),
    straightScale(iscales.straight),
    turnScale(iscales.turn) {
  setGearing(igearset);
  setEncoderUnits(AbstractMotor::motorEncoderUnits::E_MOTOR_ENCODER_DEGREES);
}

void ChassisControllerPID::moveDistance(const QLength itarget) {
  distancePid.reset();
  anglePid.reset();

  const double newTarget = itarget.convert(meter) * straightScale;
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

void ChassisControllerPID::moveDistance(const double itarget) {
  // Divide by straightScale so the final result turns back into motor degrees
  moveDistance((itarget / straightScale) * meter);
}

void ChassisControllerPID::turnAngle(const QAngle idegTarget) {
  anglePid.reset();

  const double newTarget = idegTarget.convert(degree) * turnScale;
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

void ChassisControllerPID::turnAngle(const double idegTarget) {
  // Divide by turnScale so the final result turns back into motor degrees
  turnAngle((idegTarget / turnScale) * degree);
}
} // namespace okapi
