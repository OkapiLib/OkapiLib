/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/impl/chassis/controller/chassisControllerBuilder.hpp"

namespace okapi {
ChassisControllerBuilder &ChassisControllerBuilder::withMotors(const Motor &ileft,
                                                               const Motor &iright) {
  isSkidSteer = true;
  skidSteerMotors = {std::make_shared<Motor>(ileft), std::make_shared<Motor>(iright)};
  leftSensor = ileft.getEncoder();
  rightSensor = iright.getEncoder();
  return *this;
}

ChassisControllerBuilder &ChassisControllerBuilder::withMotors(const MotorGroup &ileft,
                                                               const MotorGroup &iright) {
  isSkidSteer = true;
  skidSteerMotors = {std::make_shared<MotorGroup>(ileft), std::make_shared<MotorGroup>(iright)};
  leftSensor = ileft.getEncoder();
  rightSensor = iright.getEncoder();
  return *this;
}

ChassisControllerBuilder &ChassisControllerBuilder::withMotors(const Motor &itopLeft,
                                                               const Motor &itopRight,
                                                               const Motor &ibottomRight,
                                                               const Motor &ibottomLeft) {
  isSkidSteer = false;
  xDriveMotors = {std::make_shared<Motor>(itopLeft),
                  std::make_shared<Motor>(itopRight),
                  std::make_shared<Motor>(ibottomRight),
                  std::make_shared<Motor>(ibottomLeft)};
  leftSensor = itopLeft.getEncoder();
  rightSensor = itopRight.getEncoder();
  return *this;
}

ChassisControllerBuilder &ChassisControllerBuilder::withMotors(const MotorGroup &itopLeft,
                                                               const MotorGroup &itopRight,
                                                               const MotorGroup &ibottomRight,
                                                               const MotorGroup &ibottomLeft) {
  isSkidSteer = false;
  xDriveMotors = {std::make_shared<MotorGroup>(itopLeft),
                  std::make_shared<MotorGroup>(itopRight),
                  std::make_shared<MotorGroup>(ibottomRight),
                  std::make_shared<MotorGroup>(ibottomLeft)};
  leftSensor = itopLeft.getEncoder();
  rightSensor = itopRight.getEncoder();
  return *this;
}

ChassisControllerBuilder &ChassisControllerBuilder::withSensors(const ADIEncoder &ileft,
                                                                const ADIEncoder &iright) {
  leftSensor = std::make_shared<ADIEncoder>(ileft);
  rightSensor = std::make_shared<ADIEncoder>(iright);
  return *this;
}

ChassisControllerBuilder &ChassisControllerBuilder::withSensors(const IntegratedEncoder &ileft,
                                                                const IntegratedEncoder &iright) {
  leftSensor = std::make_shared<IntegratedEncoder>(ileft);
  rightSensor = std::make_shared<IntegratedEncoder>(iright);
  return *this;
}

ChassisControllerBuilder &
ChassisControllerBuilder::withGains(const IterativePosPIDController::Gains &idistanceGains,
                                    const IterativePosPIDController::Gains &iangleGains) {
  return withGains(idistanceGains, iangleGains, iangleGains);
}

ChassisControllerBuilder &
ChassisControllerBuilder::withGains(const IterativePosPIDController::Gains &idistanceGains,
                                    const IterativePosPIDController::Gains &iangleGains,
                                    const IterativePosPIDController::Gains &iturnGains) {
  hasGains = true;
  distanceGains = idistanceGains;
  angleGains = iangleGains;
  turnGains = iturnGains;
  return *this;
}

ChassisControllerBuilder &
ChassisControllerBuilder::withGearset(const AbstractMotor::GearsetRatioPair &igearset) {
  gearset = igearset;
  return *this;
}

ChassisControllerBuilder &ChassisControllerBuilder::withScales(const ChassisScales &iscales) {
  scales = iscales;
  return *this;
}

std::shared_ptr<ChassisController> ChassisControllerBuilder::build() {
  if (hasGains) {
    // CCPID
    if (isSkidSteer) {
      auto out = std::make_shared<ChassisControllerPID>(
        TimeUtilFactory::create(),
        makeSkidSteerModel(),
        std::make_unique<IterativePosPIDController>(distanceGains, TimeUtilFactory::create()),
        std::make_unique<IterativePosPIDController>(angleGains, TimeUtilFactory::create()),
        std::make_unique<IterativePosPIDController>(turnGains, TimeUtilFactory::create()),
        gearset,
        scales);
      out->startThread();
      return out;
    } else {
      auto out = std::make_shared<ChassisControllerPID>(
        TimeUtilFactory::create(),
        makeXDriveModel(),
        std::make_unique<IterativePosPIDController>(distanceGains, TimeUtilFactory::create()),
        std::make_unique<IterativePosPIDController>(angleGains, TimeUtilFactory::create()),
        std::make_unique<IterativePosPIDController>(turnGains, TimeUtilFactory::create()),
        gearset,
        scales);
      out->startThread();
      return out;
    }
  } else {
    // CCI
    if (isSkidSteer) {
      auto out = std::make_shared<ChassisControllerIntegrated>(
        TimeUtilFactory::create(),
        makeSkidSteerModel(),
        std::make_unique<AsyncPosIntegratedController>(skidSteerMotors.left,
                                                       TimeUtilFactory::create()),
        std::make_unique<AsyncPosIntegratedController>(skidSteerMotors.right,
                                                       TimeUtilFactory::create()),
        gearset,
        scales);
      return out;
    } else {
      auto out = std::make_shared<ChassisControllerIntegrated>(
        TimeUtilFactory::create(),
        makeXDriveModel(),
        std::make_unique<AsyncPosIntegratedController>(skidSteerMotors.left,
                                                       TimeUtilFactory::create()),
        std::make_unique<AsyncPosIntegratedController>(skidSteerMotors.right,
                                                       TimeUtilFactory::create()),
        gearset,
        scales);
      return out;
    }
  }
}

std::shared_ptr<SkidSteerModel> ChassisControllerBuilder::makeSkidSteerModel() {
  return std::make_shared<SkidSteerModel>(
    skidSteerMotors.left, skidSteerMotors.right, toUnderlyingType(gearset.internalGearset));
}

std::shared_ptr<XDriveModel> ChassisControllerBuilder::makeXDriveModel() {
  return std::make_shared<XDriveModel>(xDriveMotors.topLeft,
                                       xDriveMotors.topRight,
                                       xDriveMotors.bottomRight,
                                       xDriveMotors.bottomLeft,
                                       toUnderlyingType(gearset.internalGearset));
}
} // namespace okapi
