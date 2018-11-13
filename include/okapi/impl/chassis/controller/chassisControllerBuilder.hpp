/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#pragma once

#include "okapi/api/chassis/controller/chassisControllerIntegrated.hpp"
#include "okapi/api/chassis/controller/chassisControllerPid.hpp"
#include "okapi/api/chassis/model/skidSteerModel.hpp"
#include "okapi/api/chassis/model/xDriveModel.hpp"
#include "okapi/api/util/mathUtil.hpp"
#include "okapi/impl/device/motor/motor.hpp"
#include "okapi/impl/device/motor/motorGroup.hpp"
#include "okapi/impl/device/rotarysensor/adiEncoder.hpp"
#include "okapi/impl/device/rotarysensor/integratedEncoder.hpp"
#include "okapi/impl/util/timeUtilFactory.hpp"

namespace okapi {
class ChassisControllerBuilder {
  public:
  ChassisControllerBuilder &withMotors(const Motor &ileft, const Motor &iright);

  ChassisControllerBuilder &withMotors(const MotorGroup &ileft, const MotorGroup &iright);

  ChassisControllerBuilder &withMotors(const Motor &itopLeft,
                                       const Motor &itopRight,
                                       const Motor &ibottomRight,
                                       const Motor &ibottomLeft);

  ChassisControllerBuilder &withMotors(const MotorGroup &itopLeft,
                                       const MotorGroup &itopRight,
                                       const MotorGroup &ibottomRight,
                                       const MotorGroup &ibottomLeft);

  ChassisControllerBuilder &withSensors(const ADIEncoder &ileft, const ADIEncoder &iright);

  ChassisControllerBuilder &withSensors(const IntegratedEncoder &ileft,
                                        const IntegratedEncoder &iright);

  ChassisControllerBuilder &withGains(const IterativePosPIDController::Gains &idistanceGains,
                                      const IterativePosPIDController::Gains &iangleGains);

  ChassisControllerBuilder &withGains(const IterativePosPIDController::Gains &idistanceGains,
                                      const IterativePosPIDController::Gains &iangleGains,
                                      const IterativePosPIDController::Gains &iturnGains);

  ChassisControllerBuilder &withGearset(const AbstractMotor::GearsetRatioPair &igearset);

  ChassisControllerBuilder &withScales(const ChassisScales &iscales);

  std::shared_ptr<ChassisController> build();

  private:
  struct SkidSteerMotors {
    std::shared_ptr<AbstractMotor> left;
    std::shared_ptr<AbstractMotor> right;
  };

  struct XDriveMotors {
    std::shared_ptr<AbstractMotor> topLeft;
    std::shared_ptr<AbstractMotor> topRight;
    std::shared_ptr<AbstractMotor> bottomRight;
    std::shared_ptr<AbstractMotor> bottomLeft;
  };

  bool isSkidSteer{true};
  SkidSteerMotors skidSteerMotors;
  XDriveMotors xDriveMotors;

  std::shared_ptr<ContinuousRotarySensor> leftSensor;
  std::shared_ptr<ContinuousRotarySensor> rightSensor;

  bool hasGains{false};
  IterativePosPIDController::Gains distanceGains;
  IterativePosPIDController::Gains angleGains;
  IterativePosPIDController::Gains turnGains;

  AbstractMotor::GearsetRatioPair gearset = AbstractMotor::gearset::red;
  ChassisScales scales = {1, 1};

  std::shared_ptr<SkidSteerModel> makeSkidSteerModel();
  std::shared_ptr<XDriveModel> makeXDriveModel();
};
} // namespace okapi
