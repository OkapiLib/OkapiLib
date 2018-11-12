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

#define okapi_makeCreateEndParams                                                                  \
  AbstractMotor::GearsetRatioPair igearset = AbstractMotor::gearset::red,                          \
                                  const ChassisScales &iscales = ChassisScales({1, 1}),            \
                                  const TimeUtil &itimeUtil = TimeUtilFactory::create()

#define okapi_makeCreateEndBody igearset, iscales, itimeUtil

#define okapi_tankParams(MotorType) const MotorType &ileftMtr, const MotorType &irightMtr

#define okapi_tankBody(MotorType)                                                                  \
  std::make_shared<MotorType>(ileftMtr), std::make_shared<MotorType>(irightMtr)

#define okapi_tankSensorBody ileftMtr.getEncoder(), irightMtr.getEncoder()

#define okapi_xdriveParams(MotorType)                                                              \
  const MotorType &itopLeftMtr, const MotorType &itopRightMtr, const MotorType &ibottomRightMtr,   \
    const MotorType &ibottomLeftMtr

#define okapi_xdriveBody(MotorType)                                                                \
  std::make_shared<MotorType>(itopLeftMtr), std::make_shared<MotorType>(itopRightMtr),             \
    std::make_shared<MotorType>(ibottomRightMtr), std::make_shared<MotorType>(ibottomLeftMtr)

#define okapi_xdriveSensorBody itopLeftMtr.getEncoder(), itopRightMtr.getEncoder()

#define okapi_makeCreateIntImpl(driveType, motorType, methodName)                                  \
  static auto methodName(okapi_##driveType##Params(motorType), okapi_makeCreateEndParams) {        \
    return methodName(                                                                             \
      okapi_##driveType##Body(motorType), okapi_##driveType##SensorBody, okapi_makeCreateEndBody); \
  }

#define okapi_makeCreateInt(motorType, methodName)                                                 \
  okapi_makeCreateIntImpl(tank, motorType, methodName);                                            \
  okapi_makeCreateIntImpl(xdrive, motorType, methodName)

#define okapi_pidGains2Params()                                                                    \
  const IterativePosPIDController::Gains &idistanceGains,                                          \
    const IterativePosPIDController::Gains &iangleGains

#define okapi_pidGains2Body()                                                                      \
  std::make_unique<IterativePosPIDController>(idistanceGains, TimeUtilFactory::create()),          \
    std::make_unique<IterativePosPIDController>(iangleGains, TimeUtilFactory::create()),           \
    std::make_unique<IterativePosPIDController>(iangleGains, TimeUtilFactory::create())

#define okapi_pidGains3Params()                                                                    \
  const IterativePosPIDController::Gains &idistanceGains,                                          \
    const IterativePosPIDController::Gains &iangleGains,                                           \
    const IterativePosPIDController::Gains &iturnGains

#define okapi_pidGains3Body()                                                                      \
  std::make_unique<IterativePosPIDController>(idistanceGains, TimeUtilFactory::create()),          \
    std::make_unique<IterativePosPIDController>(iangleGains, TimeUtilFactory::create()),           \
    std::make_unique<IterativePosPIDController>(iturnGains, TimeUtilFactory::create())

#define okapi_makeCreatePidImpl(driveType, pidGainsType, motorType, methodName)                    \
  static auto methodName(okapi_##driveType##Params(motorType),                                     \
                         okapi_##pidGainsType##Params(),                                           \
                         okapi_makeCreateEndParams) {                                              \
    return methodName(okapi_##driveType##Body(motorType),                                          \
                      okapi_##driveType##SensorBody,                                               \
                      okapi_##pidGainsType##Body(),                                                \
                      okapi_makeCreateEndBody);                                                    \
  }

#define okapi_makeCreatePid(motorType, methodName)                                                 \
  okapi_makeCreatePidImpl(tank, pidGains2, motorType, methodName);                                 \
  okapi_makeCreatePidImpl(tank, pidGains3, motorType, methodName);                                 \
  okapi_makeCreatePidImpl(xdrive, pidGains2, motorType, methodName);                               \
  okapi_makeCreatePidImpl(xdrive, pidGains3, motorType, methodName)

#define okapi_sensorParams(SensorType) const SensorType &ileftSens, const SensorType &irightSens

#define okapi_sensorBody(SensorType)                                                               \
  std::make_shared<SensorType>(ileftSens), std::make_shared<SensorType>(irightSens)

#define okapi_makeCreatePidWithSensorImpl(                                                         \
  driveType, sensorType, pidGainsType, motorType, methodName)                                      \
  static auto methodName(okapi_##driveType##Params(motorType),                                     \
                         okapi_sensorParams(sensorType),                                           \
                         okapi_##pidGainsType##Params(),                                           \
                         okapi_makeCreateEndParams) {                                              \
    return methodName(okapi_##driveType##Body(motorType),                                          \
                      okapi_sensorBody(sensorType),                                                \
                      okapi_##pidGainsType##Body(),                                                \
                      okapi_makeCreateEndBody);                                                    \
  }

#define okapi_makeCreatePidWithSensorImpl2(pidGainsType, motorType, methodName)                    \
  okapi_makeCreatePidWithSensorImpl(tank, IntegratedEncoder, pidGainsType, motorType, methodName); \
  okapi_makeCreatePidWithSensorImpl(tank, ADIEncoder, pidGainsType, motorType, methodName);        \
  okapi_makeCreatePidWithSensorImpl(                                                               \
    xdrive, IntegratedEncoder, pidGainsType, motorType, methodName);                               \
  okapi_makeCreatePidWithSensorImpl(xdrive, ADIEncoder, pidGainsType, motorType, methodName)

#define okapi_makeCreatePidWithSensor(motorType, methodName)                                       \
  okapi_makeCreatePidWithSensorImpl2(pidGains2, motorType, methodName);                            \
  okapi_makeCreatePidWithSensorImpl2(pidGains3, motorType, methodName)

namespace okapi {
class ChassisControllerFactory {
  public:
  okapi_makeCreateInt(Motor, create);
  okapi_makeCreateInt(MotorGroup, create);
  okapi_makeCreateInt(Motor, createPtr);
  okapi_makeCreateInt(MotorGroup, createPtr);

  okapi_makeCreatePid(Motor, create);
  okapi_makeCreatePid(MotorGroup, create);
  okapi_makeCreatePid(Motor, createPtr);
  okapi_makeCreatePid(MotorGroup, createPtr);

  okapi_makeCreatePidWithSensor(Motor, create);
  okapi_makeCreatePidWithSensor(MotorGroup, create);
  okapi_makeCreatePidWithSensor(Motor, createPtr);
  okapi_makeCreatePidWithSensor(MotorGroup, createPtr);

  static ChassisControllerIntegrated
  create(const std::shared_ptr<AbstractMotor> &ileftMtr,
         const std::shared_ptr<AbstractMotor> &irightMtr,
         const std::shared_ptr<ContinuousRotarySensor> &ileftSensor,
         const std::shared_ptr<ContinuousRotarySensor> &irightSensor,
         AbstractMotor::GearsetRatioPair igearset,
         const ChassisScales &iscales,
         const TimeUtil &itimeUtil = TimeUtilFactory::create()) {
    return ChassisControllerIntegrated(
      itimeUtil,
      std::make_shared<SkidSteerModel>(
        ileftMtr, irightMtr, ileftSensor, irightSensor, toUnderlyingType(igearset.internalGearset)),
      std::make_unique<AsyncPosIntegratedController>(ileftMtr, TimeUtilFactory::create()),
      std::make_unique<AsyncPosIntegratedController>(irightMtr, TimeUtilFactory::create()),
      igearset,
      iscales);
  }

  static std::shared_ptr<ChassisControllerIntegrated>
  createPtr(const std::shared_ptr<AbstractMotor> &ileftMtr,
            const std::shared_ptr<AbstractMotor> &irightMtr,
            const std::shared_ptr<ContinuousRotarySensor> &ileftSensor,
            const std::shared_ptr<ContinuousRotarySensor> &irightSensor,
            AbstractMotor::GearsetRatioPair igearset,
            const ChassisScales &iscales,
            const TimeUtil &itimeUtil = TimeUtilFactory::create()) {
    return std::make_shared<ChassisControllerIntegrated>(
      itimeUtil,
      std::make_shared<SkidSteerModel>(
        ileftMtr, irightMtr, ileftSensor, irightSensor, toUnderlyingType(igearset.internalGearset)),
      std::make_unique<AsyncPosIntegratedController>(ileftMtr, TimeUtilFactory::create()),
      std::make_unique<AsyncPosIntegratedController>(irightMtr, TimeUtilFactory::create()),
      igearset,
      iscales);
  }

  static ChassisControllerIntegrated
  create(const std::shared_ptr<AbstractMotor> &itopLeftMtr,
         const std::shared_ptr<AbstractMotor> &itopRightMtr,
         const std::shared_ptr<AbstractMotor> &ibottomRightMtr,
         const std::shared_ptr<AbstractMotor> &ibottomLeftMtr,
         const std::shared_ptr<ContinuousRotarySensor> &ileftSensor,
         const std::shared_ptr<ContinuousRotarySensor> &irightSensor,
         AbstractMotor::GearsetRatioPair igearset,
         const ChassisScales &iscales,
         const TimeUtil &itimeUtil = TimeUtilFactory::create()) {
    return ChassisControllerIntegrated(
      itimeUtil,
      std::make_shared<XDriveModel>(itopLeftMtr,
                                    itopRightMtr,
                                    ibottomRightMtr,
                                    ibottomLeftMtr,
                                    ileftSensor,
                                    irightSensor,
                                    toUnderlyingType(igearset.internalGearset)),
      std::make_unique<AsyncPosIntegratedController>(itopLeftMtr, TimeUtilFactory::create()),
      std::make_unique<AsyncPosIntegratedController>(itopRightMtr, TimeUtilFactory::create()),
      igearset,
      iscales);
  }

  static std::shared_ptr<ChassisControllerIntegrated>
  createPtr(const std::shared_ptr<AbstractMotor> &itopLeftMtr,
            const std::shared_ptr<AbstractMotor> &itopRightMtr,
            const std::shared_ptr<AbstractMotor> &ibottomRightMtr,
            const std::shared_ptr<AbstractMotor> &ibottomLeftMtr,
            const std::shared_ptr<ContinuousRotarySensor> &ileftSensor,
            const std::shared_ptr<ContinuousRotarySensor> &irightSensor,
            AbstractMotor::GearsetRatioPair igearset,
            const ChassisScales &iscales,
            const TimeUtil &itimeUtil = TimeUtilFactory::create()) {
    return std::make_shared<ChassisControllerIntegrated>(
      itimeUtil,
      std::make_shared<XDriveModel>(itopLeftMtr,
                                    itopRightMtr,
                                    ibottomRightMtr,
                                    ibottomLeftMtr,
                                    ileftSensor,
                                    irightSensor,
                                    toUnderlyingType(igearset.internalGearset)),
      std::make_unique<AsyncPosIntegratedController>(itopLeftMtr, TimeUtilFactory::create()),
      std::make_unique<AsyncPosIntegratedController>(itopRightMtr, TimeUtilFactory::create()),
      igearset,
      iscales);
  }

  static ChassisControllerPID create(const std::shared_ptr<AbstractMotor> &ileftMtr,
                                     const std::shared_ptr<AbstractMotor> &irightMtr,
                                     const std::shared_ptr<ContinuousRotarySensor> &ileftSensor,
                                     const std::shared_ptr<ContinuousRotarySensor> &irightSensor,
                                     std::unique_ptr<IterativePosPIDController> idistanceController,
                                     std::unique_ptr<IterativePosPIDController> iangleController,
                                     std::unique_ptr<IterativePosPIDController> iturnController,
                                     AbstractMotor::GearsetRatioPair igearset,
                                     const ChassisScales &iscales,
                                     const TimeUtil &itimeUtil = TimeUtilFactory::create()) {
    ChassisControllerPID out(
      itimeUtil,
      std::make_shared<SkidSteerModel>(
        ileftMtr, irightMtr, ileftSensor, irightSensor, toUnderlyingType(igearset.internalGearset)),
      std::move(idistanceController),
      std::move(iangleController),
      std::move(iturnController),
      igearset,
      iscales);
    out.startThread();
    return out;
  }

  static std::shared_ptr<ChassisControllerPID>
  createPtr(const std::shared_ptr<AbstractMotor> &ileftMtr,
            const std::shared_ptr<AbstractMotor> &irightMtr,
            const std::shared_ptr<ContinuousRotarySensor> &ileftSensor,
            const std::shared_ptr<ContinuousRotarySensor> &irightSensor,
            std::unique_ptr<IterativePosPIDController> idistanceController,
            std::unique_ptr<IterativePosPIDController> iangleController,
            std::unique_ptr<IterativePosPIDController> iturnController,
            AbstractMotor::GearsetRatioPair igearset,
            const ChassisScales &iscales,
            const TimeUtil &itimeUtil = TimeUtilFactory::create()) {
    auto out = std::make_shared<ChassisControllerPID>(
      itimeUtil,
      std::make_shared<SkidSteerModel>(
        ileftMtr, irightMtr, ileftSensor, irightSensor, toUnderlyingType(igearset.internalGearset)),
      std::move(idistanceController),
      std::move(iangleController),
      std::move(iturnController),
      igearset,
      iscales);
    out->startThread();
    return out;
  }

  static ChassisControllerPID create(const std::shared_ptr<AbstractMotor> &itopLeftMtr,
                                     const std::shared_ptr<AbstractMotor> &itopRightMtr,
                                     const std::shared_ptr<AbstractMotor> &ibottomRightMtr,
                                     const std::shared_ptr<AbstractMotor> &ibottomLeftMtr,
                                     const std::shared_ptr<ContinuousRotarySensor> &ileftSensor,
                                     const std::shared_ptr<ContinuousRotarySensor> &irightSensor,
                                     std::unique_ptr<IterativePosPIDController> idistanceController,
                                     std::unique_ptr<IterativePosPIDController> iangleController,
                                     std::unique_ptr<IterativePosPIDController> iturnController,
                                     AbstractMotor::GearsetRatioPair igearset,
                                     const ChassisScales &iscales,
                                     const TimeUtil &itimeUtil = TimeUtilFactory::create()) {
    ChassisControllerPID out(
      itimeUtil,
      std::make_shared<XDriveModel>(itopLeftMtr,
                                    itopRightMtr,
                                    ibottomRightMtr,
                                    ibottomLeftMtr,
                                    ileftSensor,
                                    irightSensor,
                                    toUnderlyingType(igearset.internalGearset)),
      std::move(idistanceController),
      std::move(iangleController),
      std::move(iturnController),
      igearset,
      iscales);
    out.startThread();
    return out;
  }

  static std::shared_ptr<ChassisControllerPID>
  createPtr(const std::shared_ptr<AbstractMotor> &itopLeftMtr,
            const std::shared_ptr<AbstractMotor> &itopRightMtr,
            const std::shared_ptr<AbstractMotor> &ibottomRightMtr,
            const std::shared_ptr<AbstractMotor> &ibottomLeftMtr,
            const std::shared_ptr<ContinuousRotarySensor> &ileftSensor,
            const std::shared_ptr<ContinuousRotarySensor> &irightSensor,
            std::unique_ptr<IterativePosPIDController> idistanceController,
            std::unique_ptr<IterativePosPIDController> iangleController,
            std::unique_ptr<IterativePosPIDController> iturnController,
            AbstractMotor::GearsetRatioPair igearset,
            const ChassisScales &iscales,
            const TimeUtil &itimeUtil = TimeUtilFactory::create()) {
    auto out = std::make_shared<ChassisControllerPID>(
      itimeUtil,
      std::make_shared<XDriveModel>(itopLeftMtr,
                                    itopRightMtr,
                                    ibottomRightMtr,
                                    ibottomLeftMtr,
                                    ileftSensor,
                                    irightSensor,
                                    toUnderlyingType(igearset.internalGearset)),
      std::move(idistanceController),
      std::move(iangleController),
      std::move(iturnController),
      igearset,
      iscales);
    out->startThread();
    return out;
  }
};
} // namespace okapi
