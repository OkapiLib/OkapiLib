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

#define okapi_tankParams(MotorType) MotorType ileftMtr, MotorType irightMtr

#define okapi_tankBody(MotorType)                                                                  \
  std::make_shared<MotorType>(ileftMtr), std::make_shared<MotorType>(irightMtr)

#define okapi_tankSensorBody ileftMtr.getEncoder(), irightMtr.getEncoder()

#define okapi_xdriveParams(MotorType)                                                              \
  MotorType itopLeftMtr, MotorType itopRightMtr, MotorType ibottomRightMtr, MotorType ibottomLeftMtr

#define okapi_xdriveBody(MotorType)                                                                \
  std::make_shared<MotorType>(itopLeftMtr), std::make_shared<MotorType>(itopRightMtr),             \
    std::make_shared<MotorType>(ibottomRightMtr), std::make_shared<MotorType>(ibottomLeftMtr)

#define okapi_xdriveSensorBody itopLeftMtr.getEncoder(), itopRightMtr.getEncoder()

#define okapi_makeCreateIntImpl(driveType, motorType, methodName)                                  \
  static auto methodName(okapi_##driveType##Params(motorType), okapi_makeCreateEndParams) {        \
    return methodName(                                                                             \
      okapi_##driveType##Body(motorType), okapi_##driveType##SensorBody, okapi_makeCreateEndBody); \
  }

#define okapi_makeCreateIntImpl2(motorType, methodName)                                            \
  okapi_makeCreateIntImpl(tank, motorType, methodName);                                            \
  okapi_makeCreateIntImpl(xdrive, motorType, methodName)

#define okapi_makeCreateInt(motorType)                                                             \
  okapi_makeCreateIntImpl2(motorType, create);                                                     \
  okapi_makeCreateIntImpl2(motorType, createPtr)

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

#define okapi_makeCreatePidImpl2(pidGainsType, motorType, methodName)                              \
  okapi_makeCreatePidImpl(tank, pidGainsType, motorType, methodName);                              \
  okapi_makeCreatePidImpl(xdrive, pidGainsType, motorType, methodName)

#define okapi_makeCreatePidImpl3(motorType, methodName)                                            \
  okapi_makeCreatePidImpl2(pidGains2, motorType, methodName);                                      \
  okapi_makeCreatePidImpl2(pidGains3, motorType, methodName)

#define okapi_makeCreatePid(motorType)                                                             \
  okapi_makeCreatePidImpl3(motorType, create);                                                     \
  okapi_makeCreatePidImpl3(motorType, createPtr)

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

#define okapi_makeCreatePidWithSensorImpl2(sensorType, pidGainsType, motorType, methodName)        \
  okapi_makeCreatePidWithSensorImpl(tank, sensorType, pidGainsType, motorType, methodName);        \
  okapi_makeCreatePidWithSensorImpl(xdrive, sensorType, pidGainsType, motorType, methodName)

#define okapi_makeCreatePidWithSensorImpl3(pidGainsType, motorType, methodName)                    \
  okapi_makeCreatePidWithSensorImpl2(IntegratedEncoder, pidGainsType, motorType, methodName);      \
  okapi_makeCreatePidWithSensorImpl2(ADIEncoder, pidGainsType, motorType, methodName)

#define okapi_makeCreatePidWithSensorImpl4(motorType, methodName)                                  \
  okapi_makeCreatePidWithSensorImpl3(pidGains2, motorType, methodName);                            \
  okapi_makeCreatePidWithSensorImpl3(pidGains3, motorType, methodName)

#define okapi_makeCreatePidWithSensor(motorType)                                                   \
  okapi_makeCreatePidWithSensorImpl4(motorType, create);                                           \
  okapi_makeCreatePidWithSensorImpl4(motorType, createPtr)

#define okapi_makeCreateAll(motorType)                                                             \
  okapi_makeCreateInt(motorType);                                                                  \
  okapi_makeCreatePid(motorType);                                                                  \
  okapi_makeCreatePidWithSensor(motorType)

#define okapi_baseTankParams                                                                       \
  const std::shared_ptr<AbstractMotor> &ileftMtr, const std::shared_ptr<AbstractMotor> &irightMtr

#define okapi_baseTankBody ileftMtr, irightMtr

#define okapi_baseTankControllerBody                                                               \
  std::make_unique<AsyncPosIntegratedController>(ileftMtr, TimeUtilFactory::create()),             \
    std::make_unique<AsyncPosIntegratedController>(irightMtr, TimeUtilFactory::create())

#define okapi_baseXdriveParams                                                                     \
  const std::shared_ptr<AbstractMotor> &itopLeftMtr,                                               \
    const std::shared_ptr<AbstractMotor> &itopRightMtr,                                            \
    const std::shared_ptr<AbstractMotor> &ibottomRightMtr,                                         \
    const std::shared_ptr<AbstractMotor> &ibottomLeftMtr

#define okapi_baseXdriveBody itopLeftMtr, itopRightMtr, ibottomRightMtr, ibottomLeftMtr

#define okapi_baseXdriveControllerBody                                                             \
  std::make_unique<AsyncPosIntegratedController>(itopLeftMtr, TimeUtilFactory::create()),          \
    std::make_unique<AsyncPosIntegratedController>(itopRightMtr, TimeUtilFactory::create())

#define okapi_baseSensorParams                                                                     \
  const std::shared_ptr<ContinuousRotarySensor> &ileftSensor,                                      \
    const std::shared_ptr<ContinuousRotarySensor> &irightSensor

#define okapi_baseSensorBody ileftSensor, irightSensor

#define okapi_makeCreateBaseIntImpl(                                                               \
  returnType, allocateExpr, methodName, modelType, fullModelTypeName)                              \
  static returnType methodName(                                                                    \
    okapi_##modelType##Params, okapi_baseSensorParams, okapi_makeCreateEndParams) {                \
    return allocateExpr(                                                                           \
      itimeUtil,                                                                                   \
      std::make_shared<fullModelTypeName>(okapi_##modelType##Body,                                 \
                                          okapi_baseSensorBody,                                    \
                                          toUnderlyingType(igearset.internalGearset)),             \
      okapi_##modelType##ControllerBody,                                                           \
      igearset,                                                                                    \
      iscales);                                                                                    \
  }

#define okapi_makeCreateBaseInt(returnType, allocateExpr, methodName)                              \
  okapi_makeCreateBaseIntImpl(returnType, allocateExpr, methodName, baseTank, SkidSteerModel);     \
  okapi_makeCreateBaseIntImpl(returnType, allocateExpr, methodName, baseXdrive, XDriveModel)

#define okapi_createAllocExpr ChassisControllerPID out

#define okapi_createEndExpr                                                                        \
  out.startThread();                                                                               \
  return out

#define okapi_createPtrAllocExpr auto out = std::make_shared<ChassisControllerPID>

#define okapi_createPtrEndExpr                                                                     \
  out->startThread();                                                                              \
  return out

#define okapi_makeCreateBasePidImpl(returnType, methodName, modelType, fullModelTypeName)          \
  static returnType methodName(okapi_##modelType##Params,                                          \
                               okapi_baseSensorParams,                                             \
                               std::unique_ptr<IterativePosPIDController> idistanceController,     \
                               std::unique_ptr<IterativePosPIDController> iangleController,        \
                               std::unique_ptr<IterativePosPIDController> iturnController,         \
                               okapi_makeCreateEndParams) {                                        \
    okapi_##methodName##AllocExpr(                                                                 \
      itimeUtil,                                                                                   \
      std::make_shared<fullModelTypeName>(okapi_##modelType##Body,                                 \
                                          okapi_baseSensorBody,                                    \
                                          toUnderlyingType(igearset.internalGearset)),             \
      std::move(idistanceController),                                                              \
      std::move(iangleController),                                                                 \
      std::move(iturnController),                                                                  \
      igearset,                                                                                    \
      iscales);                                                                                    \
    okapi_##methodName##EndExpr;                                                                   \
  }

#define okapi_makeCreateBasePid(returnType, methodName)                                            \
  okapi_makeCreateBasePidImpl(returnType, methodName, baseTank, SkidSteerModel);                   \
  okapi_makeCreateBasePidImpl(returnType, methodName, baseXdrive, XDriveModel)

namespace okapi {
class ChassisControllerFactory {
  public:
  okapi_makeCreateAll(Motor);
  okapi_makeCreateAll(MotorGroup);

  okapi_makeCreateBaseInt(ChassisControllerIntegrated, ChassisControllerIntegrated, create);
  okapi_makeCreateBaseInt(std::shared_ptr<ChassisControllerIntegrated>,
                          std::make_shared<ChassisControllerIntegrated>,
                          createPtr);

  okapi_makeCreateBasePid(ChassisControllerPID, create);
  okapi_makeCreateBasePid(std::shared_ptr<ChassisControllerPID>, createPtr);
};
} // namespace okapi
