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

#define okapi_makeCreateInt(MotorType, methodName)                                                 \
  static auto methodName(const MotorType &ileftMtr,                                                \
                         const MotorType &irightMtr,                                               \
                         AbstractMotor::GearsetRatioPair igearset = AbstractMotor::gearset::red,   \
                         const ChassisScales &iscales = ChassisScales({1, 1}),                     \
                         const TimeUtil &itimeUtil = TimeUtilFactory::create()) {                  \
    return methodName(std::make_shared<MotorType>(ileftMtr),                                       \
                      std::make_shared<MotorType>(irightMtr),                                      \
                      ileftMtr.getEncoder(),                                                       \
                      irightMtr.getEncoder(),                                                      \
                      igearset,                                                                    \
                      iscales,                                                                     \
                      itimeUtil);                                                                  \
  }                                                                                                \
  static auto methodName(const MotorType &itopLeftMtr,                                             \
                         const MotorType &itopRightMtr,                                            \
                         const MotorType &ibottomRightMtr,                                         \
                         const MotorType &ibottomLeftMtr,                                          \
                         AbstractMotor::GearsetRatioPair igearset = AbstractMotor::gearset::red,   \
                         const ChassisScales &iscales = ChassisScales({1, 1}),                     \
                         const TimeUtil &itimeUtil = TimeUtilFactory::create()) {                  \
    return methodName(std::make_shared<MotorType>(itopLeftMtr),                                    \
                      std::make_shared<MotorType>(itopRightMtr),                                   \
                      std::make_shared<MotorType>(ibottomRightMtr),                                \
                      std::make_shared<MotorType>(ibottomLeftMtr),                                 \
                      itopLeftMtr.getEncoder(),                                                    \
                      itopRightMtr.getEncoder(),                                                   \
                      igearset,                                                                    \
                      iscales,                                                                     \
                      itimeUtil);                                                                  \
  }

#define okapi_makeCreatePID(MotorType, methodName)                                                 \
  static auto methodName(const MotorType &ileftMtr,                                                \
                         const MotorType &irightMtr,                                               \
                         const IterativePosPIDController::Gains &idistanceGains,                   \
                         const IterativePosPIDController::Gains &iangleGains,                      \
                         AbstractMotor::GearsetRatioPair igearset = AbstractMotor::gearset::red,   \
                         const ChassisScales &iscales = ChassisScales({1, 1}),                     \
                         const TimeUtil &itimeUtil = TimeUtilFactory::create()) {                  \
    return methodName(                                                                             \
      std::make_shared<MotorType>(ileftMtr),                                                       \
      std::make_shared<MotorType>(irightMtr),                                                      \
      ileftMtr.getEncoder(),                                                                       \
      irightMtr.getEncoder(),                                                                      \
      std::make_unique<IterativePosPIDController>(idistanceGains, TimeUtilFactory::create()),      \
      std::make_unique<IterativePosPIDController>(iangleGains, TimeUtilFactory::create()),         \
      std::make_unique<IterativePosPIDController>(iangleGains, TimeUtilFactory::create()),         \
      igearset,                                                                                    \
      iscales,                                                                                     \
      itimeUtil);                                                                                  \
  }                                                                                                \
  static auto methodName(const MotorType &ileftMtr,                                                \
                         const MotorType &irightMtr,                                               \
                         const IterativePosPIDController::Gains &idistanceGains,                   \
                         const IterativePosPIDController::Gains &iangleGains,                      \
                         const IterativePosPIDController::Gains &iturnGains,                       \
                         AbstractMotor::GearsetRatioPair igearset = AbstractMotor::gearset::red,   \
                         const ChassisScales &iscales = ChassisScales({1, 1}),                     \
                         const TimeUtil &itimeUtil = TimeUtilFactory::create()) {                  \
    return methodName(                                                                             \
      std::make_shared<MotorType>(ileftMtr),                                                       \
      std::make_shared<MotorType>(irightMtr),                                                      \
      ileftMtr.getEncoder(),                                                                       \
      irightMtr.getEncoder(),                                                                      \
      std::make_unique<IterativePosPIDController>(idistanceGains, TimeUtilFactory::create()),      \
      std::make_unique<IterativePosPIDController>(iangleGains, TimeUtilFactory::create()),         \
      std::make_unique<IterativePosPIDController>(iturnGains, TimeUtilFactory::create()),          \
      igearset,                                                                                    \
      iscales,                                                                                     \
      itimeUtil);                                                                                  \
  }                                                                                                \
  static auto methodName(const MotorType &itopLeftMtr,                                             \
                         const MotorType &itopRightMtr,                                            \
                         const MotorType &ibottomRightMtr,                                         \
                         const MotorType &ibottomLeftMtr,                                          \
                         const IterativePosPIDController::Gains &idistanceGains,                   \
                         const IterativePosPIDController::Gains &iangleGains,                      \
                         AbstractMotor::GearsetRatioPair igearset = AbstractMotor::gearset::red,   \
                         const ChassisScales &iscales = ChassisScales({1, 1}),                     \
                         const TimeUtil &itimeUtil = TimeUtilFactory::create()) {                  \
    return methodName(                                                                             \
      std::make_shared<MotorType>(itopLeftMtr),                                                    \
      std::make_shared<MotorType>(itopRightMtr),                                                   \
      std::make_shared<MotorType>(ibottomRightMtr),                                                \
      std::make_shared<MotorType>(ibottomLeftMtr),                                                 \
      itopLeftMtr.getEncoder(),                                                                    \
      itopRightMtr.getEncoder(),                                                                   \
      std::make_unique<IterativePosPIDController>(idistanceGains, TimeUtilFactory::create()),      \
      std::make_unique<IterativePosPIDController>(iangleGains, TimeUtilFactory::create()),         \
      std::make_unique<IterativePosPIDController>(iangleGains, TimeUtilFactory::create()),         \
      igearset,                                                                                    \
      iscales,                                                                                     \
      itimeUtil);                                                                                  \
  }                                                                                                \
  static auto methodName(const MotorType &itopLeftMtr,                                             \
                         const MotorType &itopRightMtr,                                            \
                         const MotorType &ibottomRightMtr,                                         \
                         const MotorType &ibottomLeftMtr,                                          \
                         const IterativePosPIDController::Gains &idistanceGains,                   \
                         const IterativePosPIDController::Gains &iangleGains,                      \
                         const IterativePosPIDController::Gains &iturnGains,                       \
                         AbstractMotor::GearsetRatioPair igearset = AbstractMotor::gearset::red,   \
                         const ChassisScales &iscales = ChassisScales({1, 1}),                     \
                         const TimeUtil &itimeUtil = TimeUtilFactory::create()) {                  \
    return methodName(                                                                             \
      std::make_shared<MotorType>(itopLeftMtr),                                                    \
      std::make_shared<MotorType>(itopRightMtr),                                                   \
      std::make_shared<MotorType>(ibottomRightMtr),                                                \
      std::make_shared<MotorType>(ibottomLeftMtr),                                                 \
      itopLeftMtr.getEncoder(),                                                                    \
      itopRightMtr.getEncoder(),                                                                   \
      std::make_unique<IterativePosPIDController>(idistanceGains, TimeUtilFactory::create()),      \
      std::make_unique<IterativePosPIDController>(iangleGains, TimeUtilFactory::create()),         \
      std::make_unique<IterativePosPIDController>(iturnGains, TimeUtilFactory::create()),          \
      igearset,                                                                                    \
      iscales,                                                                                     \
      itimeUtil);                                                                                  \
  }

#define okapi_makeCreatePIDWithSensor(MotorType, SensorType, methodName)                           \
  static auto methodName(const MotorType &ileftMtr,                                                \
                         const MotorType &irightMtr,                                               \
                         const SensorType &ileftSens,                                              \
                         const SensorType &irightSens,                                             \
                         const IterativePosPIDController::Gains &idistanceGains,                   \
                         const IterativePosPIDController::Gains &iangleGains,                      \
                         AbstractMotor::GearsetRatioPair igearset = AbstractMotor::gearset::red,   \
                         const ChassisScales &iscales = ChassisScales({1, 1}),                     \
                         const TimeUtil &itimeUtil = TimeUtilFactory::create()) {                  \
    return methodName(                                                                             \
      std::make_shared<MotorType>(ileftMtr),                                                       \
      std::make_shared<MotorType>(irightMtr),                                                      \
      std::make_shared<SensorType>(ileftSens),                                                     \
      std::make_shared<SensorType>(irightSens),                                                    \
      std::make_unique<IterativePosPIDController>(idistanceGains, TimeUtilFactory::create()),      \
      std::make_unique<IterativePosPIDController>(iangleGains, TimeUtilFactory::create()),         \
      std::make_unique<IterativePosPIDController>(iangleGains, TimeUtilFactory::create()),         \
      igearset,                                                                                    \
      iscales,                                                                                     \
      itimeUtil);                                                                                  \
  }                                                                                                \
  static auto methodName(const MotorType &ileftMtr,                                                \
                         const MotorType &irightMtr,                                               \
                         const SensorType &ileftSens,                                              \
                         const SensorType &irightSens,                                             \
                         const IterativePosPIDController::Gains &idistanceGains,                   \
                         const IterativePosPIDController::Gains &iangleGains,                      \
                         const IterativePosPIDController::Gains &iturnGains,                       \
                         AbstractMotor::GearsetRatioPair igearset = AbstractMotor::gearset::red,   \
                         const ChassisScales &iscales = ChassisScales({1, 1}),                     \
                         const TimeUtil &itimeUtil = TimeUtilFactory::create()) {                  \
    return methodName(                                                                             \
      std::make_shared<MotorType>(ileftMtr),                                                       \
      std::make_shared<MotorType>(irightMtr),                                                      \
      std::make_shared<SensorType>(ileftSens),                                                     \
      std::make_shared<SensorType>(irightSens),                                                    \
      std::make_unique<IterativePosPIDController>(idistanceGains, TimeUtilFactory::create()),      \
      std::make_unique<IterativePosPIDController>(iangleGains, TimeUtilFactory::create()),         \
      std::make_unique<IterativePosPIDController>(iturnGains, TimeUtilFactory::create()),          \
      igearset,                                                                                    \
      iscales,                                                                                     \
      itimeUtil);                                                                                  \
  }                                                                                                \
  static auto methodName(const MotorType &itopLeftMtr,                                             \
                         const MotorType &itopRightMtr,                                            \
                         const MotorType &ibottomRightMtr,                                         \
                         const MotorType &ibottomLeftMtr,                                          \
                         const SensorType &ileftSens,                                              \
                         const SensorType &irightSens,                                             \
                         const IterativePosPIDController::Gains &idistanceGains,                   \
                         const IterativePosPIDController::Gains &iangleGains,                      \
                         AbstractMotor::GearsetRatioPair igearset = AbstractMotor::gearset::red,   \
                         const ChassisScales &iscales = ChassisScales({1, 1}),                     \
                         const TimeUtil &itimeUtil = TimeUtilFactory::create()) {                  \
    return methodName(                                                                             \
      std::make_shared<MotorType>(itopLeftMtr),                                                    \
      std::make_shared<MotorType>(itopRightMtr),                                                   \
      std::make_shared<MotorType>(ibottomRightMtr),                                                \
      std::make_shared<MotorType>(ibottomLeftMtr),                                                 \
      std::make_shared<SensorType>(ileftSens),                                                     \
      std::make_shared<SensorType>(irightSens),                                                    \
      std::make_unique<IterativePosPIDController>(idistanceGains, TimeUtilFactory::create()),      \
      std::make_unique<IterativePosPIDController>(iangleGains, TimeUtilFactory::create()),         \
      std::make_unique<IterativePosPIDController>(iangleGains, TimeUtilFactory::create()),         \
      igearset,                                                                                    \
      iscales,                                                                                     \
      itimeUtil);                                                                                  \
  }                                                                                                \
  static auto methodName(const MotorType &itopLeftMtr,                                             \
                         const MotorType &itopRightMtr,                                            \
                         const MotorType &ibottomRightMtr,                                         \
                         const MotorType &ibottomLeftMtr,                                          \
                         const SensorType &ileftSens,                                              \
                         const SensorType &irightSens,                                             \
                         const IterativePosPIDController::Gains &idistanceGains,                   \
                         const IterativePosPIDController::Gains &iangleGains,                      \
                         const IterativePosPIDController::Gains &iturnGains,                       \
                         AbstractMotor::GearsetRatioPair igearset = AbstractMotor::gearset::red,   \
                         const ChassisScales &iscales = ChassisScales({1, 1}),                     \
                         const TimeUtil &itimeUtil = TimeUtilFactory::create()) {                  \
    return methodName(                                                                             \
      std::make_shared<MotorType>(itopLeftMtr),                                                    \
      std::make_shared<MotorType>(itopRightMtr),                                                   \
      std::make_shared<MotorType>(ibottomRightMtr),                                                \
      std::make_shared<MotorType>(ibottomLeftMtr),                                                 \
      std::make_shared<SensorType>(ileftSens),                                                     \
      std::make_shared<SensorType>(irightSens),                                                    \
      std::make_unique<IterativePosPIDController>(idistanceGains, TimeUtilFactory::create()),      \
      std::make_unique<IterativePosPIDController>(iangleGains, TimeUtilFactory::create()),         \
      std::make_unique<IterativePosPIDController>(iturnGains, TimeUtilFactory::create()),          \
      igearset,                                                                                    \
      iscales,                                                                                     \
      itimeUtil);                                                                                  \
  }

namespace okapi {
class ChassisControllerFactory {
  public:
  okapi_makeCreateInt(Motor, create);
  okapi_makeCreateInt(MotorGroup, create);

  okapi_makeCreateInt(Motor, createPtr);
  okapi_makeCreateInt(MotorGroup, createPtr);

  okapi_makeCreatePID(Motor, create);
  okapi_makeCreatePID(MotorGroup, create);

  okapi_makeCreatePID(Motor, createPtr);
  okapi_makeCreatePID(MotorGroup, createPtr);

  okapi_makeCreatePIDWithSensor(Motor, IntegratedEncoder, create);
  okapi_makeCreatePIDWithSensor(MotorGroup, IntegratedEncoder, create);
  okapi_makeCreatePIDWithSensor(Motor, ADIEncoder, create);
  okapi_makeCreatePIDWithSensor(MotorGroup, ADIEncoder, create);

  okapi_makeCreatePIDWithSensor(Motor, IntegratedEncoder, createPtr);
  okapi_makeCreatePIDWithSensor(MotorGroup, IntegratedEncoder, createPtr);
  okapi_makeCreatePIDWithSensor(Motor, ADIEncoder, createPtr);
  okapi_makeCreatePIDWithSensor(MotorGroup, ADIEncoder, createPtr);

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
