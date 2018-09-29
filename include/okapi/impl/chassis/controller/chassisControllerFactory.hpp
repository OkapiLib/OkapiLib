/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_CHASSISCONTROLLERFACTORY_HPP_
#define _OKAPI_CHASSISCONTROLLERFACTORY_HPP_

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

#define makeCreateInt(MotorType, methodName)                                                       \
  static auto methodName(const MotorType &ileftMtr,                                                \
                         const MotorType &irightMtr,                                               \
                         AbstractMotor::GearsetRatioPair igearset = AbstractMotor::gearset::red,   \
                         const ChassisScales &iscales = ChassisScales({1, 1})) {                   \
    return methodName(std::make_shared<MotorType>(ileftMtr),                                       \
                      std::make_shared<MotorType>(irightMtr),                                      \
                      ileftMtr.getEncoder(),                                                       \
                      irightMtr.getEncoder(),                                                      \
                      igearset,                                                                    \
                      iscales);                                                                    \
  }                                                                                                \
  static auto methodName(const MotorType &itopLeftMtr,                                             \
                         const MotorType &itopRightMtr,                                            \
                         const MotorType &ibottomRightMtr,                                         \
                         const MotorType &ibottomLeftMtr,                                          \
                         AbstractMotor::GearsetRatioPair igearset = AbstractMotor::gearset::red,   \
                         const ChassisScales &iscales = ChassisScales({1, 1})) {                   \
    return methodName(std::make_shared<MotorType>(itopLeftMtr),                                    \
                      std::make_shared<MotorType>(itopRightMtr),                                   \
                      std::make_shared<MotorType>(ibottomRightMtr),                                \
                      std::make_shared<MotorType>(ibottomLeftMtr),                                 \
                      itopLeftMtr.getEncoder(),                                                    \
                      itopRightMtr.getEncoder(),                                                   \
                      igearset,                                                                    \
                      iscales);                                                                    \
  }

#define makeCreatePID(MotorType, methodName)                                                       \
  static auto methodName(const MotorType &ileftMtr,                                                \
                         const MotorType &irightMtr,                                               \
                         const IterativePosPIDController::Gains &idistanceGains,                   \
                         const IterativePosPIDController::Gains &iangleGains,                      \
                         AbstractMotor::GearsetRatioPair igearset = AbstractMotor::gearset::red,   \
                         const ChassisScales &iscales = ChassisScales({1, 1})) {                   \
    return methodName(                                                                             \
      std::make_shared<MotorType>(ileftMtr),                                                       \
      std::make_shared<MotorType>(irightMtr),                                                      \
      ileftMtr.getEncoder(),                                                                       \
      irightMtr.getEncoder(),                                                                      \
      std::make_unique<IterativePosPIDController>(idistanceGains, TimeUtilFactory::create()),      \
      std::make_unique<IterativePosPIDController>(iangleGains, TimeUtilFactory::create()),         \
      std::make_unique<IterativePosPIDController>(iangleGains, TimeUtilFactory::create()),         \
      igearset,                                                                                    \
      iscales);                                                                                    \
  }                                                                                                \
  static auto methodName(const MotorType &ileftMtr,                                                \
                         const MotorType &irightMtr,                                               \
                         const IterativePosPIDController::Gains &idistanceGains,                   \
                         const IterativePosPIDController::Gains &iangleGains,                      \
                         const IterativePosPIDController::Gains &iturnGains,                       \
                         AbstractMotor::GearsetRatioPair igearset = AbstractMotor::gearset::red,   \
                         const ChassisScales &iscales = ChassisScales({1, 1})) {                   \
    return methodName(                                                                             \
      std::make_shared<MotorType>(ileftMtr),                                                       \
      std::make_shared<MotorType>(irightMtr),                                                      \
      ileftMtr.getEncoder(),                                                                       \
      irightMtr.getEncoder(),                                                                      \
      std::make_unique<IterativePosPIDController>(idistanceGains, TimeUtilFactory::create()),      \
      std::make_unique<IterativePosPIDController>(iangleGains, TimeUtilFactory::create()),         \
      std::make_unique<IterativePosPIDController>(iturnGains, TimeUtilFactory::create()),          \
      igearset,                                                                                    \
      iscales);                                                                                    \
  }                                                                                                \
  static auto methodName(const MotorType &itopLeftMtr,                                             \
                         const MotorType &itopRightMtr,                                            \
                         const MotorType &ibottomRightMtr,                                         \
                         const MotorType &ibottomLeftMtr,                                          \
                         const IterativePosPIDController::Gains &idistanceGains,                   \
                         const IterativePosPIDController::Gains &iangleGains,                      \
                         AbstractMotor::GearsetRatioPair igearset = AbstractMotor::gearset::red,   \
                         const ChassisScales &iscales = ChassisScales({1, 1})) {                   \
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
      iscales);                                                                                    \
  }                                                                                                \
  static auto methodName(const MotorType &itopLeftMtr,                                             \
                         const MotorType &itopRightMtr,                                            \
                         const MotorType &ibottomRightMtr,                                         \
                         const MotorType &ibottomLeftMtr,                                          \
                         const IterativePosPIDController::Gains &idistanceGains,                   \
                         const IterativePosPIDController::Gains &iangleGains,                      \
                         const IterativePosPIDController::Gains &iturnGains,                       \
                         AbstractMotor::GearsetRatioPair igearset = AbstractMotor::gearset::red,   \
                         const ChassisScales &iscales = ChassisScales({1, 1})) {                   \
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
      iscales);                                                                                    \
  }

#define makeCreatePIDWithSensor(MotorType, SensorType, methodName)                                 \
  static auto methodName(const MotorType &ileftMtr,                                                \
                         const MotorType &irightMtr,                                               \
                         const SensorType &ileftSens,                                              \
                         const SensorType &irightSens,                                             \
                         const IterativePosPIDController::Gains &idistanceGains,                   \
                         const IterativePosPIDController::Gains &iangleGains,                      \
                         AbstractMotor::GearsetRatioPair igearset = AbstractMotor::gearset::red,   \
                         const ChassisScales &iscales = ChassisScales({1, 1})) {                   \
    return methodName(                                                                             \
      std::make_shared<MotorType>(ileftMtr),                                                       \
      std::make_shared<MotorType>(irightMtr),                                                      \
      std::make_shared<SensorType>(ileftSens),                                                     \
      std::make_shared<SensorType>(irightSens),                                                    \
      std::make_unique<IterativePosPIDController>(idistanceGains, TimeUtilFactory::create()),      \
      std::make_unique<IterativePosPIDController>(iangleGains, TimeUtilFactory::create()),         \
      std::make_unique<IterativePosPIDController>(iangleGains, TimeUtilFactory::create()),         \
      igearset,                                                                                    \
      iscales);                                                                                    \
  }                                                                                                \
  static auto methodName(const MotorType &ileftMtr,                                                \
                         const MotorType &irightMtr,                                               \
                         const SensorType &ileftSens,                                              \
                         const SensorType &irightSens,                                             \
                         const IterativePosPIDController::Gains &idistanceGains,                   \
                         const IterativePosPIDController::Gains &iangleGains,                      \
                         const IterativePosPIDController::Gains &iturnGains,                       \
                         AbstractMotor::GearsetRatioPair igearset = AbstractMotor::gearset::red,   \
                         const ChassisScales &iscales = ChassisScales({1, 1})) {                   \
    return methodName(                                                                             \
      std::make_shared<MotorType>(ileftMtr),                                                       \
      std::make_shared<MotorType>(irightMtr),                                                      \
      std::make_shared<SensorType>(ileftSens),                                                     \
      std::make_shared<SensorType>(irightSens),                                                    \
      std::make_unique<IterativePosPIDController>(idistanceGains, TimeUtilFactory::create()),      \
      std::make_unique<IterativePosPIDController>(iangleGains, TimeUtilFactory::create()),         \
      std::make_unique<IterativePosPIDController>(iturnGains, TimeUtilFactory::create()),          \
      igearset,                                                                                    \
      iscales);                                                                                    \
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
                         const ChassisScales &iscales = ChassisScales({1, 1})) {                   \
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
      iscales);                                                                                    \
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
                         const ChassisScales &iscales = ChassisScales({1, 1})) {                   \
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
      iscales);                                                                                    \
  }

namespace okapi {
class ChassisControllerFactory {
  public:
  makeCreateInt(Motor, create);
  makeCreateInt(MotorGroup, create);

  makeCreateInt(Motor, createPtr);
  makeCreateInt(MotorGroup, createPtr);

  makeCreatePID(Motor, create);
  makeCreatePID(MotorGroup, create);

  makeCreatePID(Motor, createPtr);
  makeCreatePID(MotorGroup, createPtr);

  makeCreatePIDWithSensor(Motor, IntegratedEncoder, create);
  makeCreatePIDWithSensor(MotorGroup, IntegratedEncoder, create);
  makeCreatePIDWithSensor(Motor, ADIEncoder, create);
  makeCreatePIDWithSensor(MotorGroup, ADIEncoder, create);

  makeCreatePIDWithSensor(Motor, IntegratedEncoder, createPtr);
  makeCreatePIDWithSensor(MotorGroup, IntegratedEncoder, createPtr);
  makeCreatePIDWithSensor(Motor, ADIEncoder, createPtr);
  makeCreatePIDWithSensor(MotorGroup, ADIEncoder, createPtr);

  static ChassisControllerIntegrated create(std::shared_ptr<AbstractMotor> ileftMtr,
                                            std::shared_ptr<AbstractMotor> irightMtr,
                                            std::shared_ptr<ContinuousRotarySensor> ileftSensor,
                                            std::shared_ptr<ContinuousRotarySensor> irightSensor,
                                            AbstractMotor::GearsetRatioPair igearset,
                                            const ChassisScales &iscales) {
    return ChassisControllerIntegrated(
      TimeUtilFactory::create(),
      std::make_shared<SkidSteerModel>(
        ileftMtr, irightMtr, ileftSensor, irightSensor, toUnderlyingType(igearset.internalGearset)),
      std::make_unique<AsyncPosIntegratedController>(ileftMtr, TimeUtilFactory::create()),
      std::make_unique<AsyncPosIntegratedController>(irightMtr, TimeUtilFactory::create()),
      igearset,
      iscales);
  }

  static std::shared_ptr<ChassisControllerIntegrated>
  createPtr(std::shared_ptr<AbstractMotor> ileftMtr,
            std::shared_ptr<AbstractMotor> irightMtr,
            std::shared_ptr<ContinuousRotarySensor> ileftSensor,
            std::shared_ptr<ContinuousRotarySensor> irightSensor,
            AbstractMotor::GearsetRatioPair igearset,
            const ChassisScales &iscales) {
    return std::make_shared<ChassisControllerIntegrated>(
      TimeUtilFactory::create(),
      std::make_shared<SkidSteerModel>(
        ileftMtr, irightMtr, ileftSensor, irightSensor, toUnderlyingType(igearset.internalGearset)),
      std::make_unique<AsyncPosIntegratedController>(ileftMtr, TimeUtilFactory::create()),
      std::make_unique<AsyncPosIntegratedController>(irightMtr, TimeUtilFactory::create()),
      igearset,
      iscales);
  }

  static ChassisControllerIntegrated create(std::shared_ptr<AbstractMotor> itopLeftMtr,
                                            std::shared_ptr<AbstractMotor> itopRightMtr,
                                            std::shared_ptr<AbstractMotor> ibottomRightMtr,
                                            std::shared_ptr<AbstractMotor> ibottomLeftMtr,
                                            std::shared_ptr<ContinuousRotarySensor> ileftSensor,
                                            std::shared_ptr<ContinuousRotarySensor> irightSensor,
                                            AbstractMotor::GearsetRatioPair igearset,
                                            const ChassisScales &iscales) {
    return ChassisControllerIntegrated(
      TimeUtilFactory::create(),
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
  createPtr(std::shared_ptr<AbstractMotor> itopLeftMtr,
            std::shared_ptr<AbstractMotor> itopRightMtr,
            std::shared_ptr<AbstractMotor> ibottomRightMtr,
            std::shared_ptr<AbstractMotor> ibottomLeftMtr,
            std::shared_ptr<ContinuousRotarySensor> ileftSensor,
            std::shared_ptr<ContinuousRotarySensor> irightSensor,
            AbstractMotor::GearsetRatioPair igearset,
            const ChassisScales &iscales) {
    return std::make_shared<ChassisControllerIntegrated>(
      TimeUtilFactory::create(),
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

  static ChassisControllerPID create(std::shared_ptr<AbstractMotor> ileftMtr,
                                     std::shared_ptr<AbstractMotor> irightMtr,
                                     std::shared_ptr<ContinuousRotarySensor> ileftSensor,
                                     std::shared_ptr<ContinuousRotarySensor> irightSensor,
                                     std::unique_ptr<IterativePosPIDController> idistanceController,
                                     std::unique_ptr<IterativePosPIDController> iangleController,
                                     std::unique_ptr<IterativePosPIDController> iturnController,
                                     AbstractMotor::GearsetRatioPair igearset,
                                     const ChassisScales &iscales) {
    ChassisControllerPID out(
      TimeUtilFactory::create(),
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
  createPtr(std::shared_ptr<AbstractMotor> ileftMtr,
            std::shared_ptr<AbstractMotor> irightMtr,
            std::shared_ptr<ContinuousRotarySensor> ileftSensor,
            std::shared_ptr<ContinuousRotarySensor> irightSensor,
            std::unique_ptr<IterativePosPIDController> idistanceController,
            std::unique_ptr<IterativePosPIDController> iangleController,
            std::unique_ptr<IterativePosPIDController> iturnController,
            AbstractMotor::GearsetRatioPair igearset,
            const ChassisScales &iscales) {
    auto out = std::make_shared<ChassisControllerPID>(
      TimeUtilFactory::create(),
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

  static ChassisControllerPID create(std::shared_ptr<AbstractMotor> itopLeftMtr,
                                     std::shared_ptr<AbstractMotor> itopRightMtr,
                                     std::shared_ptr<AbstractMotor> ibottomRightMtr,
                                     std::shared_ptr<AbstractMotor> ibottomLeftMtr,
                                     std::shared_ptr<ContinuousRotarySensor> ileftSensor,
                                     std::shared_ptr<ContinuousRotarySensor> irightSensor,
                                     std::unique_ptr<IterativePosPIDController> idistanceController,
                                     std::unique_ptr<IterativePosPIDController> iangleController,
                                     std::unique_ptr<IterativePosPIDController> iturnController,
                                     AbstractMotor::GearsetRatioPair igearset,
                                     const ChassisScales &iscales) {
    ChassisControllerPID out(
      TimeUtilFactory::create(),
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
  createPtr(std::shared_ptr<AbstractMotor> itopLeftMtr,
            std::shared_ptr<AbstractMotor> itopRightMtr,
            std::shared_ptr<AbstractMotor> ibottomRightMtr,
            std::shared_ptr<AbstractMotor> ibottomLeftMtr,
            std::shared_ptr<ContinuousRotarySensor> ileftSensor,
            std::shared_ptr<ContinuousRotarySensor> irightSensor,
            std::unique_ptr<IterativePosPIDController> idistanceController,
            std::unique_ptr<IterativePosPIDController> iangleController,
            std::unique_ptr<IterativePosPIDController> iturnController,
            AbstractMotor::GearsetRatioPair igearset,
            const ChassisScales &iscales) {
    auto out = std::make_shared<ChassisControllerPID>(
      TimeUtilFactory::create(),
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

#endif
