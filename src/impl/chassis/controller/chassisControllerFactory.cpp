/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/impl/chassis/controller/chassisControllerFactory.hpp"
#include "okapi/api/chassis/model/skidSteerModel.hpp"
#include "okapi/api/chassis/model/xDriveModel.hpp"
#include "okapi/impl/util/timeUtilFactory.hpp"

namespace okapi {
ChassisControllerIntegrated
ChassisControllerFactory::create(Motor ileftSideMotor, Motor irightSideMotor,
                                 const AbstractMotor::GearsetRatioPair igearset,
                                 const ChassisScales &iscales) {
  auto leftMtr = std::make_shared<Motor>(ileftSideMotor);
  auto rightMtr = std::make_shared<Motor>(irightSideMotor);
  return ChassisControllerIntegrated(TimeUtilFactory::create(),
                                     std::make_unique<SkidSteerModel>(leftMtr, rightMtr),
                                     AsyncPosIntegratedControllerArgs(leftMtr),
                                     AsyncPosIntegratedControllerArgs(rightMtr), igearset, iscales);
}

ChassisControllerIntegrated
ChassisControllerFactory::create(MotorGroup ileftSideMotor, MotorGroup irightSideMotor,
                                 const AbstractMotor::GearsetRatioPair igearset,
                                 const ChassisScales &iscales) {
  auto leftMtr = std::make_shared<MotorGroup>(ileftSideMotor);
  auto rightMtr = std::make_shared<MotorGroup>(irightSideMotor);
  return ChassisControllerIntegrated(TimeUtilFactory::create(),
                                     std::make_unique<SkidSteerModel>(leftMtr, rightMtr),
                                     AsyncPosIntegratedControllerArgs(leftMtr),
                                     AsyncPosIntegratedControllerArgs(rightMtr), igearset, iscales);
}

ChassisControllerIntegrated ChassisControllerFactory::create(
  Motor itopLeftMotor, Motor itopRightMotor, Motor ibottomRightMotor, Motor ibottomLeftMotor,
  const AbstractMotor::GearsetRatioPair igearset, const ChassisScales &iscales) {
  auto topLeftMtr = std::make_shared<Motor>(itopLeftMotor);
  auto topRightMtr = std::make_shared<Motor>(itopRightMotor);
  auto bottomRightMtr = std::make_shared<Motor>(ibottomRightMotor);
  auto bottomLeftMtr = std::make_shared<Motor>(ibottomLeftMotor);
  return ChassisControllerIntegrated(
    TimeUtilFactory::create(),
    std::make_unique<XDriveModel>(topLeftMtr, topRightMtr, bottomRightMtr, bottomLeftMtr),
    AsyncPosIntegratedControllerArgs(topLeftMtr), AsyncPosIntegratedControllerArgs(topRightMtr),
    igearset, iscales);
}

ChassisControllerPID ChassisControllerFactory::create(
  Motor ileftSideMotor, Motor irightSideMotor, const IterativePosPIDControllerArgs &idistanceArgs,
  const IterativePosPIDControllerArgs &iangleArgs, const AbstractMotor::GearsetRatioPair igearset,
  const ChassisScales &iscales) {
  auto leftMtr = std::make_shared<Motor>(ileftSideMotor);
  auto rightMtr = std::make_shared<Motor>(irightSideMotor);
  return ChassisControllerPID(TimeUtilFactory::create(),
                              std::make_unique<SkidSteerModel>(leftMtr, rightMtr), idistanceArgs,
                              iangleArgs, igearset, iscales);
}

ChassisControllerPID
ChassisControllerFactory::create(MotorGroup ileftSideMotor, MotorGroup irightSideMotor,
                                 const IterativePosPIDControllerArgs &idistanceArgs,
                                 const IterativePosPIDControllerArgs &iangleArgs,
                                 const AbstractMotor::GearsetRatioPair igearset,
                                 const ChassisScales &iscales) {
  auto leftMtr = std::make_shared<MotorGroup>(ileftSideMotor);
  auto rightMtr = std::make_shared<MotorGroup>(irightSideMotor);
  return ChassisControllerPID(TimeUtilFactory::create(),
                              std::make_unique<SkidSteerModel>(leftMtr, rightMtr), idistanceArgs,
                              iangleArgs, igearset, iscales);
}

ChassisControllerPID ChassisControllerFactory::create(
  MotorGroup ileftSideMotor, MotorGroup irightSideMotor, ADIEncoder ileftEnc, ADIEncoder irightEnc,
  const IterativePosPIDControllerArgs &idistanceArgs,
  const IterativePosPIDControllerArgs &iangleArgs, const AbstractMotor::GearsetRatioPair igearset,
  const ChassisScales &iscales) {
  auto leftMtr = std::make_shared<MotorGroup>(ileftSideMotor);
  auto rightMtr = std::make_shared<MotorGroup>(irightSideMotor);
  return ChassisControllerPID(
    TimeUtilFactory::create(),
    std::make_unique<SkidSteerModel>(leftMtr, rightMtr, std::make_shared<ADIEncoder>(ileftEnc),
                                     std::make_shared<ADIEncoder>(irightEnc)),
    idistanceArgs, iangleArgs, igearset, iscales);
}

ChassisControllerPID ChassisControllerFactory::create(
  std::shared_ptr<AbstractMotor> ileftSideMotor, std::shared_ptr<AbstractMotor> irightSideMotor,
  std::shared_ptr<ContinuousRotarySensor> ileftEnc,
  std::shared_ptr<ContinuousRotarySensor> irightEnc,
  const IterativePosPIDControllerArgs &idistanceArgs,
  const IterativePosPIDControllerArgs &iangleArgs, const AbstractMotor::GearsetRatioPair igearset,
  const ChassisScales &iscales) {
  return ChassisControllerPID(
    TimeUtilFactory::create(),
    std::make_unique<SkidSteerModel>(ileftSideMotor, irightSideMotor, ileftEnc, irightEnc),
    idistanceArgs, iangleArgs, igearset, iscales);
}

ChassisControllerPID ChassisControllerFactory::create(
  Motor itopLeftMotor, Motor itopRightMotor, Motor ibottomRightMotor, Motor ibottomLeftMotor,
  const IterativePosPIDControllerArgs &idistanceArgs,
  const IterativePosPIDControllerArgs &iangleArgs, const AbstractMotor::GearsetRatioPair igearset,
  const ChassisScales &iscales) {
  auto topLeftMtr = std::make_shared<Motor>(itopLeftMotor);
  auto topRightMtr = std::make_shared<Motor>(itopRightMotor);
  auto bottomRightMtr = std::make_shared<Motor>(ibottomRightMotor);
  auto bottomLeftMtr = std::make_shared<Motor>(ibottomLeftMotor);
  return ChassisControllerPID(
    TimeUtilFactory::create(),
    std::make_unique<XDriveModel>(topLeftMtr, topRightMtr, bottomRightMtr, bottomLeftMtr),
    idistanceArgs, iangleArgs, igearset, iscales);
}

ChassisControllerPID ChassisControllerFactory::create(
  Motor itopLeftMotor, Motor itopRightMotor, Motor ibottomRightMotor, Motor ibottomLeftMotor,
  ADIEncoder itopLeftEnc, ADIEncoder itopRightEnc,
  const IterativePosPIDControllerArgs &idistanceArgs,
  const IterativePosPIDControllerArgs &iangleArgs, const AbstractMotor::GearsetRatioPair igearset,
  const ChassisScales &iscales) {
  auto topLeftMtr = std::make_shared<Motor>(itopLeftMotor);
  auto topRightMtr = std::make_shared<Motor>(itopRightMotor);
  auto bottomRightMtr = std::make_shared<Motor>(ibottomRightMotor);
  auto bottomLeftMtr = std::make_shared<Motor>(ibottomLeftMotor);
  return ChassisControllerPID(
    TimeUtilFactory::create(),
    std::make_unique<XDriveModel>(topLeftMtr, topRightMtr, bottomRightMtr, bottomLeftMtr,
                                  std::make_shared<ADIEncoder>(itopLeftEnc),
                                  std::make_shared<ADIEncoder>(itopRightEnc)),
    idistanceArgs, iangleArgs, igearset, iscales);
}

ChassisControllerPID ChassisControllerFactory::create(
  std::shared_ptr<AbstractMotor> itopLeftMotor, std::shared_ptr<AbstractMotor> itopRightMotor,
  std::shared_ptr<AbstractMotor> ibottomRightMotor, std::shared_ptr<AbstractMotor> ibottomLeftMotor,
  std::shared_ptr<ContinuousRotarySensor> itopLeftEnc,
  std::shared_ptr<ContinuousRotarySensor> itopRightEnc,
  const IterativePosPIDControllerArgs &idistanceArgs,
  const IterativePosPIDControllerArgs &iangleArgs, const AbstractMotor::GearsetRatioPair igearset,
  const ChassisScales &iscales) {
  return ChassisControllerPID(TimeUtilFactory::create(),
                              std::make_unique<XDriveModel>(itopLeftMotor, itopRightMotor,
                                                            ibottomRightMotor, ibottomLeftMotor,
                                                            itopLeftEnc, itopRightEnc),
                              idistanceArgs, iangleArgs, igearset, iscales);
}
} // namespace okapi
