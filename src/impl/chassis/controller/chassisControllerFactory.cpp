/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/impl/chassis/controller/chassisControllerFactory.hpp"
#include "okapi/impl/chassis/model/skidSteerModel.hpp"
#include "okapi/impl/chassis/model/xDriveModel.hpp"

namespace okapi {
ChassisControllerIntegrated
ChassisControllerFactory::create(Motor ileftSideMotor, Motor irightSideMotor,
                                 const AbstractMotor::GearsetRatioPair igearset,
                                 const ChassisScales &iscales) {
  auto leftMtr = std::make_shared<Motor>(ileftSideMotor);
  auto rightMtr = std::make_shared<Motor>(irightSideMotor);
  return ChassisControllerIntegrated(std::make_shared<SkidSteerModel>(leftMtr, rightMtr),
                                     AsyncPosIntegratedControllerArgs(leftMtr),
                                     AsyncPosIntegratedControllerArgs(rightMtr), igearset, iscales);
}

ChassisControllerIntegrated
ChassisControllerFactory::create(MotorGroup ileftSideMotor, MotorGroup irightSideMotor,
                                 const AbstractMotor::GearsetRatioPair igearset,
                                 const ChassisScales &iscales) {
  auto leftMtr = std::make_shared<MotorGroup>(ileftSideMotor);
  auto rightMtr = std::make_shared<MotorGroup>(irightSideMotor);
  return ChassisControllerIntegrated(std::make_shared<SkidSteerModel>(leftMtr, rightMtr),
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
    std::make_shared<XDriveModel>(topLeftMtr, topRightMtr, bottomRightMtr, bottomLeftMtr),
    AsyncPosIntegratedControllerArgs(topLeftMtr), AsyncPosIntegratedControllerArgs(topRightMtr),
    igearset, iscales);
}

ChassisControllerPID ChassisControllerFactory::create(
  Motor ileftSideMotor, Motor irightSideMotor, const IterativePosPIDControllerArgs &idistanceArgs,
  const IterativePosPIDControllerArgs &iangleArgs, const AbstractMotor::GearsetRatioPair igearset,
  const ChassisScales &iscales) {
  auto leftMtr = std::make_shared<Motor>(ileftSideMotor);
  auto rightMtr = std::make_shared<Motor>(irightSideMotor);
  return ChassisControllerPID(std::make_shared<SkidSteerModel>(leftMtr, rightMtr), idistanceArgs,
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
  return ChassisControllerPID(std::make_shared<SkidSteerModel>(leftMtr, rightMtr), idistanceArgs,
                              iangleArgs, igearset, iscales);
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
    std::make_shared<XDriveModel>(topLeftMtr, topRightMtr, bottomRightMtr, bottomLeftMtr),
    idistanceArgs, iangleArgs, igearset, iscales);
}
} // namespace okapi
