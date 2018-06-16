/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/chassis/controller/chassisControllerFactory.hpp"

namespace okapi {
ChassisControllerIntegrated
ChassisControllerFactory::create(Motor ileftSideMotor, Motor irightSideMotor,
                                 const AbstractMotor::motorGearset igearset,
                                 const ChassisScales &iscales) {
  return ChassisControllerIntegrated(std::make_shared<Motor>(ileftSideMotor),
                                     std::make_shared<Motor>(irightSideMotor), igearset, iscales);
}

ChassisControllerIntegrated
ChassisControllerFactory::create(MotorGroup ileftSideMotor, MotorGroup irightSideMotor,
                                 const AbstractMotor::motorGearset igearset,
                                 const ChassisScales &iscales) {
  return ChassisControllerIntegrated(std::make_shared<MotorGroup>(ileftSideMotor),
                                     std::make_shared<MotorGroup>(irightSideMotor), igearset,
                                     iscales);
}

ChassisControllerIntegrated
ChassisControllerFactory::create(Motor itopLeftMotor, Motor itopRightMotor, Motor ibottomRightMotor,
                                 Motor ibottomLeftMotor, const AbstractMotor::motorGearset igearset,
                                 const ChassisScales &iscales) {
  return ChassisControllerIntegrated(std::make_shared<Motor>(itopLeftMotor),
                                     std::make_shared<Motor>(itopRightMotor),
                                     std::make_shared<Motor>(ibottomRightMotor),
                                     std::make_shared<Motor>(ibottomLeftMotor), igearset, iscales);
}

ChassisControllerPID ChassisControllerFactory::create(
  Motor ileftSideMotor, Motor irightSideMotor, const IterativePosPIDControllerArgs &idistanceArgs,
  const IterativePosPIDControllerArgs &iangleArgs, const AbstractMotor::motorGearset igearset,
  const ChassisScales &iscales) {
  return ChassisControllerPID(std::make_shared<Motor>(ileftSideMotor),
                              std::make_shared<Motor>(irightSideMotor), idistanceArgs, iangleArgs,
                              igearset, iscales);
}

ChassisControllerPID
ChassisControllerFactory::create(MotorGroup ileftSideMotor, MotorGroup irightSideMotor,
                                 const IterativePosPIDControllerArgs &idistanceArgs,
                                 const IterativePosPIDControllerArgs &iangleArgs,
                                 const AbstractMotor::motorGearset igearset,
                                 const ChassisScales &iscales) {
  return ChassisControllerPID(std::make_shared<MotorGroup>(ileftSideMotor),
                              std::make_shared<MotorGroup>(irightSideMotor), idistanceArgs,
                              iangleArgs, igearset, iscales);
}

ChassisControllerPID ChassisControllerFactory::create(
  Motor itopLeftMotor, Motor itopRightMotor, Motor ibottomRightMotor, Motor ibottomLeftMotor,
  const IterativePosPIDControllerArgs &idistanceArgs,
  const IterativePosPIDControllerArgs &iangleArgs, const AbstractMotor::motorGearset igearset,
  const ChassisScales &iscales) {
  return ChassisControllerPID(
    std::make_shared<Motor>(itopLeftMotor), std::make_shared<Motor>(itopRightMotor),
    std::make_shared<Motor>(ibottomRightMotor), std::make_shared<Motor>(ibottomLeftMotor),
    idistanceArgs, iangleArgs, igearset, iscales);
}
} // namespace okapi
