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

namespace okapi {
class ChassisControllerFactory {
  public:
  /**
   * ChassisController using the V5 motor's integrated control. This constructor assumes a skid
   * steer layout. Puts the motors into degree units. Throws a std::invalid_argument exception if
   * the gear ratio is zero.
   *
   * @param ileftSideMotor left side motor
   * @param irightSideMotor right side motor
   * @param igearset motor internal gearset and gear ratio
   * @param iscales see ChassisScales docs
   */
  static ChassisControllerIntegrated
  create(Motor ileftSideMotor, Motor irightSideMotor,
         const AbstractMotor::GearsetRatioPair igearset = AbstractMotor::gearset::red,
         const ChassisScales &iscales = ChassisScales({1, 1}));

  /**
   * ChassisController using the V5 motor's integrated control. This constructor assumes a skid
   * steer layout. Puts the motors into degree units. Throws a std::invalid_argument exception if
   * the gear ratio is zero.
   *
   * @param ileftSideMotor left side motor
   * @param irightSideMotor right side motor
   * @param igearset motor internal gearset and gear ratio
   * @param iscales see ChassisScales docs
   */
  static ChassisControllerIntegrated
  create(MotorGroup ileftSideMotor, MotorGroup irightSideMotor,
         const AbstractMotor::GearsetRatioPair igearset = AbstractMotor::gearset::red,
         const ChassisScales &iscales = ChassisScales({1, 1}));

  /**
   * ChassisController using V5 motor's integrated control. This constructor assumes an x-drive
   * layout. Puts the motors into degree units. Throws a std::invalid_argument exception if the gear
   * ratio is zero.
   *
   * @param itopLeftMotor top left motor
   * @param itopRightMotor top right motor
   * @param ibottomRightMotor bottom right motor
   * @param ibottomLeftMotor bottom left motor
   * @param igearset motor internal gearset and gear ratio
   * @param iscales see ChassisScales docs
   */
  static ChassisControllerIntegrated
  create(Motor itopLeftMotor, Motor itopRightMotor, Motor ibottomRightMotor, Motor ibottomLeftMotor,
         const AbstractMotor::GearsetRatioPair igearset = AbstractMotor::gearset::red,
         const ChassisScales &iscales = ChassisScales({1, 1}));

  /**
   * ChassisController using PID control. This constructor assumes a skid
   * steer layout. Puts the motors into encoder degree units. Throws a std::invalid_argument
   * exception if the gear ratio is zero.
   *
   * @param ileftSideMotor left side motor
   * @param irightSideMotor right side motor
   * @param idistanceArgs distance PID controller params
   * @param iangleArgs angle PID controller params (keeps the robot straight)
   * @param igearset motor internal gearset and gear ratio
   * @param iscales see ChassisScales docs
   */
  static ChassisControllerPID
  create(Motor ileftSideMotor, Motor irightSideMotor,
         const IterativePosPIDControllerArgs &idistanceArgs,
         const IterativePosPIDControllerArgs &iangleArgs,
         const AbstractMotor::GearsetRatioPair igearset = AbstractMotor::gearset::red,
         const ChassisScales &iscales = ChassisScales({1, 1}));

  /**
   * ChassisController using PID control. This constructor assumes a skid
   * steer layout. Puts the motors into encoder degree units. Throws a std::invalid_argument
   * exception if the gear ratio is zero.
   *
   * @param ileftSideMotor left side motor
   * @param irightSideMotor right side motor
   * @param idistanceArgs distance PID controller params
   * @param iangleArgs angle PID controller params (keeps the robot straight)
   * @param igearset motor internal gearset and gear ratio
   * @param iscales see ChassisScales docs
   */
  static ChassisControllerPID
  create(MotorGroup ileftSideMotor, MotorGroup irightSideMotor,
         const IterativePosPIDControllerArgs &idistanceArgs,
         const IterativePosPIDControllerArgs &iangleArgs,
         const AbstractMotor::GearsetRatioPair igearset = AbstractMotor::gearset::red,
         const ChassisScales &iscales = ChassisScales({1, 1}));

  /**
   * ChassisController using PID control. This constructor assumes a skid
   * steer layout. Puts the motors into encoder degree units. Throws a std::invalid_argument
   * exception if the gear ratio is zero.
   *
   * @param ileftSideMotor left side motor
   * @param irightSideMotor right side motor
   * @param idistanceArgs distance PID controller params
   * @param iangleArgs angle PID controller params (keeps the robot straight)
   * @param igearset motor internal gearset and gear ratio
   * @param iscales see ChassisScales docs
   */
  static ChassisControllerPID
  create(Motor itopLeftMotor, Motor itopRightMotor, Motor ibottomRightMotor, Motor ibottomLeftMotor,
         const IterativePosPIDControllerArgs &idistanceArgs,
         const IterativePosPIDControllerArgs &iangleArgs,
         const AbstractMotor::GearsetRatioPair igearset = AbstractMotor::gearset::red,
         const ChassisScales &iscales = ChassisScales({1, 1}));
};
} // namespace okapi

#endif
