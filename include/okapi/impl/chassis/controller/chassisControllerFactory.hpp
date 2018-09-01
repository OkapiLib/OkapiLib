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
#include "okapi/impl/device/motor/motor.hpp"
#include "okapi/impl/device/motor/motorGroup.hpp"
#include "okapi/impl/device/rotarysensor/adiEncoder.hpp"

namespace okapi {
class ChassisControllerFactory {
  public:
  /**
   * ChassisController using the V5 motor's integrated control. This constructor assumes a skid
   * steer layout. Puts the motors into degree units. Throws a std::invalid_argument exception if
   * the gear ratio is zero.
   *
   * @param ileftSideMotor left side motor (also used for controller input)
   * @param irightSideMotor right side motor (also used for controller input)
   * @param igearset motor internal gearset and gear ratio
   * @param iscales see ChassisScales docs
   */
  static ChassisControllerIntegrated
  create(Motor ileftSideMotor,
         Motor irightSideMotor,
         AbstractMotor::GearsetRatioPair igearset = AbstractMotor::gearset::red,
         const ChassisScales &iscales = ChassisScales({1, 1}));

  /**
   * ChassisController using the V5 motor's integrated control. This constructor assumes a skid
   * steer layout. Puts the motors into degree units. Throws a std::invalid_argument exception if
   * the gear ratio is zero.
   *
   * @param ileftSideMotor left side motor (also used for controller input)
   * @param irightSideMotor right side motor (also used for controller input)
   * @param igearset motor internal gearset and gear ratio
   * @param iscales see ChassisScales docs
   */
  static ChassisControllerIntegrated
  create(MotorGroup ileftSideMotor,
         MotorGroup irightSideMotor,
         AbstractMotor::GearsetRatioPair igearset = AbstractMotor::gearset::red,
         const ChassisScales &iscales = ChassisScales({1, 1}));

  /**
   * ChassisController using V5 motor's integrated control. This constructor assumes an x-drive
   * layout. Puts the motors into degree units. Throws a std::invalid_argument exception if the gear
   * ratio is zero.
   *
   * @param itopLeftMotor top left motor (also used for controller input)
   * @param itopRightMotor top right motor (also used for controller input)
   * @param ibottomRightMotor bottom right motor
   * @param ibottomLeftMotor bottom left motor
   * @param igearset motor internal gearset and gear ratio
   * @param iscales see ChassisScales docs
   */
  static ChassisControllerIntegrated
  create(Motor itopLeftMotor,
         Motor itopRightMotor,
         Motor ibottomRightMotor,
         Motor ibottomLeftMotor,
         AbstractMotor::GearsetRatioPair igearset = AbstractMotor::gearset::red,
         const ChassisScales &iscales = ChassisScales({1, 1}));

  /**
   * ChassisController using PID control. This constructor assumes a skid
   * steer layout. Puts the motors into encoder degree units. Throws a std::invalid_argument
   * exception if the gear ratio is zero.
   *
   * @param ileftSideMotor left side motor (also used for controller input)
   * @param irightSideMotor right side motor (also used for controller input)
   * @param idistanceArgs distance PID controller params
   * @param iangleArgs angle PID controller params (keeps the robot straight)
   * @param igearset motor internal gearset and gear ratio
   * @param iscales see ChassisScales docs
   */
  static ChassisControllerPID
  create(Motor ileftSideMotor,
         Motor irightSideMotor,
         const IterativePosPIDController::Gains &idistanceArgs,
         const IterativePosPIDController::Gains &iangleArgs,
         AbstractMotor::GearsetRatioPair igearset = AbstractMotor::gearset::red,
         const ChassisScales &iscales = ChassisScales({1, 1}));

  /**
   * ChassisController using PID control. This constructor assumes a skid
   * steer layout. Puts the motors into encoder degree units. Throws a std::invalid_argument
   * exception if the gear ratio is zero.
   *
   * @param ileftSideMotor left side motor (also used for controller input)
   * @param irightSideMotor right side motor (also used for controller input)
   * @param idistanceArgs distance PID controller params
   * @param iangleArgs angle PID controller params (keeps the robot straight)
   * @param igearset motor internal gearset and gear ratio
   * @param iscales see ChassisScales docs
   */
  static ChassisControllerPID
  create(Motor ileftSideMotor,
         Motor irightSideMotor,
         const IterativePosPIDController::Gains &idistanceArgs,
         const IterativePosPIDController::Gains &iangleArgs,
         const IterativePosPIDController::Gains &iturnArgs,
         AbstractMotor::GearsetRatioPair igearset = AbstractMotor::gearset::red,
         const ChassisScales &iscales = ChassisScales({1, 1}));

  /**
   * ChassisController using PID control. This constructor assumes a skid
   * steer layout. Puts the motors into encoder degree units. Throws a std::invalid_argument
   * exception if the gear ratio is zero.
   *
   * @param ileftSideMotor left side motor (also used for controller input)
   * @param irightSideMotor right side motor (also used for controller input)
   * @param idistanceArgs distance PID controller params
   * @param iangleArgs angle PID controller params (keeps the robot straight)
   * @param igearset motor internal gearset and gear ratio
   * @param iscales see ChassisScales docs
   */
  static ChassisControllerPID
  create(MotorGroup ileftSideMotor,
         MotorGroup irightSideMotor,
         const IterativePosPIDController::Gains &idistanceArgs,
         const IterativePosPIDController::Gains &iangleArgs,
         AbstractMotor::GearsetRatioPair igearset = AbstractMotor::gearset::red,
         const ChassisScales &iscales = ChassisScales({1, 1}));

  /**
   * ChassisController using PID control. This constructor assumes a skid
   * steer layout. Puts the motors into encoder degree units. Throws a std::invalid_argument
   * exception if the gear ratio is zero.
   *
   * @param ileftSideMotor left side motor (also used for controller input)
   * @param irightSideMotor right side motor (also used for controller input)
   * @param idistanceArgs distance PID controller params
   * @param iangleArgs angle PID controller params (keeps the robot straight)
   * @param igearset motor internal gearset and gear ratio
   * @param iscales see ChassisScales docs
   */
  static ChassisControllerPID
  create(MotorGroup ileftSideMotor,
         MotorGroup irightSideMotor,
         const IterativePosPIDController::Gains &idistanceArgs,
         const IterativePosPIDController::Gains &iangleArgs,
         const IterativePosPIDController::Gains &iturnArgs,
         AbstractMotor::GearsetRatioPair igearset = AbstractMotor::gearset::red,
         const ChassisScales &iscales = ChassisScales({1, 1}));

  /**
   * ChassisController using PID control. This constructor assumes a skid
   * steer layout. Puts the motors into encoder degree units. Throws a std::invalid_argument
   * exception if the gear ratio is zero.
   *
   * @param ileftSideMotor left side motor
   * @param irightSideMotor right side motor
   * @param ileftEnc left side encoder
   * @param irightEnc right side encoder
   * @param idistanceArgs distance PID controller params
   * @param iangleArgs angle PID controller params (keeps the robot straight)
   * @param igearset motor internal gearset and gear ratio
   * @param iscales see ChassisScales docs
   */
  static ChassisControllerPID
  create(MotorGroup ileftSideMotor,
         MotorGroup irightSideMotor,
         ADIEncoder ileftEnc,
         ADIEncoder irightEnc,
         const IterativePosPIDController::Gains &idistanceArgs,
         const IterativePosPIDController::Gains &iangleArgs,
         AbstractMotor::GearsetRatioPair igearset = AbstractMotor::gearset::red,
         const ChassisScales &iscales = ChassisScales({1, 1}));

  /**
   * ChassisController using PID control. This constructor assumes a skid
   * steer layout. Puts the motors into encoder degree units. Throws a std::invalid_argument
   * exception if the gear ratio is zero.
   *
   * @param ileftSideMotor left side motor
   * @param irightSideMotor right side motor
   * @param ileftEnc left side encoder
   * @param irightEnc right side encoder
   * @param idistanceArgs distance PID controller params
   * @param iangleArgs angle PID controller params (keeps the robot straight)
   * @param igearset motor internal gearset and gear ratio
   * @param iscales see ChassisScales docs
   */
  static ChassisControllerPID
  create(MotorGroup ileftSideMotor,
         MotorGroup irightSideMotor,
         ADIEncoder ileftEnc,
         ADIEncoder irightEnc,
         const IterativePosPIDController::Gains &idistanceArgs,
         const IterativePosPIDController::Gains &iangleArgs,
         const IterativePosPIDController::Gains &iturnArgs,
         AbstractMotor::GearsetRatioPair igearset = AbstractMotor::gearset::red,
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
  create(std::shared_ptr<AbstractMotor> ileftSideMotor,
         std::shared_ptr<AbstractMotor> irightSideMotor,
         std::shared_ptr<ContinuousRotarySensor> ileftEnc,
         std::shared_ptr<ContinuousRotarySensor> irightEnc,
         const IterativePosPIDController::Gains &idistanceArgs,
         const IterativePosPIDController::Gains &iangleArgs,
         AbstractMotor::GearsetRatioPair igearset = AbstractMotor::gearset::red,
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
  create(std::shared_ptr<AbstractMotor> ileftSideMotor,
         std::shared_ptr<AbstractMotor> irightSideMotor,
         std::shared_ptr<ContinuousRotarySensor> ileftEnc,
         std::shared_ptr<ContinuousRotarySensor> irightEnc,
         const IterativePosPIDController::Gains &idistanceArgs,
         const IterativePosPIDController::Gains &iangleArgs,
         const IterativePosPIDController::Gains &iturnArgs,
         AbstractMotor::GearsetRatioPair igearset = AbstractMotor::gearset::red,
         const ChassisScales &iscales = ChassisScales({1, 1}));

  /**
   * ChassisController using PID control. This constructor assumes a skid
   * steer layout. Puts the motors into encoder degree units. Throws a std::invalid_argument
   * exception if the gear ratio is zero.
   *
   * @param itopLeftMotor top left motor (also used for controller input)
   * @param itopRightMotor top right motor (also used for controller input)
   * @param ibottomRightMotor bottom right motor
   * @param ibottomLeftMotor bottom left motor
   * @param idistanceArgs distance PID controller params
   * @param iangleArgs angle PID controller params (keeps the robot straight)
   * @param igearset motor internal gearset and gear ratio
   * @param iscales see ChassisScales docs
   */
  static ChassisControllerPID
  create(Motor itopLeftMotor,
         Motor itopRightMotor,
         Motor ibottomRightMotor,
         Motor ibottomLeftMotor,
         const IterativePosPIDController::Gains &idistanceArgs,
         const IterativePosPIDController::Gains &iangleArgs,
         AbstractMotor::GearsetRatioPair igearset = AbstractMotor::gearset::red,
         const ChassisScales &iscales = ChassisScales({1, 1}));

  /**
   * ChassisController using PID control. This constructor assumes a skid
   * steer layout. Puts the motors into encoder degree units. Throws a std::invalid_argument
   * exception if the gear ratio is zero.
   *
   * @param itopLeftMotor top left motor (also used for controller input)
   * @param itopRightMotor top right motor (also used for controller input)
   * @param ibottomRightMotor bottom right motor
   * @param ibottomLeftMotor bottom left motor
   * @param idistanceArgs distance PID controller params
   * @param iangleArgs angle PID controller params (keeps the robot straight)
   * @param igearset motor internal gearset and gear ratio
   * @param iscales see ChassisScales docs
   */
  static ChassisControllerPID
  create(Motor itopLeftMotor,
         Motor itopRightMotor,
         Motor ibottomRightMotor,
         Motor ibottomLeftMotor,
         const IterativePosPIDController::Gains &idistanceArgs,
         const IterativePosPIDController::Gains &iangleArgs,
         const IterativePosPIDController::Gains &iturnArgs,
         AbstractMotor::GearsetRatioPair igearset = AbstractMotor::gearset::red,
         const ChassisScales &iscales = ChassisScales({1, 1}));

  /**
   * ChassisController using PID control. This constructor assumes a skid
   * steer layout. Puts the motors into encoder degree units. Throws a std::invalid_argument
   * exception if the gear ratio is zero.
   *
   * @param itopLeftMotor top left motor
   * @param itopRightMotor top right motor
   * @param ibottomRightMotor bottom right motor
   * @param ibottomLeftMotor bottom left motor
   * @param itopLeftEnc top left encoder
   * @param itopRightEnc top right encoder
   * @param irightEnc right side encoder
   * @param idistanceArgs distance PID controller params
   * @param iangleArgs angle PID controller params (keeps the robot straight)
   * @param igearset motor internal gearset and gear ratio
   * @param iscales see ChassisScales docs
   */
  static ChassisControllerPID
  create(Motor itopLeftMotor,
         Motor itopRightMotor,
         Motor ibottomRightMotor,
         Motor ibottomLeftMotor,
         ADIEncoder itopLeftEnc,
         ADIEncoder itopRightEnc,
         const IterativePosPIDController::Gains &idistanceArgs,
         const IterativePosPIDController::Gains &iangleArgs,
         AbstractMotor::GearsetRatioPair igearset = AbstractMotor::gearset::red,
         const ChassisScales &iscales = ChassisScales({1, 1}));

  /**
   * ChassisController using PID control. This constructor assumes a skid
   * steer layout. Puts the motors into encoder degree units. Throws a std::invalid_argument
   * exception if the gear ratio is zero.
   *
   * @param itopLeftMotor top left motor
   * @param itopRightMotor top right motor
   * @param ibottomRightMotor bottom right motor
   * @param ibottomLeftMotor bottom left motor
   * @param itopLeftEnc top left encoder
   * @param itopRightEnc top right encoder
   * @param irightEnc right side encoder
   * @param idistanceArgs distance PID controller params
   * @param iangleArgs angle PID controller params (keeps the robot straight)
   * @param igearset motor internal gearset and gear ratio
   * @param iscales see ChassisScales docs
   */
  static ChassisControllerPID
  create(Motor itopLeftMotor,
         Motor itopRightMotor,
         Motor ibottomRightMotor,
         Motor ibottomLeftMotor,
         ADIEncoder itopLeftEnc,
         ADIEncoder itopRightEnc,
         const IterativePosPIDController::Gains &idistanceArgs,
         const IterativePosPIDController::Gains &iangleArgs,
         const IterativePosPIDController::Gains &iturnArgs,
         AbstractMotor::GearsetRatioPair igearset = AbstractMotor::gearset::red,
         const ChassisScales &iscales = ChassisScales({1, 1}));

  /**
   * ChassisController using PID control. This constructor assumes a skid
   * steer layout. Puts the motors into encoder degree units. Throws a std::invalid_argument
   * exception if the gear ratio is zero.
   *
   * @param itopLeftMotor top left motor
   * @param itopRightMotor top right motor
   * @param ibottomRightMotor bottom right motor
   * @param ibottomLeftMotor bottom left motor
   * @param itopLeftEnc top left encoder
   * @param itopRightEnc top right encoder
   * @param irightEnc right side encoder
   * @param idistanceArgs distance PID controller params
   * @param iangleArgs angle PID controller params (keeps the robot straight)
   * @param igearset motor internal gearset and gear ratio
   * @param iscales see ChassisScales docs
   */
  static ChassisControllerPID
  create(std::shared_ptr<AbstractMotor> itopLeftMotor,
         std::shared_ptr<AbstractMotor> itopRightMotor,
         std::shared_ptr<AbstractMotor> ibottomRightMotor,
         std::shared_ptr<AbstractMotor> ibottomLeftMotor,
         std::shared_ptr<ContinuousRotarySensor> itopLeftEnc,
         std::shared_ptr<ContinuousRotarySensor> itopRightEnc,
         const IterativePosPIDController::Gains &idistanceArgs,
         const IterativePosPIDController::Gains &iangleArgs,
         AbstractMotor::GearsetRatioPair igearset = AbstractMotor::gearset::red,
         const ChassisScales &iscales = ChassisScales({1, 1}));

  /**
   * ChassisController using PID control. This constructor assumes a skid
   * steer layout. Puts the motors into encoder degree units. Throws a std::invalid_argument
   * exception if the gear ratio is zero.
   *
   * @param itopLeftMotor top left motor
   * @param itopRightMotor top right motor
   * @param ibottomRightMotor bottom right motor
   * @param ibottomLeftMotor bottom left motor
   * @param itopLeftEnc top left encoder
   * @param itopRightEnc top right encoder
   * @param irightEnc right side encoder
   * @param idistanceArgs distance PID controller params
   * @param iangleArgs angle PID controller params (keeps the robot straight)
   * @param igearset motor internal gearset and gear ratio
   * @param iscales see ChassisScales docs
   */
  static ChassisControllerPID
  create(std::shared_ptr<AbstractMotor> itopLeftMotor,
         std::shared_ptr<AbstractMotor> itopRightMotor,
         std::shared_ptr<AbstractMotor> ibottomRightMotor,
         std::shared_ptr<AbstractMotor> ibottomLeftMotor,
         std::shared_ptr<ContinuousRotarySensor> itopLeftEnc,
         std::shared_ptr<ContinuousRotarySensor> itopRightEnc,
         const IterativePosPIDController::Gains &idistanceArgs,
         const IterativePosPIDController::Gains &iangleArgs,
         const IterativePosPIDController::Gains &iturnArgs,
         AbstractMotor::GearsetRatioPair igearset = AbstractMotor::gearset::red,
         const ChassisScales &iscales = ChassisScales({1, 1}));
};
} // namespace okapi

#endif
