/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_CHASSISCONTROLLERPID_HPP_
#define _OKAPI_CHASSISCONTROLLERPID_HPP_

#include "okapi/chassis/controller/chassisController.hpp"
#include "okapi/chassis/controller/chassisScales.hpp"
#include "okapi/chassis/model/skidSteerModel.hpp"
#include "okapi/chassis/model/xDriveModel.hpp"
#include "okapi/control/iterative/iterativePosPidController.hpp"
#include "okapi/device/motor/motor.hpp"
#include "okapi/device/motor/motorGroup.hpp"
#include <memory>

namespace okapi {
class ChassisControllerPID : public virtual ChassisController {
  public:
  /**
   * ChassisController using PID control. This constructor assumes a skid
   * steer layout. Puts the motors into encoder degree units.
   *
   * @param ileftSideMotor left side motor
   * @param irightSideMotor right side motor
   * @param idistanceArgs distance PID controller params
   * @param iangleArgs angle PID controller params (keeps the robot straight)
   * @param iscales see ChassisScales docs
   */
  ChassisControllerPID(Motor ileftSideMotor, Motor irightSideMotor,
                       const IterativePosPIDControllerArgs &idistanceArgs,
                       const IterativePosPIDControllerArgs &iangleArgs,
                       const pros::c::motor_gearset_e_t igearset = pros::c::E_MOTOR_GEARSET_36,
                       const ChassisScales &iscales = ChassisScales({1, 1}));

  /**
   * ChassisController using PID control. This constructor assumes a skid
   * steer layout. Puts the motors into encoder degree units.
   *
   * @param ileftSideMotor left side motor
   * @param irightSideMotor right side motor
   * @param idistanceArgs distance PID controller params
   * @param iangleArgs angle PID controller params (keeps the robot straight)
   * @param iscales see ChassisScales docs
   */
  ChassisControllerPID(MotorGroup ileftSideMotor, MotorGroup irightSideMotor,
                       const IterativePosPIDControllerArgs &idistanceArgs,
                       const IterativePosPIDControllerArgs &iangleArgs,
                       const pros::c::motor_gearset_e_t igearset = pros::c::E_MOTOR_GEARSET_36,
                       const ChassisScales &iscales = ChassisScales({1, 1}));

  /**
   * ChassisController using PID control. This constructor assumes a skid
   * steer layout. Puts the motors into encoder degree units.
   *
   * @param ileftSideMotor left side motor
   * @param irightSideMotor right side motor
   * @param idistanceArgs distance PID controller params
   * @param iangleArgs angle PID controller params (keeps the robot straight)
   * @param iscales see ChassisScales docs
   */
  ChassisControllerPID(Motor itopLeftMotor, Motor itopRightMotor, Motor ibottomRightMotor,
                       Motor ibottomLeftMotor, const IterativePosPIDControllerArgs &idistanceArgs,
                       const IterativePosPIDControllerArgs &iangleArgs,
                       const pros::c::motor_gearset_e_t igearset = pros::c::E_MOTOR_GEARSET_36,
                       const ChassisScales &iscales = ChassisScales({1, 1}));

  /**
   * ChassisController using PID control. This constructor assumes a skid
   * steer layout. Puts the motors into encoder degree units.
   *
   * @param ileftSideMotor left side motor
   * @param irightSideMotor right side motor
   * @param idistanceArgs distance PID controller params
   * @param iangleArgs angle PID controller params (keeps the robot straight)
   * @param iscales see ChassisScales docs
   */
  ChassisControllerPID(std::shared_ptr<AbstractMotor> ileftSideMotor,
                       std::shared_ptr<AbstractMotor> irightSideMotor,
                       const IterativePosPIDControllerArgs &idistanceArgs,
                       const IterativePosPIDControllerArgs &iangleArgs,
                       const pros::c::motor_gearset_e_t igearset = pros::c::E_MOTOR_GEARSET_36,
                       const ChassisScales &iscales = ChassisScales({1, 1}));

  /**
   * ChassisController using PID control. This constructor assumes an x-drive
   * layout. Puts the motors into encoder degree units.
   *
   * @param itopLeftMotor top left motor
   * @param itopRightMotor top right motor
   * @param ibottomRightMotor bottom right motor
   * @param ibottomLeftMotor bottom left motor
   * @param idistanceArgs distance PID controller params
   * @param iangleArgs angle PID controller params (keeps the robot straight)
   * @param iscales see ChassisScales docs
   */
  ChassisControllerPID(std::shared_ptr<AbstractMotor> itopLeftMotor,
                       std::shared_ptr<AbstractMotor> itopRightMotor,
                       std::shared_ptr<AbstractMotor> ibottomRightMotor,
                       std::shared_ptr<AbstractMotor> ibottomLeftMotor,
                       const IterativePosPIDControllerArgs &idistanceArgs,
                       const IterativePosPIDControllerArgs &iangleArgs,
                       const pros::c::motor_gearset_e_t igearset = pros::c::E_MOTOR_GEARSET_36,
                       const ChassisScales &iscales = ChassisScales({1, 1}));

  /**
   * ChassisController using PID control. Puts the motors into encoder degree units.
   *
   * @param imodelArgs ChassisModelArgs
   * @param idistanceArgs distance PID controller params
   * @param iangleArgs angle PID controller params (keeps the robot straight)
   * @param iscales see ChassisScales docs
   */
  ChassisControllerPID(std::shared_ptr<ChassisModel> imodel,
                       const IterativePosPIDControllerArgs &idistanceArgs,
                       const IterativePosPIDControllerArgs &iangleArgs,
                       const pros::c::motor_gearset_e_t igearset = pros::c::E_MOTOR_GEARSET_36,
                       const ChassisScales &iscales = ChassisScales({1, 1}));

  /**
   * Drives the robot straight for a distance (using closed-loop control).
   *
   * @param itarget distance to travel
   */
  virtual void moveDistance(const QLength itarget) override;

  /**
   * Drives the robot straight for a distance (using closed-loop control).
   *
   * @param itarget distance to travel in motor degrees
   */
  virtual void moveDistance(const double itarget) override;

  /**
   * Turns the robot clockwise in place (using closed-loop control).
   *
   * @param idegTarget angle to turn for
   */
  virtual void turnAngle(const QAngle idegTarget) override;

  /**
   * Turns the robot clockwise in place (using closed-loop control).
   *
   * @param idegTarget angle to turn for in motor degrees
   */
  virtual void turnAngle(const double idegTarget) override;

  protected:
  IterativePosPIDController distancePid;
  IterativePosPIDController anglePid;
  const double straightScale;
  const double turnScale;
};
} // namespace okapi

#endif
