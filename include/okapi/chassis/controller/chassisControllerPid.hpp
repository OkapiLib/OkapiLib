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
#include "okapi/chassis/model/skidSteerModel.hpp"
#include "okapi/chassis/model/xDriveModel.hpp"
#include "okapi/control/iterative/posPidController.hpp"

namespace okapi {
class ChassisControllerPID : public virtual ChassisController {
  public:
  /**
   * ChassisController using PID control. Puts the motors into encoder tick units.
   *
   * @param imodelParams ChassisModelParams
   * @param idistanceParams distance PID controller params
   * @param iangleParams angle PID controller params (keeps the robot straight)
   * @param istraightScale scale converting your units of choice to encoder ticks, used for
   * measuring distance
   * @param iturnScale scale converting your units of choice to encoder ticks, used for measuring
   * angle
   */
  ChassisControllerPID(const ChassisModel &imodel, const PosPIDControllerParams &idistanceParams,
                       const PosPIDControllerParams &iangleParams, const double istraightScale = 1,
                       const double iturnScale = 1);

  /**
   * ChassisController using PID control. This constructor assumes a skid
   * steer layout. Puts the motors into encoder tick units.
   *
   * @param ileftSideMotor left side motor
   * @param irightSideMotor right side motor
   * @param idistanceParams distance PID controller params
   * @param iangleParams angle PID controller params (keeps the robot straight)
   * @param istraightScale scale converting your units of choice to encoder ticks, used for
   * measuring distance
   * @param iturnScale scale converting your units of choice to encoder ticks, used for measuring
   * angle
   */
  ChassisControllerPID(const AbstractMotor &ileftSideMotor, const AbstractMotor &irightSideMotor,
                       const PosPIDControllerParams &idistanceParams,
                       const PosPIDControllerParams &iangleParams, const double istraightScale = 1,
                       const double iturnScale = 1);

  /**
   * ChassisController using PID control. This constructor assumes an x-drive
   * layout. Puts the motors into encoder tick units.
   *
   * @param itopLeftMotor top left motor
   * @param itopRightMotor top right motor
   * @param ibottomRightMotor bottom right motor
   * @param ibottomLeftMotor bottom left motor
   * @param idistanceParams distance PID controller params
   * @param iangleParams angle PID controller params (keeps the robot straight)
   * @param istraightScale scale converting your units of choice to encoder ticks, used for
   * measuring distance
   * @param iturnScale scale converting your units of choice to encoder ticks, used for measuring
   * angle
   */
  ChassisControllerPID(const AbstractMotor &itopLeftMotor, const AbstractMotor &itopRightMotor,
                       const AbstractMotor &ibottomRightMotor,
                       const AbstractMotor &ibottomLeftMotor,
                       const PosPIDControllerParams &idistanceParams,
                       const PosPIDControllerParams &iangleParams, const double istraightScale = 1,
                       const double iturnScale = 1);

  virtual ~ChassisControllerPID();

  /**
   * Drives the robot straight for a distance (using closed-loop control).
   *
   * @param itarget distance to travel
   */
  void moveDistance(const int itarget) override;

  /**
   * Turns the robot clockwise in place (using closed-loop control).
   *
   * @param idegTarget angle to turn for
   */
  void turnAngle(float idegTarget) override;

  protected:
  PosPIDController distancePid, anglePid;
  const double straightScale;
  const double turnScale;
};
} // namespace okapi

#endif
