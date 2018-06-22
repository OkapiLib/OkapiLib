/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_CHASSISCONTROLLERPID_HPP_
#define _OKAPI_CHASSISCONTROLLERPID_HPP_

#include "okapi/api/chassis/controller/chassisController.hpp"
#include "okapi/api/chassis/controller/chassisScales.hpp"
#include "okapi/api/control/iterative/iterativePosPidController.hpp"
#include <memory>

namespace okapi {
class ChassisControllerPID : public virtual ChassisController {
  public:
  /**
   * ChassisController using PID control. Puts the motors into encoder degree units.
   *
   * @param imodelArgs ChassisModelArgs
   * @param idistanceArgs distance PID controller params
   * @param iangleArgs angle PID controller params (keeps the robot straight)
   * @param iscales see ChassisScales docs
   */
  ChassisControllerPID(
    std::shared_ptr<ChassisModel> imodel, const IterativePosPIDControllerArgs &idistanceArgs,
    const IterativePosPIDControllerArgs &iangleArgs,
    const AbstractMotor::gearset igearset = AbstractMotor::gearset::E_MOTOR_GEARSET_36,
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
