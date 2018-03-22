/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#ifndef _OKAPI_CHASSISCONTROLLERPID_HPP_
#define _OKAPI_CHASSISCONTROLLERPID_HPP_

#include "okapi/chassis/controller/chassisController.hpp"
#include "okapi/control/iterative/posPidController.hpp"

namespace okapi {
class ChassisControllerPID : public virtual ChassisController {
  public:
  /**
   * ChassisController using PID control.
   *
   * @param imodelParams ChassisModelParams
   * @param idistanceParams distance PID controller params
   * @param iangleParams angle PID controller params (keeps the robot straight)
   */
  ChassisControllerPID(const ChassisModelParams &imodelParams,
                       const PosPIDControllerParams &idistanceParams,
                       const PosPIDControllerParams &iangleParams);

  ChassisControllerPID(std::shared_ptr<const ChassisModel> imodel,
                       const PosPIDControllerParams &idistanceParams,
                       const PosPIDControllerParams &iangleParams);

  ChassisControllerPID(const ChassisModelParams &imodelParams, const PosPIDController &idistance,
                       const PosPIDController &iangle);

  ChassisControllerPID(std::shared_ptr<const ChassisModel> imodel,
                       const PosPIDController &idistance, const PosPIDController &iangle);

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
   * @param idegTarget degrees to turn for
   */
  void turnAngle(float idegTarget) override;

  protected:
  PosPIDController distancePid, anglePid;
};
} // namespace okapi

#endif
