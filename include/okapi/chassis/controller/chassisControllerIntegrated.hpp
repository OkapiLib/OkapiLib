/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_CHASSISCONTROLLERINTEGRATED_HPP_
#define _OKAPI_CHASSISCONTROLLERINTEGRATED_HPP_

#include "okapi/chassis/controller/chassisController.hpp"
#include "okapi/chassis/model/skidSteerModel.hpp"
#include "okapi/chassis/model/xDriveModel.hpp"
#include "okapi/control/async/posIntegratedController.hpp"

namespace okapi {
class ChassisControllerIntegrated : public virtual ChassisController {
  public:
  /**
   * ChassisController using the V5 motor's integrated control. Puts the motors' encoders into tick
   * units.
   *
   * @param imodelParams ChassisModelParams
   * @param ileftControllerParams left side controller params
   * @param irightControllerParams right side controller params
   * @param istraightScale scale converting your units of choice to encoder ticks, used for
   * measuring distance
   * @param iturnScale scale converting your units of choice to encoder ticks, used for measuring
   * angle
   */
  ChassisControllerIntegrated(const ChassisModel &imodel,
                              const PosIntegratedControllerParams &ileftControllerParams,
                              const PosIntegratedControllerParams &irightControllerParams,
                              const double istraightScale = 1, const double iturnScale = 1);

  /**
   * ChassisController using the V5 motor's integrated control. This constructor assumes a skid
   * steer layout. Puts the motors' encoders into tick units.
   *
   * @param ileftSideMotor left side motor
   * @param irightSideMotor right side motor
   * @param istraightScale scale converting your units of choice to encoder ticks, used for
   * measuring distance
   * @param iturnScale scale converting your units of choice to encoder ticks, used for measuring
   * angle
   */
  ChassisControllerIntegrated(const AbstractMotor &ileftSideMotor,
                              const AbstractMotor &irightSideMotor, const double istraightScale = 1,
                              const double iturnScale = 1);

  /**
   * ChassisController using V5 motor's integrated control. This constructor assumes an x-drive
   * layout. Puts the motors' encoders into tick units.
   *
   * @param itopLeftMotor top left motor
   * @param itopRightMotor top right motor
   * @param ibottomRightMotor bottom right motor
   * @param ibottomLeftMotor bottom left motor
   * @param istraightScale scale converting your units of choice to encoder ticks, used for
   * measuring distance
   * @param iturnScale scale converting your units of choice to encoder ticks, used for measuring
   * angle
   */
  ChassisControllerIntegrated(const AbstractMotor &itopLeftMotor,
                              const AbstractMotor &itopRightMotor,
                              const AbstractMotor &ibottomRightMotor,
                              const AbstractMotor &ibottomLeftMotor,
                              const double istraightScale = 1, const double iturnScale = 1);

  virtual ~ChassisControllerIntegrated();

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
  PosIntegratedController leftController;
  PosIntegratedController rightController;
  int lastTarget;
  const double straightScale;
  const double turnScale;
};
} // namespace okapi

#endif
