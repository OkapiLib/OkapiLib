/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
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
   * ChassisController using the V5 motor's integrated control.
   *
   * @param imodelParams ChassisModelParams
   * @param ileftControllerParams left side controller params
   * @param irightControllerParams right side controller params
   */
  ChassisControllerIntegrated(const ChassisModelParams &imodelParams,
                              const PosIntegratedControllerParams &ileftControllerParams,
                              const PosIntegratedControllerParams &irightControllerParams);

  ChassisControllerIntegrated(std::shared_ptr<const ChassisModel> imodel,
                              const PosIntegratedControllerParams &ileftControllerParams,
                              const PosIntegratedControllerParams &irightControllerParams);

  /**
   * ChassisController using the V5 motor's integrated control. This constructor assumes a skid
   * steer layout.
   *
   * @param ileftSideMotor left side motor
   * @param irightSideMotor right side motor
   */
  ChassisControllerIntegrated(const AbstractMotor &ileftSideMotor,
                              const AbstractMotor &irightSideMotor);

  /**
   * ChassisController using V5 motor's integrated control. This constructor assumes an x-drive
   * layout.
   *
   * @param itopLeftMotor top left motor
   * @param itopRightMotor top right motor
   * @param ibottomRightMotor bottom right motor
   * @param ibottomLeftMotor bottom left motor
   */
  ChassisControllerIntegrated(const AbstractMotor &itopLeftMotor,
                              const AbstractMotor &itopRightMotor,
                              const AbstractMotor &ibottomRightMotor,
                              const AbstractMotor &ibottomLeftMotor);

  virtual ~ChassisControllerIntegrated();

  /**
   * Drives the robot straight.
   *
   * @param itarget Distance to travel
   */
  void driveStraight(const int itarget) override;

  /**
   * Turns the robot clockwise in place.
   *
   * @param idegTarget Degrees to turn for
   */
  void pointTurn(float idegTarget) override;

  protected:
  PosIntegratedController leftController;
  PosIntegratedController rightController;
  int lastTarget;
};
} // namespace okapi

#endif
