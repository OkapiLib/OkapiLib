/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#ifndef _OKAPI_CHASSISCONTROLLERINTEGRATED_HPP_
#define _OKAPI_CHASSISCONTROLLERINTEGRATED_HPP_

#include "okapi/chassis/chassisController.hpp"
#include "okapi/control/integratedController.hpp"

namespace okapi {
class ChassisControllerIntegrated : public virtual ChassisController {
  public:
  ChassisControllerIntegrated(const ChassisModelParams &imodelParams,
                              const IntegratedControllerParams &icontrollerParams)
    : ChassisController(imodelParams), controller(icontrollerParams) {
  }

  ChassisControllerIntegrated(std::shared_ptr<const ChassisModel> imodel,
                              const IntegratedControllerParams &icontrollerParams)
    : ChassisController(imodel), controller(icontrollerParams) {
  }

  ChassisControllerIntegrated(const ChassisModelParams &imodelParams,
                              const IntegratedController &icontroller)
    : ChassisController(imodelParams), controller(icontroller) {
  }

  ChassisControllerIntegrated(std::shared_ptr<const ChassisModel> imodel,
                              const IntegratedController &icontroller)
    : ChassisController(imodel), controller(icontroller) {
  }

  virtual ~ChassisControllerIntegrated();

  /**
   * Drives the robot straight.
   *
   * @param itarget Distance to travel
   */
  void driveStraight(const int itarget) override {
    // TODO: How to control this using onboard? One controller for left and right side? Does
    // ChassisModel need to be changed?
  }

  /**
   * Turns the robot clockwise in place.
   *
   * @param idegTarget Degrees to turn for
   */
  void pointTurn(float idegTarget) override {
  }

  protected:
  IntegratedController controller;
};
} // namespace okapi

#endif
