/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#ifndef _OKAPI_CHASSISCONTROLLERPID_HPP_
#define _OKAPI_CHASSISCONTROLLERPID_HPP_

#include "okapi/chassis/chassisController.hpp"
#include "okapi/control/pidController.hpp"
#include <memory>

namespace okapi {
  class ChassisControllerPid : public virtual ChassisController {
  public:
    ChassisControllerPid(const ChassisModelParams& imodelParams,
      const PidControllerParams& idistanceParams, const PidControllerParams& iangleParams):
      ChassisController(imodelParams),
      distancePid(idistanceParams),
      anglePid(iangleParams) {}

    ChassisControllerPid(const std::shared_ptr<ChassisModel>& imodel,
      const PidControllerParams& idistanceParams, const PidControllerParams& iangleParams):
      ChassisController(imodel),
      distancePid(idistanceParams),
      anglePid(iangleParams) {}

    ChassisControllerPid(const ChassisModelParams& imodelParams,
      const PidController& idistance, const PidController& iangle):
      ChassisController(imodelParams),
      distancePid(idistance),
      anglePid(iangle) {}

    ChassisControllerPid(const std::shared_ptr<ChassisModel>& imodel,
      const PidController& idistance, const PidController& iangle):
      ChassisController(imodel),
      distancePid(idistance),
      anglePid(iangle) {}

    virtual ~ChassisControllerPid();

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
    PidController distancePid, anglePid;
  };
}

#endif
