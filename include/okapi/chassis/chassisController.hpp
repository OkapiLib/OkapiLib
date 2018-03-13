/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#ifndef _OKAPI_CHASSISCONTROLLER_HPP_
#define _OKAPI_CHASSISCONTROLLER_HPP_

#include "okapi/chassis/chassisModel.hpp"
#include <valarray>

namespace okapi {
  class ChassisController {
  public:
    ChassisController(const ChassisModelParams& imodelParams):
      model(imodelParams.make()) {}

    ChassisController(const std::shared_ptr<ChassisModel>& imodel):
      model(imodel) {}

    virtual ~ChassisController();

    /**
     * Drives the robot straight.
     * 
     * @param itarget Distance to travel
     */
    virtual void driveStraight(const int itarget) = 0;

    /**
     * Turns the robot clockwise in place.
     * 
     * @param idegTarget Degrees to turn for
     */
    virtual void pointTurn(const float idegTarget) = 0;

    void driveForward(const int power);

    void driveVector(const int distPower, const int anglePower);

    void turnClockwise(const int power);

    void stop();

    void tank(const int leftVal, const int rightVal, const int threshold = 0);

    void arcade(int verticalVal, int horizontalVal, const int threshold = 0);

    void left(const int val);

    void right(const int val);

    std::valarray<int> getSensorVals();

  protected:
    std::shared_ptr<ChassisModel> model;
  };
}

#endif
