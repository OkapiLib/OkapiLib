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
  ChassisController(const ChassisModelParams &imodelParams) : model(imodelParams.make()) {}

  ChassisController(const ChassisModel &imodel) : model(imodel) {}

  virtual ~ChassisController();

  /**
   * Drives the robot straight (using closed-loop control).
   *
   * @param itarget distance to travel
   */
  virtual void driveStraight(const int itarget) = 0;

  /**
   * Turns the robot clockwise in place (using closed-loop control).
   *
   * @param idegTarget degrees to turn for
   */
  virtual void pointTurn(const float idegTarget) = 0;

  /**
   * Drive the robot forwards (using open-loop control).
   *
   * @param ipower motor power
   */
  void driveForward(const int ipower);

  /**
   * Drive the robot in an arc (using open-loop control).
   * The algorithm is:
   *   leftPower = distPower + anglePower
   *   rightPower = distPower - anglePower
   *
   * @param idistPower see above
   * @param ianglePower see above
   */
  void driveVector(const int idistPower, const int ianglePower);

  /**
   * Turn the robot clockwise (using open-loop control).
   *
   * @param ipower motor power
   */
  void turnClockwise(const int ipower);

  /**
   * Stop the robot (set all the motors to 0).
   */
  void stop();

  /**
   * Drive the robot with a tank drive layout.
   *
   * @param ileftVal left joystick value
   * @param irightVal right joystick value
   * @param ithreshold deadband on joystick values
   */
  void tank(const int ileftVal, const int irightVal, const int ithreshold = 0);

  /**
   * Drive the robot with an arcade drive layout.
   *
   * @param iverticalVal vertical joystick value
   * @param ihorizontalVal horizontal joystick value
   * @param ithreshold deadband on joystick values
   */
  void arcade(int iverticalVal, int ihorizontalVal, const int ithreshold = 0);

  /**
   * Power the left side motors.
   *
   * @param ipower motor power
   */
  void left(const int ipower);

  /**
   * Power the right side motors.
   *
   * @param ipower motor power
   */
  void right(const int ipower);

  /**
   * Read the sensors.
   *
   * @return sensor readings in the format {left, right}
   */
  std::valarray<int> getSensorVals();

  /**
   * Reset the sensors to their zero point.
   */
  void resetSensors() const;

  protected:
  const ChassisModel &model;
};
} // namespace okapi

#endif
