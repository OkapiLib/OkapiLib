/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_CHASSISCONTROLLER_HPP_
#define _OKAPI_CHASSISCONTROLLER_HPP_

#include "okapi/chassis/model/chassisModel.hpp"
#include <valarray>

namespace okapi {
class ChassisController {
  public:
  /**
   * A ChassisController adds a closed-loop layer on top of a ChassisModel. moveDistance and
   * turnAngle both use closed-loop control to move the robot. There are passthrough functions for
   * everything defined in ChassisModel.
   *
   * @param imodel underlying ChassisModel
   */
  ChassisController(const ChassisModel &imodel);

  virtual ~ChassisController();

  /**
   * Drives the robot straight for a distance (using closed-loop control).
   *
   * @param itarget distance to travel
   */
  virtual void moveDistance(const int itarget) = 0;

  /**
   * Turns the robot clockwise in place (using closed-loop control).
   *
   * @param idegTarget angle to turn for
   */
  virtual void turnAngle(const float idegTarget) = 0;

  /**
   * Drive the robot forwards (using open-loop control).
   *
   * @param ipower motor power
   */
  virtual void forward(const int ipower) const;

  /**
   * Drive the robot in an arc (using open-loop control).
   * The algorithm is:
   *   leftPower = distPower + anglePower
   *   rightPower = distPower - anglePower
   *
   * @param idistPower see above
   * @param ianglePower see above
   */
  virtual void driveVector(const int idistPower, const int ianglePower) const;

  /**
   * Turn the robot clockwise (using open-loop control).
   *
   * @param ipower motor power
   */
  virtual void rotate(const int ipower) const;

  /**
   * Stop the robot (set all the motors to 0).
   */
  virtual void stop() const;

  /**
   * Drive the robot with a tank drive layout.
   *
   * @param ileftVal left joystick value
   * @param irightVal right joystick value
   * @param ithreshold deadband on joystick values
   */
  virtual void tank(const int ileftVal, const int irightVal, const int ithreshold = 0) const;

  /**
   * Drive the robot with an arcade drive layout.
   *
   * @param iverticalVal vertical joystick value
   * @param ihorizontalVal horizontal joystick value
   * @param ithreshold deadband on joystick values
   */
  virtual void arcade(int iverticalVal, int ihorizontalVal, const int ithreshold = 0) const;

  /**
   * Power the left side motors.
   *
   * @param ipower motor power
   */
  virtual void left(const int ipower) const;

  /**
   * Power the right side motors.
   *
   * @param ipower motor power
   */
  virtual void right(const int ipower) const;

  /**
   * Read the sensors.
   *
   * @return sensor readings in the format {left, right}
   */
  virtual std::valarray<int> getSensorVals() const;

  /**
   * Reset the sensors to their zero point.
   */
  virtual void resetSensors() const;

  protected:
  const ChassisModel &model;
};
} // namespace okapi

#endif
