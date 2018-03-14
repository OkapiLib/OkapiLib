/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#ifndef _OKAPI_CHASSISMODEL_HPP_
#define _OKAPI_CHASSISMODEL_HPP_

#include <array>
#include <initializer_list>
#include <memory>
#include <valarray>

namespace okapi {
class ChassisModel;

class ChassisModelParams {
  public:
  virtual ~ChassisModelParams() = default;

  /**
   * Consutructs a new ChassisModel.
   *
   * @return const reference to the ChassisModel
   */
  virtual std::unique_ptr<ChassisModel> make() const = 0;
};

class ChassisModel {
  public:
  virtual ~ChassisModel();

  /**
   * Drive the robot forwards (using open-loop control).
   *
   * @param ipower motor power
   */
  virtual void driveForward(const int ipower) const = 0;

  /**
   * Drive the robot in an arc (using open-loop control).
   * The algorithm is:
   *   leftPower = distPower + anglePower
   *   rightPower = distPower - anglePower
   *
   * @param idistPower see above
   * @param ianglePower see above
   */
  virtual void driveVector(const int idistPower, const int ianglePower) const = 0;

  /**
   * Turn the robot clockwise (using open-loop control).
   *
   * @param ipower motor power
   */
  virtual void turnClockwise(const int ipower) const = 0;

  /**
   * Stop the robot (set all the motors to 0).
   */
  virtual void stop() const = 0;

  /**
   * Drive the robot with a tank drive layout.
   *
   * @param ileftVal left joystick value
   * @param irightVal right joystick value
   * @param ithreshold deadband on joystick values
   */
  virtual void tank(const int ileftVal, const int irightVal, const int ithreshold = 0) const = 0;

  /**
   * Drive the robot with an arcade drive layout.
   *
   * @param iverticalVal vertical joystick value
   * @param ihorizontalVal horizontal joystick value
   * @param ithreshold deadband on joystick values
   */
  virtual void arcade(int iverticalVal, int ihorizontalVal, const int ithreshold = 0) const = 0;

  /**
   * Power the left side motors.
   *
   * @param ipower motor power
   */
  virtual void left(const int ipower) const = 0;

  /**
   * Power the right side motors.
   *
   * @param ipower motor power
   */
  virtual void right(const int ipower) const = 0;

  /**
   * Read the sensors.
   *
   * @return sensor readings (format is implementation dependent)
   */
  virtual std::valarray<int> getSensorVals() const = 0;

  /**
   * Reset the sensors to their zero point.
   */
  virtual void resetSensors() const = 0;
};
} // namespace okapi

#endif
