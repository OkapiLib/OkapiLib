/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#ifndef _OKAPI_SKIDSTEERMODEL_HPP_
#define _OKAPI_SKIDSTEERMODEL_HPP_

#include "okapi/chassis/chassisModel.hpp"
#include "okapi/device/abstractMotor.hpp"
#include "okapi/device/rotarySensor.hpp"

namespace okapi {
class SkidSteerModelParams : public ChassisModelParams {
  public:
  SkidSteerModelParams(const AbstractMotor &ileftSideMotor, const AbstractMotor &irightSideMotor,
                       const RotarySensor &ileftEnc, const RotarySensor &irightEnc);

  virtual ~SkidSteerModelParams();

  /**
   * Consutructs a new SkidSteerModel.
   *
   * @return const reference to the ChassisModel
   */
  std::shared_ptr<const ChassisModel> make() const override;

  const AbstractMotor &leftSideMotor;
  const AbstractMotor &rightSideMotor;
  const RotarySensor &leftSensor;
  const RotarySensor &rightSensor;
};

class SkidSteerModel : public ChassisModel {
  public:
  /**
   * Model for a skid steer drive (wheels parallel with robot's direction of motion). When all
   * motors are powered +127, the robot should move forward in a straight line.
   *
   * @param ileftSideMotor left side motor
   * @param irightSideMotor right side motor
   * @param ileftEnc  Left side encoder
   * @param irightEnc Right side encoder
   */
  SkidSteerModel(const AbstractMotor &ileftSideMotor, const AbstractMotor &irightSideMotor,
                 const RotarySensor &ileftEnc, const RotarySensor &irightEnc);

  SkidSteerModel(const SkidSteerModelParams &iparams);

  SkidSteerModel(const SkidSteerModel &other);

  virtual ~SkidSteerModel();

  /**
   * Drive the robot forwards (using open-loop control).
   *
   * @param ipower motor power
   */
  void driveForward(const int ipower) const override;

  /**
   * Drive the robot in an arc (using open-loop control).
   * The algorithm is:
   *   leftPower = distPower + anglePower
   *   rightPower = distPower - anglePower
   *
   * @param idistPower see above
   * @param ianglePower see above
   */
  void driveVector(const int idistPower, const int ianglePower) const override;

  /**
   * Turn the robot clockwise (using open-loop control).
   *
   * @param ipower motor power
   */
  void turnClockwise(const int ipower) const override;

  /**
   * Stop the robot (set all the motors to 0).
   */
  void stop() const override;

  /**
   * Drive the robot with a tank drive layout.
   *
   * @param ileftVal left joystick value
   * @param irightVal right joystick value
   * @param ithreshold deadband on joystick values
   */
  void tank(const int ileftVal, const int irightVal, const int ithreshold = 0) const override;

  /**
   * Drive the robot with an arcade drive layout.
   *
   * @param iverticalVal vertical joystick value
   * @param ihorizontalVal horizontal joystick value
   * @param ithreshold deadband on joystick values
   */
  void arcade(int iverticalVal, int ihorizontalVal, const int ithreshold = 0) const override;

  /**
   * Power the left side motors.
   *
   * @param ipower motor power
   */
  void left(const int ipower) const override;

  /**
   * Power the right side motors.
   *
   * @param ipower motor power
   */
  void right(const int ipower) const override;

  /**
   * Read the sensors.
   *
   * @return sensor readings in the format {left, right}
   */
  std::valarray<int> getSensorVals() const override;

  /**
   * Reset the sensors to their zero point.
   */
  void resetSensors() const override;

  private:
  const AbstractMotor &leftSideMotor;
  const AbstractMotor &rightSideMotor;
  const RotarySensor &leftSensor;
  const RotarySensor &rightSensor;
};
} // namespace okapi

#endif
