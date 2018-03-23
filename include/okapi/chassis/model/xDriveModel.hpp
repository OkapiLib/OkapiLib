/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_XDRIVEMODEL_HPP_
#define _OKAPI_XDRIVEMODEL_HPP_

#include "okapi/chassis/model/chassisModel.hpp"
#include "okapi/device/abstractMotor.hpp"
#include "okapi/device/rotarySensor.hpp"

namespace okapi {
class XDriveModelParams : public ChassisModelParams {
  public:
  XDriveModelParams(const AbstractMotor &itopLeftMotor, const AbstractMotor &itopRightMotor,
                    const AbstractMotor &ibottomRightMotor, const AbstractMotor &ibottomLeftMotor,
                    const RotarySensor &ileftEnc, const RotarySensor &irightEnc);

  XDriveModelParams(const AbstractMotor &itopLeftMotor, const AbstractMotor &itopRightMotor,
                    const AbstractMotor &ibottomRightMotor, const AbstractMotor &ibottomLeftMotor);

  virtual ~XDriveModelParams();

  /**
   * Constructs a new XDriveModel.
   *
   * @return const reference to the ChassisModel
   */
  std::shared_ptr<const ChassisModel> make() const override;

  const AbstractMotor &topLeftMotor;
  const AbstractMotor &topRightMotor;
  const AbstractMotor &bottomRightMotor;
  const AbstractMotor &bottomLeftMotor;
  const RotarySensor &leftSensor;
  const RotarySensor &rightSensor;
};

class XDriveModel : public ChassisModel {
  public:
  /**
   * Model for an x drive (wheels at 45 deg from a skid steer drive). When all motors are powered
   * +127, the robot should move forward in a straight line.
   *
   * @param itopLeftMotor top left motor
   * @param itopRightMotor top right motor
   * @param ibottomRightMotor bottom right motor
   * @param ibottomLeftMotor bottom left motor
   * @param ileftEnc Left side encoder
   * @param irightEnc Right side encoder
   */
  XDriveModel(const AbstractMotor &itopLeftMotor, const AbstractMotor &itopRightMotor,
              const AbstractMotor &ibottomRightMotor, const AbstractMotor &ibottomLeftMotor,
              const RotarySensor &ileftEnc, const RotarySensor &irightEnc);

  /**
   * Model for an x drive (wheels at 45 deg from a skid steer drive). When all motors are powered
   * +127, the robot should move forward in a straight line.
   *
   * This constructor infers the two sensors from the top left and top right motors (using the
   * integrated encoders).
   *
   * @param itopLeftMotor top left motor
   * @param itopRightMotor top right motor
   * @param ibottomRightMotor bottom right motor
   * @param ibottomLeftMotor bottom left motor
   */
  XDriveModel(const AbstractMotor &itopLeftMotor, const AbstractMotor &itopRightMotor,
              const AbstractMotor &ibottomRightMotor, const AbstractMotor &ibottomLeftMotor);

  XDriveModel(const XDriveModelParams &iparams);

  XDriveModel(const XDriveModel &other);

  virtual ~XDriveModel();

  /**
   * Drive the robot forwards (using open-loop control).
   *
   * @param ipower motor power
   */
  void forward(const int ipower) const override;

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
  void rotate(const int ipower) const override;

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
   * Drive the robot with an arcade drive layout specially for a holonomic drive. The horizontal
   * value no longer rotates the robot (the rotate value does that); instead, it strafes the robot
   * sideways.
   *
   * @param iverticalVal vertical joystick value
   * @param ihorizontalVal horizontal joystick value
   * @param ithreshold deadband on joystick values
   */
  void xArcade(int iverticalVal, int ihorizontalVal, int irotateVal,
               const int ithreshold = 0) const;

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
  const AbstractMotor &topLeftMotor;
  const AbstractMotor &topRightMotor;
  const AbstractMotor &bottomRightMotor;
  const AbstractMotor &bottomLeftMotor;
  const RotarySensor &leftSensor;
  const RotarySensor &rightSensor;
};
} // namespace okapi

#endif
