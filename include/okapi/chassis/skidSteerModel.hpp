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
  const ChassisModel &make() const override;

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

  void driveForward(const int ipower) const override;

  void driveVector(const int idistPower, const int ianglePower) const override;

  void turnClockwise(const int ipower) const override;

  void stop() const override;

  void tank(const int ileftVal, const int irightVal, const int ithreshold = 0) const override;

  void arcade(int iverticalVal, int ihorizontalVal, const int ithreshold = 0) const override;

  void left(const int ipower) const override;

  void right(const int ipower) const override;

  std::valarray<int> getSensorVals() const override;

  void resetSensors() const override;

  private:
  const AbstractMotor &leftSideMotor;
  const AbstractMotor &rightSideMotor;
  const RotarySensor &leftSensor;
  const RotarySensor &rightSensor;
};
} // namespace okapi

#endif
