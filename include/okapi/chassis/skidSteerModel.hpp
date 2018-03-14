/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#ifndef _OKAPI_SKIDSTEERMODEL_HPP_
#define _OKAPI_SKIDSTEERMODEL_HPP_

#include "okapi/chassis/chassisModel.hpp"
#include "okapi/device/abstractMotor.hpp"
#include "okapi/device/rotarySensor.hpp"

namespace okapi {
class SkidSteerModel;

class SkidSteerModelParams : public ChassisModelParams {
  public:
  SkidSteerModelParams(const AbstractMotor &ileftSide, const AbstractMotor &irightSide,
                       const RotarySensor &ileftEnc, const RotarySensor &irightEnc)
    : leftSide(ileftSide), rightSide(irightSide), leftSensor(ileftEnc), rightSensor(irightEnc) {
  }

  virtual ~SkidSteerModelParams() = default;

  /**
   * Consutructs a new SkidSteerModel.
   *
   * @return const reference to the ChassisModel
   */
  const ChassisModel &make() const override {
    return SkidSteerModel(*this);
  }

  const AbstractMotor &leftSide;
  const AbstractMotor &rightSide;
  const RotarySensor &leftSensor;
  const RotarySensor &rightSensor;
};

class SkidSteerModel : public ChassisModel {
  public:
  /**
   * Model for a skid steer drive (wheels parallel with robot's direction of motion). When all
   * motors are powered +127, the robot should move forward in a straight line.
   *
   * @param imotors   Motors in the format: {left side motors, right side motors}
   *   For example,
   *     {1_m, 2_m}
   *     {1_m, 2_m, 3_m, 4_m}
   * @param ileftEnc  Left side encoder
   * @param irightEnc Right side encoder
   */
  SkidSteerModel(const AbstractMotor &ileftSide, const AbstractMotor &irightSide,
                 const RotarySensor &ileftEnc, const RotarySensor &irightEnc)
    : leftSide(ileftSide), rightSide(irightSide), leftSensor(ileftEnc), rightSensor(irightEnc) {
  }

  SkidSteerModel(const SkidSteerModelParams &iparams)
    : leftSide(iparams.leftSide),
      rightSide(iparams.rightSide),
      leftSensor(iparams.leftSensor),
      rightSensor(iparams.rightSensor) {
  }

  SkidSteerModel(const SkidSteerModel &other)
    : leftSide(other.leftSide),
      rightSide(other.rightSide),
      leftSensor(other.leftSensor),
      rightSensor(other.rightSensor) {
  }

  virtual ~SkidSteerModel() = default;

  void driveForward(const int ipower) const override {
    leftSide.set_velocity(ipower);
    rightSide.set_velocity(ipower);
  }

  void driveVector(const int idistPower, const int ianglePower) const override {
    leftSide.set_velocity(idistPower + ianglePower);
    rightSide.set_velocity(idistPower - ianglePower);
  }

  void turnClockwise(const int ipower) const override {
    leftSide.set_velocity(ipower);
    rightSide.set_velocity(-1 * ipower);
  }

  void stop() const override {
    leftSide.set_velocity(0);
    rightSide.set_velocity(0);
  }

  void tank(const int ileftVal, const int irightVal, const int ithreshold = 0) const override {
    if (fabs(ileftVal) < ithreshold) {
      leftSide.set_velocity(0);
    } else {
      leftSide.set_velocity(ileftVal);
    }

    if (fabs(irightVal) < ithreshold) {
      rightSide.set_velocity(0);
    } else {
      rightSide.set_velocity(irightVal);
    }
  }

  void arcade(int iverticalVal, int ihorizontalVal, const int ithreshold = 0) const override {
    if (fabs(iverticalVal) < ithreshold)
      iverticalVal = 0;
    if (fabs(ihorizontalVal) < ithreshold)
      ihorizontalVal = 0;

    leftSide.set_velocity(iverticalVal + ihorizontalVal);
    rightSide.set_velocity(iverticalVal - ihorizontalVal);
  }

  void left(const int ipower) const override {
    leftSide.set_velocity(ipower);
  }

  void right(const int ipower) const override {
    rightSide.set_velocity(ipower);
  }

  std::valarray<int> getSensorVals() const override {
    return std::valarray<int>{leftSensor.get(), rightSensor.get()};
  }

  void resetSensors() const override {
    leftSensor.reset();
    rightSensor.reset();
  }

  private:
  const AbstractMotor &leftSide;
  const AbstractMotor &rightSide;
  const RotarySensor &leftSensor;
  const RotarySensor &rightSensor;
};
} // namespace okapi

#endif
