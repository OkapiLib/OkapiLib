/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#ifndef _OKAPI_XDRIVEMODEL_HPP_
#define _OKAPI_XDRIVEMODEL_HPP_

#include "okapi/chassis/chassisModel.hpp"
#include "okapi/device/abstractMotor.hpp"
#include "okapi/device/rotarySensor.hpp"

namespace okapi {
class XDriveModel;

class XDriveModelParams : public ChassisModelParams {
  public:
  XDriveModelParams(const AbstractMotor &itopLeftMotor, const AbstractMotor &itopRightMotor,
                    const AbstractMotor &ibottomRightMotor, const AbstractMotor &ibottomLeftMotor,
                    const RotarySensor &ileftEnc, const RotarySensor &irightEnc)
    : topLeftMotor(itopLeftMotor),
      topRightMotor(itopRightMotor),
      bottomRightMotor(ibottomRightMotor),
      bottomLeftMotor(ibottomLeftMotor),
      leftSensor(ileftEnc),
      rightSensor(irightEnc) {
  }

  virtual ~XDriveModelParams() {
  }

  /**
   * Consutructs a new XDriveModel.
   *
   * @return const reference to the ChassisModel
   */
  const ChassisModel &make() const override {
    return XDriveModel(*this);
  }

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
              const RotarySensor &ileftEnc, const RotarySensor &irightEnc)
    : topLeftMotor(itopLeftMotor),
      topRightMotor(itopRightMotor),
      bottomRightMotor(ibottomRightMotor),
      bottomLeftMotor(ibottomLeftMotor),
      leftSensor(ileftEnc),
      rightSensor(irightEnc) {
  }

  XDriveModel(const XDriveModelParams &iparams)
    : topLeftMotor(iparams.topLeftMotor),
      topRightMotor(iparams.topRightMotor),
      bottomRightMotor(iparams.bottomRightMotor),
      bottomLeftMotor(iparams.bottomLeftMotor),
      leftSensor(iparams.leftSensor),
      rightSensor(iparams.rightSensor) {
  }

  XDriveModel(const XDriveModel &other)
    : topLeftMotor(other.topLeftMotor),
      topRightMotor(other.topRightMotor),
      bottomRightMotor(other.bottomRightMotor),
      bottomLeftMotor(other.bottomLeftMotor),
      leftSensor(other.leftSensor),
      rightSensor(other.rightSensor) {
  }

  virtual ~XDriveModel() = default;

  void driveForward(const int ipower) const override {
    topLeftMotor.set_velocity(ipower);
    topRightMotor.set_velocity(ipower);
    bottomRightMotor.set_velocity(ipower);
    bottomLeftMotor.set_velocity(ipower);
  }

  void driveVector(const int idistPower, const int ianglePower) const override {
    topLeftMotor.set_velocity(idistPower + ianglePower);
    topRightMotor.set_velocity(idistPower - ianglePower);
    bottomRightMotor.set_velocity(idistPower - ianglePower);
    bottomLeftMotor.set_velocity(idistPower + ianglePower);
  }

  void turnClockwise(const int ipower) const override {
    topLeftMotor.set_velocity(ipower);
    topRightMotor.set_velocity(-1 * ipower);
    bottomRightMotor.set_velocity(-1 * ipower);
    bottomLeftMotor.set_velocity(ipower);
  }

  void stop() const override {
    topLeftMotor.set_velocity(0);
    topRightMotor.set_velocity(0);
    bottomRightMotor.set_velocity(0);
    bottomLeftMotor.set_velocity(0);
  }

  void tank(const int ileftVal, const int irightVal, const int ithreshold = 0) const override {
    if (fabs(ileftVal) < ithreshold) {
      topLeftMotor.set_velocity(0);
      bottomLeftMotor.set_velocity(0);
    } else {
      topLeftMotor.set_velocity(ileftVal);
      bottomLeftMotor.set_velocity(ileftVal);
    }

    if (fabs(irightVal) < ithreshold) {
      topRightMotor.set_velocity(0);
      bottomRightMotor.set_velocity(0);
    } else {
      topRightMotor.set_velocity(irightVal);
      bottomRightMotor.set_velocity(irightVal);
    }
  }

  void arcade(int iverticalVal, int ihorizontalVal, const int ithreshold = 0) const override {
    if (fabs(iverticalVal) < ithreshold)
      iverticalVal = 0;
    if (fabs(ihorizontalVal) < ithreshold)
      ihorizontalVal = 0;

    topLeftMotor.set_velocity(iverticalVal + ihorizontalVal);
    topRightMotor.set_velocity(iverticalVal - ihorizontalVal);
    bottomRightMotor.set_velocity(iverticalVal - ihorizontalVal);
    bottomLeftMotor.set_velocity(iverticalVal + ihorizontalVal);
  }

  void xArcade(int iverticalVal, int ihorizontalVal, int irotateVal,
               const int ithreshold = 0) const {
    if (fabs(iverticalVal) < ithreshold)
      iverticalVal = 0;
    if (fabs(ihorizontalVal) < ithreshold)
      ihorizontalVal = 0;
    if (fabs(irotateVal) < ithreshold)
      irotateVal = 0;

    topLeftMotor.set_velocity(iverticalVal + ihorizontalVal + irotateVal);
    topRightMotor.set_velocity(iverticalVal - ihorizontalVal - irotateVal);
    bottomRightMotor.set_velocity(iverticalVal + ihorizontalVal - irotateVal);
    bottomLeftMotor.set_velocity(iverticalVal - ihorizontalVal + irotateVal);
  }

  void left(const int ipower) const override {
    topLeftMotor.set_velocity(ipower);
    bottomLeftMotor.set_velocity(ipower);
  }

  void right(const int ipower) const override {
    topRightMotor.set_velocity(ipower);
    bottomRightMotor.set_velocity(ipower);
  }

  std::valarray<int> getSensorVals() const override {
    return std::valarray<int>{leftSensor.get(), rightSensor.get()};
  }

  void resetSensors() const override {
    leftSensor.reset();
    rightSensor.reset();
  }

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
