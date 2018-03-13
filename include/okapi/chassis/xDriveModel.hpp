/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#ifndef _OKAPI_XDRIVEMODEL_HPP_
#define _OKAPI_XDRIVEMODEL_HPP_

#include "okapi/chassis/chassisModel.hpp"

namespace okapi {
template <size_t motorsPerCorner> class XDriveModel;

template <size_t motorsPerCorner> class XDriveModelParams : public ChassisModelParams {
  public:
  XDriveModelParams(const std::array<pros::Motor, motorsPerCorner * 4> &imotorList,
                    const RotarySensor &ileftEnc, const RotarySensor &irightEnc)
    : motorList(imotorList), leftSensor(ileftEnc), rightSensor(irightEnc) {
  }

  virtual ~XDriveModelParams() {
  }

  /**
   * Consutructs a new XDriveModel.
   *
   * @return const reference to the ChassisModel
   */
  const ChassisModel &make() const override {
    return XDriveModel<motorsPerCorner>(*this);
  }

  const std::array<pros::Motor, motorsPerCorner * 4> &motorList;
  const RotarySensor &leftSensor;
  const RotarySensor &rightSensor;
};

template <size_t motorsPerCorner> class XDriveModel : public ChassisModel {
  public:
  /**
   * Model for an x drive (wheels at 45 deg from a skid steer drive). When all motors are powered
   * +127, the robot should move forward in a straight line.
   *
   * @param imotors Motors in the format:
   *  {top left motors, top right motors, bottom right motors, bottom left motors}
   *  For example,
   *    {1_m, 2_m, 3_m, 4_m}
   *    {1_m, 2_m, 3_m, 4_m, 5_m, 6_m, 7_m, 8_m}
   * @param ileftEnc Left side encoder
   * @param irightEnc Right side encoder
   */
  XDriveModel(const std::array<pros::Motor, motorsPerCorner * 4> &imotorList,
              const RotarySensor &ileftEnc, const RotarySensor &irightEnc)
    : motors(imotorList), leftSensor(ileftEnc), rightSensor(irightEnc) {
  }

  XDriveModel(const XDriveModelParams<motorsPerCorner> &iparams)
    : motors(iparams.motorList), leftSensor(iparams.leftSensor), rightSensor(iparams.rightSensor) {
  }

  XDriveModel(const XDriveModel<motorsPerCorner> &other)
    : motors(other.motors), leftSensor(other.leftSensor), rightSensor(other.rightSensor) {
  }

  virtual ~XDriveModel() {
    delete &motors;
  }

  void driveForward(const int ipower) const override {
    for (size_t i = 0; i < motorsPerCorner * 4; i++)
      motors[i].set_velocity(ipower);
  }

  void driveVector(const int idistPower, const int ianglePower) const override {
    for (size_t i = 0; i < motorsPerCorner; i++)
      motors[i].set_velocity(idistPower + ianglePower);
    for (size_t i = motorsPerCorner; i < motorsPerCorner * 2; i++)
      motors[i].set_velocity(idistPower - ianglePower);
    for (size_t i = motorsPerCorner * 2; i < motorsPerCorner * 3; i++)
      motors[i].set_velocity(idistPower - ianglePower);
    for (size_t i = motorsPerCorner * 3; i < motorsPerCorner * 4; i++)
      motors[i].set_velocity(idistPower + ianglePower);
  }

  void turnClockwise(const int ipower) const override {
    for (size_t i = 0; i < motorsPerCorner; i++)
      motors[i].set_velocity(ipower);
    for (size_t i = motorsPerCorner; i < motorsPerCorner * 2; i++)
      motors[i].set_velocity(-1 * ipower);
    for (size_t i = motorsPerCorner * 2; i < motorsPerCorner * 3; i++)
      motors[i].set_velocity(-1 * ipower);
    for (size_t i = motorsPerCorner * 3; i < motorsPerCorner * 4; i++)
      motors[i].set_velocity(ipower);
  }

  void stop() const override {
    for (size_t i = 0; i < motorsPerCorner * 4; i++)
      motors[i].set_velocity(0);
  }

  void tank(const int ileftVal, const int irightVal, const int ithreshold = 0) const override {
    if (fabs(ileftVal) < ithreshold) {
      for (size_t i = 0; i < motorsPerCorner; i++)
        motors[i].set_velocity(0);
      for (size_t i = motorsPerCorner * 3; i < motorsPerCorner * 4; i++)
        motors[i].set_velocity(0);
    } else {
      for (size_t i = 0; i < motorsPerCorner; i++)
        motors[i].set_velocity(ileftVal);
      for (size_t i = motorsPerCorner * 3; i < motorsPerCorner * 4; i++)
        motors[i].set_velocity(ileftVal);
    }

    if (fabs(irightVal) < ithreshold) {
      for (size_t i = motorsPerCorner; i < motorsPerCorner * 2; i++)
        motors[i].set_velocity(0);
      for (size_t i = motorsPerCorner * 2; i < motorsPerCorner * 3; i++)
        motors[i].set_velocity(0);
    } else {
      for (size_t i = motorsPerCorner; i < motorsPerCorner * 2; i++)
        motors[i].set_velocity(irightVal);
      for (size_t i = motorsPerCorner * 2; i < motorsPerCorner * 3; i++)
        motors[i].set_velocity(irightVal);
    }
  }

  void arcade(int iverticalVal, int ihorizontalVal, const int ithreshold = 0) const override {
    if (fabs(iverticalVal) < ithreshold)
      iverticalVal = 0;
    if (fabs(ihorizontalVal) < ithreshold)
      ihorizontalVal = 0;

    for (size_t i = 0; i < motorsPerCorner; i++)
      motors[i].set_velocity(iverticalVal + ihorizontalVal);
    for (size_t i = motorsPerCorner; i < motorsPerCorner * 2; i++)
      motors[i].set_velocity(iverticalVal - ihorizontalVal);
    for (size_t i = motorsPerCorner * 2; i < motorsPerCorner * 3; i++)
      motors[i].set_velocity(iverticalVal - ihorizontalVal);
    for (size_t i = motorsPerCorner * 3; i < motorsPerCorner * 4; i++)
      motors[i].set_velocity(iverticalVal + ihorizontalVal);
  }

  void xArcade(int iverticalVal, int ihorizontalVal, int irotateVal,
               const int ithreshold = 0) const {
    if (fabs(iverticalVal) < ithreshold)
      iverticalVal = 0;
    if (fabs(ihorizontalVal) < ithreshold)
      ihorizontalVal = 0;
    if (fabs(irotateVal) < ithreshold)
      irotateVal = 0;

    for (size_t i = 0; i < motorsPerCorner; i++)
      motors[i].set_velocity(iverticalVal + ihorizontalVal + irotateVal);
    for (size_t i = motorsPerCorner; i < motorsPerCorner * 2; i++)
      motors[i].set_velocity(iverticalVal - ihorizontalVal - irotateVal);
    for (size_t i = motorsPerCorner * 2; i < motorsPerCorner * 3; i++)
      motors[i].set_velocity(iverticalVal + ihorizontalVal - irotateVal);
    for (size_t i = motorsPerCorner * 3; i < motorsPerCorner * 4; i++)
      motors[i].set_velocity(iverticalVal - ihorizontalVal + irotateVal);
  }

  void left(const int ipower) const override {
    for (size_t i = 0; i < motorsPerCorner; i++)
      motors[i].set_velocity(ipower);
    for (size_t i = motorsPerCorner * 3; i < motorsPerCorner * 4; i++)
      motors[i].set_velocity(ipower);
  }

  void right(const int ipower) const override {
    for (size_t i = motorsPerCorner; i < motorsPerCorner * 3; i++)
      motors[i].set_velocity(ipower);
  }

  std::valarray<int> getSensorVals() const override {
    return std::valarray<int>{leftSensor.get(), rightSensor.get()};
  }

  void resetSensors() const override {
    leftSensor.reset();
    rightSensor.reset();
  }

  private:
  std::array<pros::Motor, motorsPerCorner * 4> motors;
  const RotarySensor &leftSensor;
  const RotarySensor &rightSensor;
};
}

#endif
