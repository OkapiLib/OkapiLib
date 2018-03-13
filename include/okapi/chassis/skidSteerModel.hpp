/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#ifndef _OKAPI_SKIDSTEERMODEL_HPP_
#define _OKAPI_SKIDSTEERMODEL_HPP_

#include "okapi/chassis/chassisModel.hpp"

namespace okapi {
template <size_t motorsPerSide> class SkidSteerModel;

template <size_t motorsPerSide> class SkidSteerModelParams : public ChassisModelParams {
  public:
  SkidSteerModelParams(const std::array<pros::Motor, motorsPerSide * 2> &imotorList,
                       const RotarySensor &ileftEnc, const RotarySensor &irightEnc)
    : motorList(imotorList), leftSensor(ileftEnc), rightSensor(irightEnc) {
  }

  virtual ~SkidSteerModelParams() = default;

  /**
   * Consutructs a new SkidSteerModel.
   *
   * @return const reference to the ChassisModel
   */
  const ChassisModel &make() const override {
    return SkidSteerModel<motorsPerSide>(*this);
  }

  const std::array<pros::Motor, motorsPerSide * 2> &motorList;
  const RotarySensor &leftSensor;
  const RotarySensor &rightSensor;
};

template <size_t motorsPerSide> class SkidSteerModel : public ChassisModel {
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
  SkidSteerModel(const std::array<pros::Motor, motorsPerSide * 2> &imotorList,
                 const RotarySensor &ileftEnc, const RotarySensor &irightEnc)
    : motors(imotorList), leftSensor(ileftEnc), rightSensor(irightEnc) {
  }

  SkidSteerModel(const SkidSteerModelParams<motorsPerSide> &iparams)
    : motors(iparams.motorList), leftSensor(iparams.leftSensor), rightSensor(iparams.rightSensor) {
  }

  SkidSteerModel(const SkidSteerModel<motorsPerSide> &other)
    : motors(other.motors), leftSensor(other.leftSensor), rightSensor(other.rightSensor) {
  }

  virtual ~SkidSteerModel() {
    delete &motors;
  }

  void driveForward(const int ipower) const override {
    for (size_t i = 0; i < motorsPerSide * 2; i++)
      motors[i].set_velocity(ipower);
  }

  void driveVector(const int idistPower, const int ianglePower) const override {
    for (size_t i = 0; i < motorsPerSide; i++)
      motors[i].set_velocity(idistPower + ianglePower);
    for (size_t i = motorsPerSide; i < motorsPerSide * 2; i++)
      motors[i].set_velocity(idistPower - ianglePower);
  }

  void turnClockwise(const int ipower) const override {
    for (size_t i = 0; i < motorsPerSide; i++)
      motors[i].set_velocity(ipower);
    for (size_t i = motorsPerSide; i < motorsPerSide * 2; i++)
      motors[i].set_velocity(-1 * ipower);
  }

  void stop() const override {
    for (size_t i = 0; i < motorsPerSide * 2; i++)
      motors[i].set_velocity(0);
  }

  void tank(const int ileftVal, const int irightVal, const int ithreshold = 0) const override {
    if (fabs(ileftVal) < ithreshold) {
      for (size_t i = 0; i < motorsPerSide; i++)
        motors[i].set_velocity(0);
    } else {
      for (size_t i = 0; i < motorsPerSide; i++)
        motors[i].set_velocity(ileftVal);
    }

    if (fabs(irightVal) < ithreshold) {
      for (size_t i = motorsPerSide; i < motorsPerSide * 2; i++)
        motors[i].set_velocity(0);
    } else {
      for (size_t i = motorsPerSide; i < motorsPerSide * 2; i++)
        motors[i].set_velocity(irightVal);
    }
  }

  void arcade(int iverticalVal, int ihorizontalVal, const int ithreshold = 0) const override {
    if (fabs(iverticalVal) < ithreshold)
      iverticalVal = 0;
    if (fabs(ihorizontalVal) < ithreshold)
      ihorizontalVal = 0;

    for (size_t i = 0; i < motorsPerSide; i++)
      motors[i].set_velocity(iverticalVal + ihorizontalVal);
    for (size_t i = motorsPerSide; i < motorsPerSide * 2; i++)
      motors[i].set_velocity(iverticalVal - ihorizontalVal);
  }

  void left(const int ipower) const override {
    for (size_t i = 0; i < motorsPerSide; i++)
      motors[i].set_velocity(ipower);
  }

  void right(const int ipower) const override {
    for (size_t i = motorsPerSide; i < motorsPerSide * 2; i++)
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
  std::array<pros::Motor, motorsPerSide * 2> motors;
  const RotarySensor &leftSensor;
  const RotarySensor &rightSensor;
};
} // namespace okapi

#endif
