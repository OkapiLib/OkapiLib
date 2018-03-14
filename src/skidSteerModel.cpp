/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#include "okapi/chassis/skidSteerModel.hpp"

namespace okapi {
SkidSteerModelParams::SkidSteerModelParams(const AbstractMotor &ileftSideMotor,
                                           const AbstractMotor &irightSideMotor,
                                           const RotarySensor &ileftEnc,
                                           const RotarySensor &irightEnc)
  : leftSideMotor(ileftSideMotor),
    rightSideMotor(irightSideMotor),
    leftSensor(ileftEnc),
    rightSensor(irightEnc) {
}

SkidSteerModelParams::~SkidSteerModelParams() = default;

std::shared_ptr<const ChassisModel> SkidSteerModelParams::make() const {
  return std::make_shared<const SkidSteerModel>(SkidSteerModel(*this));
}

SkidSteerModel::SkidSteerModel(const AbstractMotor &ileftSideMotor,
                               const AbstractMotor &irightSideMotor, const RotarySensor &ileftEnc,
                               const RotarySensor &irightEnc)
  : leftSideMotor(ileftSideMotor),
    rightSideMotor(irightSideMotor),
    leftSensor(ileftEnc),
    rightSensor(irightEnc) {
}

SkidSteerModel::SkidSteerModel(const SkidSteerModelParams &iparams)
  : leftSideMotor(iparams.leftSideMotor),
    rightSideMotor(iparams.rightSideMotor),
    leftSensor(iparams.leftSensor),
    rightSensor(iparams.rightSensor) {
}

SkidSteerModel::SkidSteerModel(const SkidSteerModel &other)
  : leftSideMotor(other.leftSideMotor),
    rightSideMotor(other.rightSideMotor),
    leftSensor(other.leftSensor),
    rightSensor(other.rightSensor) {
}

SkidSteerModel::~SkidSteerModel() = default;

void SkidSteerModel::driveForward(const int ipower) const {
  leftSideMotor.set_velocity(ipower);
  rightSideMotor.set_velocity(ipower);
}

void SkidSteerModel::driveVector(const int idistPower, const int ianglePower) const {
  leftSideMotor.set_velocity(idistPower + ianglePower);
  rightSideMotor.set_velocity(idistPower - ianglePower);
}

void SkidSteerModel::turnClockwise(const int ipower) const {
  leftSideMotor.set_velocity(ipower);
  rightSideMotor.set_velocity(-1 * ipower);
}

void SkidSteerModel::stop() const {
  leftSideMotor.set_velocity(0);
  rightSideMotor.set_velocity(0);
}

void SkidSteerModel::tank(const int ileftVal, const int irightVal, const int ithreshold) const {
  if (fabs(ileftVal) < ithreshold) {
    leftSideMotor.set_velocity(0);
  } else {
    leftSideMotor.set_velocity(ileftVal);
  }

  if (fabs(irightVal) < ithreshold) {
    rightSideMotor.set_velocity(0);
  } else {
    rightSideMotor.set_velocity(irightVal);
  }
}

void SkidSteerModel::arcade(int iverticalVal, int ihorizontalVal, const int ithreshold) const {
  if (fabs(iverticalVal) < ithreshold)
    iverticalVal = 0;
  if (fabs(ihorizontalVal) < ithreshold)
    ihorizontalVal = 0;

  leftSideMotor.set_velocity(iverticalVal + ihorizontalVal);
  rightSideMotor.set_velocity(iverticalVal - ihorizontalVal);
}

void SkidSteerModel::left(const int ipower) const {
  leftSideMotor.set_velocity(ipower);
}

void SkidSteerModel::right(const int ipower) const {
  rightSideMotor.set_velocity(ipower);
}

std::valarray<int> SkidSteerModel::getSensorVals() const {
  return std::valarray<int>{leftSensor.get(), rightSensor.get()};
}

void SkidSteerModel::resetSensors() const {
  leftSensor.reset();
  rightSensor.reset();
}
} // namespace okapi