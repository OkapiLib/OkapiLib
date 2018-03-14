/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#include "okapi/chassis/xDriveModel.hpp"

namespace okapi {
XDriveModelParams::XDriveModelParams(const AbstractMotor &itopLeftMotor,
                                     const AbstractMotor &itopRightMotor,
                                     const AbstractMotor &ibottomRightMotor,
                                     const AbstractMotor &ibottomLeftMotor,
                                     const RotarySensor &ileftEnc, const RotarySensor &irightEnc)
  : topLeftMotor(itopLeftMotor),
    topRightMotor(itopRightMotor),
    bottomRightMotor(ibottomRightMotor),
    bottomLeftMotor(ibottomLeftMotor),
    leftSensor(ileftEnc),
    rightSensor(irightEnc) {
}

XDriveModelParams::~XDriveModelParams() = default;

std::shared_ptr<const ChassisModel> XDriveModelParams::make() const {
  return std::make_shared<const XDriveModel>(XDriveModel(*this));
}

XDriveModel::XDriveModel(const AbstractMotor &itopLeftMotor, const AbstractMotor &itopRightMotor,
                         const AbstractMotor &ibottomRightMotor,
                         const AbstractMotor &ibottomLeftMotor, const RotarySensor &ileftEnc,
                         const RotarySensor &irightEnc)
  : topLeftMotor(itopLeftMotor),
    topRightMotor(itopRightMotor),
    bottomRightMotor(ibottomRightMotor),
    bottomLeftMotor(ibottomLeftMotor),
    leftSensor(ileftEnc),
    rightSensor(irightEnc) {
}

XDriveModel::XDriveModel(const XDriveModelParams &iparams)
  : topLeftMotor(iparams.topLeftMotor),
    topRightMotor(iparams.topRightMotor),
    bottomRightMotor(iparams.bottomRightMotor),
    bottomLeftMotor(iparams.bottomLeftMotor),
    leftSensor(iparams.leftSensor),
    rightSensor(iparams.rightSensor) {
}

XDriveModel::XDriveModel(const XDriveModel &other)
  : topLeftMotor(other.topLeftMotor),
    topRightMotor(other.topRightMotor),
    bottomRightMotor(other.bottomRightMotor),
    bottomLeftMotor(other.bottomLeftMotor),
    leftSensor(other.leftSensor),
    rightSensor(other.rightSensor) {
}

XDriveModel::~XDriveModel() = default;

void XDriveModel::driveForward(const int ipower) const {
  topLeftMotor.set_velocity(ipower);
  topRightMotor.set_velocity(ipower);
  bottomRightMotor.set_velocity(ipower);
  bottomLeftMotor.set_velocity(ipower);
}

void XDriveModel::driveVector(const int idistPower, const int ianglePower) const {
  topLeftMotor.set_velocity(idistPower + ianglePower);
  topRightMotor.set_velocity(idistPower - ianglePower);
  bottomRightMotor.set_velocity(idistPower - ianglePower);
  bottomLeftMotor.set_velocity(idistPower + ianglePower);
}

void XDriveModel::turnClockwise(const int ipower) const {
  topLeftMotor.set_velocity(ipower);
  topRightMotor.set_velocity(-1 * ipower);
  bottomRightMotor.set_velocity(-1 * ipower);
  bottomLeftMotor.set_velocity(ipower);
}

void XDriveModel::stop() const {
  topLeftMotor.set_velocity(0);
  topRightMotor.set_velocity(0);
  bottomRightMotor.set_velocity(0);
  bottomLeftMotor.set_velocity(0);
}

void XDriveModel::tank(const int ileftVal, const int irightVal, const int ithreshold) const {
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

void XDriveModel::arcade(int iverticalVal, int ihorizontalVal, const int ithreshold) const {
  if (fabs(iverticalVal) < ithreshold)
    iverticalVal = 0;
  if (fabs(ihorizontalVal) < ithreshold)
    ihorizontalVal = 0;

  topLeftMotor.set_velocity(iverticalVal + ihorizontalVal);
  topRightMotor.set_velocity(iverticalVal - ihorizontalVal);
  bottomRightMotor.set_velocity(iverticalVal - ihorizontalVal);
  bottomLeftMotor.set_velocity(iverticalVal + ihorizontalVal);
}

void XDriveModel::xArcade(int iverticalVal, int ihorizontalVal, int irotateVal,
                          const int ithreshold) const {
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

void XDriveModel::left(const int ipower) const {
  topLeftMotor.set_velocity(ipower);
  bottomLeftMotor.set_velocity(ipower);
}

void XDriveModel::right(const int ipower) const {
  topRightMotor.set_velocity(ipower);
  bottomRightMotor.set_velocity(ipower);
}

std::valarray<int> XDriveModel::getSensorVals() const {
  return std::valarray<int>{leftSensor.get(), rightSensor.get()};
}

void XDriveModel::resetSensors() const {
  leftSensor.reset();
  rightSensor.reset();
}
} // namespace okapi