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
  topLeftMotor.moveVelocity(ipower);
  topRightMotor.moveVelocity(ipower);
  bottomRightMotor.moveVelocity(ipower);
  bottomLeftMotor.moveVelocity(ipower);
}

void XDriveModel::driveVector(const int idistPower, const int ianglePower) const {
  topLeftMotor.moveVelocity(idistPower + ianglePower);
  topRightMotor.moveVelocity(idistPower - ianglePower);
  bottomRightMotor.moveVelocity(idistPower - ianglePower);
  bottomLeftMotor.moveVelocity(idistPower + ianglePower);
}

void XDriveModel::turnClockwise(const int ipower) const {
  topLeftMotor.moveVelocity(ipower);
  topRightMotor.moveVelocity(-1 * ipower);
  bottomRightMotor.moveVelocity(-1 * ipower);
  bottomLeftMotor.moveVelocity(ipower);
}

void XDriveModel::stop() const {
  topLeftMotor.moveVelocity(0);
  topRightMotor.moveVelocity(0);
  bottomRightMotor.moveVelocity(0);
  bottomLeftMotor.moveVelocity(0);
}

void XDriveModel::tank(const int ileftVal, const int irightVal, const int ithreshold) const {
  if (fabs(ileftVal) < ithreshold) {
    topLeftMotor.moveVelocity(0);
    bottomLeftMotor.moveVelocity(0);
  } else {
    topLeftMotor.moveVelocity(ileftVal);
    bottomLeftMotor.moveVelocity(ileftVal);
  }

  if (fabs(irightVal) < ithreshold) {
    topRightMotor.moveVelocity(0);
    bottomRightMotor.moveVelocity(0);
  } else {
    topRightMotor.moveVelocity(irightVal);
    bottomRightMotor.moveVelocity(irightVal);
  }
}

void XDriveModel::arcade(int iverticalVal, int ihorizontalVal, const int ithreshold) const {
  if (fabs(iverticalVal) < ithreshold)
    iverticalVal = 0;
  if (fabs(ihorizontalVal) < ithreshold)
    ihorizontalVal = 0;

  topLeftMotor.moveVelocity(iverticalVal + ihorizontalVal);
  topRightMotor.moveVelocity(iverticalVal - ihorizontalVal);
  bottomRightMotor.moveVelocity(iverticalVal - ihorizontalVal);
  bottomLeftMotor.moveVelocity(iverticalVal + ihorizontalVal);
}

void XDriveModel::xArcade(int iverticalVal, int ihorizontalVal, int irotateVal,
                          const int ithreshold) const {
  if (fabs(iverticalVal) < ithreshold)
    iverticalVal = 0;
  if (fabs(ihorizontalVal) < ithreshold)
    ihorizontalVal = 0;
  if (fabs(irotateVal) < ithreshold)
    irotateVal = 0;

  topLeftMotor.moveVelocity(iverticalVal + ihorizontalVal + irotateVal);
  topRightMotor.moveVelocity(iverticalVal - ihorizontalVal - irotateVal);
  bottomRightMotor.moveVelocity(iverticalVal + ihorizontalVal - irotateVal);
  bottomLeftMotor.moveVelocity(iverticalVal - ihorizontalVal + irotateVal);
}

void XDriveModel::left(const int ipower) const {
  topLeftMotor.moveVelocity(ipower);
  bottomLeftMotor.moveVelocity(ipower);
}

void XDriveModel::right(const int ipower) const {
  topRightMotor.moveVelocity(ipower);
  bottomRightMotor.moveVelocity(ipower);
}

std::valarray<int> XDriveModel::getSensorVals() const {
  return std::valarray<int>{leftSensor.get(), rightSensor.get()};
}

void XDriveModel::resetSensors() const {
  leftSensor.reset();
  rightSensor.reset();
}
} // namespace okapi