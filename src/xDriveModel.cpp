/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/chassis/model/xDriveModel.hpp"

namespace okapi {
XDriveModelParams::XDriveModelParams(const AbstractMotor &itopLeftMotor,
                                     const AbstractMotor &itopRightMotor,
                                     const AbstractMotor &ibottomRightMotor,
                                     const AbstractMotor &ibottomLeftMotor,
                                     const RotarySensor &ileftEnc, const RotarySensor &irightEnc,
                                     const double imaxOutput)
  : topLeftMotor(itopLeftMotor),
    topRightMotor(itopRightMotor),
    bottomRightMotor(ibottomRightMotor),
    bottomLeftMotor(ibottomLeftMotor),
    leftSensor(ileftEnc),
    rightSensor(irightEnc),
    maxOutput(imaxOutput) {
}

XDriveModelParams::XDriveModelParams(const AbstractMotor &itopLeftMotor,
                                     const AbstractMotor &itopRightMotor,
                                     const AbstractMotor &ibottomRightMotor,
                                     const AbstractMotor &ibottomLeftMotor, const double imaxOutput)
  : topLeftMotor(itopLeftMotor),
    topRightMotor(itopRightMotor),
    bottomRightMotor(ibottomRightMotor),
    bottomLeftMotor(ibottomLeftMotor),
    leftSensor(itopLeftMotor.getEncoder()),
    rightSensor(itopRightMotor.getEncoder()),
    maxOutput(imaxOutput) {
}

XDriveModelParams::~XDriveModelParams() = default;

XDriveModel::XDriveModel(const AbstractMotor &itopLeftMotor, const AbstractMotor &itopRightMotor,
                         const AbstractMotor &ibottomRightMotor,
                         const AbstractMotor &ibottomLeftMotor, const RotarySensor &ileftEnc,
                         const RotarySensor &irightEnc, const double imaxOutput)
  : topLeftMotor(itopLeftMotor),
    topRightMotor(itopRightMotor),
    bottomRightMotor(ibottomRightMotor),
    bottomLeftMotor(ibottomLeftMotor),
    leftSensor(ileftEnc),
    rightSensor(irightEnc),
    maxOutput(imaxOutput) {
}

XDriveModel::XDriveModel(const AbstractMotor &itopLeftMotor, const AbstractMotor &itopRightMotor,
                         const AbstractMotor &ibottomRightMotor,
                         const AbstractMotor &ibottomLeftMotor, const double imaxOutput)
  : topLeftMotor(itopLeftMotor),
    topRightMotor(itopRightMotor),
    bottomRightMotor(ibottomRightMotor),
    bottomLeftMotor(ibottomLeftMotor),
    leftSensor(itopLeftMotor.getEncoder()),
    rightSensor(itopRightMotor.getEncoder()),
    maxOutput(imaxOutput) {
}

XDriveModel::XDriveModel(const XDriveModelParams &iparams)
  : topLeftMotor(iparams.topLeftMotor),
    topRightMotor(iparams.topRightMotor),
    bottomRightMotor(iparams.bottomRightMotor),
    bottomLeftMotor(iparams.bottomLeftMotor),
    leftSensor(iparams.leftSensor),
    rightSensor(iparams.rightSensor),
    maxOutput(iparams.maxOutput) {
}

XDriveModel::XDriveModel(const XDriveModel &other)
  : topLeftMotor(other.topLeftMotor),
    topRightMotor(other.topRightMotor),
    bottomRightMotor(other.bottomRightMotor),
    bottomLeftMotor(other.bottomLeftMotor),
    leftSensor(other.leftSensor),
    rightSensor(other.rightSensor),
    maxOutput(other.maxOutput) {
}

XDriveModel::~XDriveModel() = default;

void XDriveModel::forward(const double ipower) const {
  topLeftMotor.move_velocity(ipower);
  topRightMotor.move_velocity(ipower);
  bottomRightMotor.move_velocity(ipower);
  bottomLeftMotor.move_velocity(ipower);
}

void XDriveModel::driveVector(const double idistPower, const double ianglePower) const {
  topLeftMotor.move_velocity(idistPower + ianglePower);
  topRightMotor.move_velocity(idistPower - ianglePower);
  bottomRightMotor.move_velocity(idistPower - ianglePower);
  bottomLeftMotor.move_velocity(idistPower + ianglePower);
}

void XDriveModel::rotate(const double ipower) const {
  topLeftMotor.move_velocity(ipower);
  topRightMotor.move_velocity(-1 * ipower);
  bottomRightMotor.move_velocity(-1 * ipower);
  bottomLeftMotor.move_velocity(ipower);
}

void XDriveModel::stop() const {
  topLeftMotor.move_velocity(0);
  topRightMotor.move_velocity(0);
  bottomRightMotor.move_velocity(0);
  bottomLeftMotor.move_velocity(0);
}

void XDriveModel::tank(const double ileftVal, const double irightVal,
                       const double ithreshold) const {
  if (fabs(ileftVal) < ithreshold) {
    topLeftMotor.move_velocity(0);
    bottomLeftMotor.move_velocity(0);
  } else {
    topLeftMotor.move_velocity(ileftVal);
    bottomLeftMotor.move_velocity(ileftVal);
  }

  if (fabs(irightVal) < ithreshold) {
    topRightMotor.move_velocity(0);
    bottomRightMotor.move_velocity(0);
  } else {
    topRightMotor.move_velocity(irightVal);
    bottomRightMotor.move_velocity(irightVal);
  }
}

void XDriveModel::arcade(double iverticalVal, double ihorizontalVal,
                         const double ithreshold) const {
  if (fabs(iverticalVal) < ithreshold)
    iverticalVal = 0;
  if (fabs(ihorizontalVal) < ithreshold)
    ihorizontalVal = 0;

  topLeftMotor.move_velocity(iverticalVal + ihorizontalVal);
  topRightMotor.move_velocity(iverticalVal - ihorizontalVal);
  bottomRightMotor.move_velocity(iverticalVal - ihorizontalVal);
  bottomLeftMotor.move_velocity(iverticalVal + ihorizontalVal);
}

void XDriveModel::xArcade(double iverticalVal, double ihorizontalVal, double irotateVal,
                          const double ithreshold) const {
  if (fabs(iverticalVal) < ithreshold)
    iverticalVal = 0;
  if (fabs(ihorizontalVal) < ithreshold)
    ihorizontalVal = 0;
  if (fabs(irotateVal) < ithreshold)
    irotateVal = 0;

  topLeftMotor.move_velocity(iverticalVal + ihorizontalVal + irotateVal);
  topRightMotor.move_velocity(iverticalVal - ihorizontalVal - irotateVal);
  bottomRightMotor.move_velocity(iverticalVal + ihorizontalVal - irotateVal);
  bottomLeftMotor.move_velocity(iverticalVal - ihorizontalVal + irotateVal);
}

void XDriveModel::left(const double ipower) const {
  topLeftMotor.move_velocity(ipower);
  bottomLeftMotor.move_velocity(ipower);
}

void XDriveModel::right(const double ipower) const {
  topRightMotor.move_velocity(ipower);
  bottomRightMotor.move_velocity(ipower);
}

std::valarray<int> XDriveModel::getSensorVals() const {
  return std::valarray<int>{leftSensor.get(), rightSensor.get()};
}

void XDriveModel::resetSensors() const {
  leftSensor.reset();
  rightSensor.reset();
}
} // namespace okapi
