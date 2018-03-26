/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/chassis/model/skidSteerModel.hpp"
#include <algorithm>

namespace okapi {
SkidSteerModelParams::SkidSteerModelParams(const AbstractMotor &ileftSideMotor,
                                           const AbstractMotor &irightSideMotor,
                                           const RotarySensor &ileftEnc,
                                           const RotarySensor &irightEnc, const double imaxOutput)
  : leftSideMotor(ileftSideMotor),
    rightSideMotor(irightSideMotor),
    leftSensor(ileftEnc),
    rightSensor(irightEnc),
    maxOutput(imaxOutput) {
}

SkidSteerModelParams::SkidSteerModelParams(const AbstractMotor &ileftSideMotor,
                                           const AbstractMotor &irightSideMotor,
                                           const double imaxOutput)
  : leftSideMotor(ileftSideMotor),
    rightSideMotor(irightSideMotor),
    leftSensor(ileftSideMotor.getEncoder()),
    rightSensor(irightSideMotor.getEncoder()),
    maxOutput(imaxOutput) {
}

SkidSteerModelParams::~SkidSteerModelParams() = default;

SkidSteerModel::SkidSteerModel(const AbstractMotor &ileftSideMotor,
                               const AbstractMotor &irightSideMotor, const RotarySensor &ileftEnc,
                               const RotarySensor &irightEnc, const double imaxOutput)
  : leftSideMotor(ileftSideMotor),
    rightSideMotor(irightSideMotor),
    leftSensor(ileftEnc),
    rightSensor(irightEnc),
    maxOutput(imaxOutput) {
}

SkidSteerModel::SkidSteerModel(const AbstractMotor &ileftSideMotor,
                               const AbstractMotor &irightSideMotor, const double imaxOutput)
  : leftSideMotor(ileftSideMotor),
    rightSideMotor(irightSideMotor),
    leftSensor(ileftSideMotor.getEncoder()),
    rightSensor(irightSideMotor.getEncoder()),
    maxOutput(imaxOutput) {
}

SkidSteerModel::SkidSteerModel(const SkidSteerModelParams &iparams)
  : leftSideMotor(iparams.leftSideMotor),
    rightSideMotor(iparams.rightSideMotor),
    leftSensor(iparams.leftSensor),
    rightSensor(iparams.rightSensor),
    maxOutput(iparams.maxOutput) {
}

SkidSteerModel::SkidSteerModel(const SkidSteerModel &other)
  : leftSideMotor(other.leftSideMotor),
    rightSideMotor(other.rightSideMotor),
    leftSensor(other.leftSensor),
    rightSensor(other.rightSensor),
    maxOutput(other.maxOutput) {
}

SkidSteerModel::~SkidSteerModel() = default;

void SkidSteerModel::forward(const double ipower) const {
  leftSideMotor.move_velocity(ipower * maxOutput);
  rightSideMotor.move_velocity(ipower * maxOutput);
}

void SkidSteerModel::driveVector(const double iySpeed, const double izRotation) const {
  // This code is taken from WPIlib. ALl credit goes to them. Link:
  // https://github.com/wpilibsuite/allwpilib/blob/master/wpilibc/src/main/native/cpp/Drive/DifferentialDrive.cpp#L73
  const double ySpeed = std::clamp(iySpeed, -1.0, 1.0);
  const double zRotation = std::clamp(izRotation, -1.0, 1.0);

  double leftOutput = ySpeed + zRotation;
  double rightOutput = ySpeed - zRotation;
  const double maxInputMag = std::max(abs(ySpeed), abs(zRotation));
  if (maxInputMag > 1) {
    leftOutput /= maxInputMag;
    rightOutput /= maxInputMag;
  }

  leftSideMotor.move_velocity(leftOutput * maxOutput);
  rightSideMotor.move_velocity(rightOutput * maxOutput);
}

void SkidSteerModel::rotate(const double ipower) const {
  leftSideMotor.move_velocity(ipower * maxOutput);
  rightSideMotor.move_velocity(-1 * ipower * maxOutput);
}

void SkidSteerModel::stop() const {
  leftSideMotor.move_velocity(0);
  rightSideMotor.move_velocity(0);
}

void SkidSteerModel::tank(const double ileftSpeed, const double irightSpeed,
                          const double ithreshold) const {
  // This code is taken from WPIlib. ALl credit goes to them. Link:
  // https://github.com/wpilibsuite/allwpilib/blob/master/wpilibc/src/main/native/cpp/Drive/DifferentialDrive.cpp#L73
  double leftSpeed = std::clamp(ileftSpeed, -1.0, 1.0);
  if (fabs(leftSpeed) < ithreshold) {
    leftSpeed = 0;
  }

  double rightSpeed = std::clamp(irightSpeed, -1.0, 1.0);
  if (fabs(rightSpeed) < ithreshold) {
    rightSpeed = 0;
  }

  leftSideMotor.move_voltage(leftSpeed * maxOutput);
  rightSideMotor.move_voltage(rightSpeed * maxOutput);
}

void SkidSteerModel::arcade(const double iySpeed, const double izRotation,
                            const double ithreshold) const {
  // This code is taken from WPIlib. ALl credit goes to them. Link:
  // https://github.com/wpilibsuite/allwpilib/blob/master/wpilibc/src/main/native/cpp/Drive/DifferentialDrive.cpp#L73
  double ySpeed = std::clamp(iySpeed, -1.0, 1.0);
  if (fabs(ySpeed) < ithreshold) {
    ySpeed = 0;
  }

  double zRotation = std::clamp(izRotation, -1.0, 1.0);
  if (fabs(zRotation) < ithreshold) {
    zRotation = 0;
  }

  double maxInput = std::copysign(std::max(fabs(ySpeed), fabs(zRotation)), ySpeed);
  double leftOutput = 0;
  double rightOutput = 0;

  if (ySpeed >= 0) {
    if (zRotation >= 0) {
      leftOutput = maxInput;
      rightOutput = ySpeed - zRotation;
    } else {
      leftOutput = ySpeed + zRotation;
      rightOutput = maxInput;
    }
  } else {
    if (zRotation >= 0) {
      leftOutput = ySpeed + zRotation;
      rightOutput = maxInput;
    } else {
      leftOutput = maxInput;
      rightOutput = ySpeed - zRotation;
    }
  }

  leftSideMotor.move_voltage(std::clamp(leftOutput, -1.0, 1.0) * maxOutput);
  rightSideMotor.move_voltage(std::clamp(rightOutput, -1.0, 1.0) * maxOutput);
}

void SkidSteerModel::left(const double ipower) const {
  leftSideMotor.move_velocity(ipower * maxOutput);
}

void SkidSteerModel::right(const double ipower) const {
  rightSideMotor.move_velocity(ipower * maxOutput);
}

std::valarray<int> SkidSteerModel::getSensorVals() const {
  return std::valarray<int>{leftSensor.get(), rightSensor.get()};
}

void SkidSteerModel::resetSensors() const {
  leftSensor.reset();
  rightSensor.reset();
}
} // namespace okapi
