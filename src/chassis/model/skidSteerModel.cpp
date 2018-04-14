/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/chassis/model/skidSteerModel.hpp"
#include <algorithm>
#include <utility>

namespace okapi {
SkidSteerModelArgs::SkidSteerModelArgs(const AbstractMotor &ileftSideMotor,
                                       const AbstractMotor &irightSideMotor,
                                       const RotarySensor &ileftEnc, const RotarySensor &irightEnc,
                                       const double imaxOutput)
  : leftSideMotor(ileftSideMotor),
    rightSideMotor(irightSideMotor),
    leftSensor(ileftEnc),
    rightSensor(irightEnc),
    maxOutput(imaxOutput) {
}

SkidSteerModelArgs::SkidSteerModelArgs(const AbstractMotor &ileftSideMotor,
                                       const AbstractMotor &irightSideMotor,
                                       const double imaxOutput)
  : leftSideMotor(ileftSideMotor),
    rightSideMotor(irightSideMotor),
    leftSensor(std::move(ileftSideMotor.getEncoder())),
    rightSensor(std::move(irightSideMotor.getEncoder())),
    maxOutput(imaxOutput) {
}

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
    leftSensor(std::move(ileftSideMotor.getEncoder())),
    rightSensor(std::move(irightSideMotor.getEncoder())),
    maxOutput(imaxOutput) {
}

SkidSteerModel::SkidSteerModel(const SkidSteerModelArgs &iparams)
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

void SkidSteerModel::forward(const double ispeed) const {
  leftSideMotor.moveVelocity(ispeed * maxOutput);
  rightSideMotor.moveVelocity(ispeed * maxOutput);
}

void SkidSteerModel::driveVector(const double iySpeed, const double izRotation) const {
  // This code is taken from WPIlib. All credit goes to them. Link:
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

  leftSideMotor.moveVelocity(leftOutput * maxOutput);
  rightSideMotor.moveVelocity(rightOutput * maxOutput);
}

void SkidSteerModel::rotate(const double ispeed) const {
  leftSideMotor.moveVelocity(ispeed * maxOutput);
  rightSideMotor.moveVelocity(-1 * ispeed * maxOutput);
}

void SkidSteerModel::stop() const {
  leftSideMotor.moveVelocity(0);
  rightSideMotor.moveVelocity(0);
}

void SkidSteerModel::tank(const double ileftSpeed, const double irightSpeed,
                          const double ithreshold) const {
  // This code is taken from WPIlib. All credit goes to them. Link:
  // https://github.com/wpilibsuite/allwpilib/blob/master/wpilibc/src/main/native/cpp/Drive/DifferentialDrive.cpp#L73
  double leftSpeed = std::clamp(ileftSpeed, -1.0, 1.0);
  if (fabs(leftSpeed) < ithreshold) {
    leftSpeed = 0;
  }

  double rightSpeed = std::clamp(irightSpeed, -1.0, 1.0);
  if (fabs(rightSpeed) < ithreshold) {
    rightSpeed = 0;
  }

  leftSideMotor.moveVoltage(leftSpeed * maxOutput);
  rightSideMotor.moveVoltage(rightSpeed * maxOutput);
}

void SkidSteerModel::arcade(const double iySpeed, const double izRotation,
                            const double ithreshold) const {
  // This code is taken from WPIlib. All credit goes to them. Link:
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
  printf("%1.2f,%1.2f\n", leftOutput, std::clamp(leftOutput, -1.0, 1.0) * maxOutput);

  leftSideMotor.moveVoltage(std::clamp(leftOutput, -1.0, 1.0) * maxOutput);
  rightSideMotor.moveVoltage(std::clamp(rightOutput, -1.0, 1.0) * maxOutput);
}

void SkidSteerModel::left(const double ispeed) const {
  leftSideMotor.moveVelocity(ispeed * maxOutput);
}

void SkidSteerModel::right(const double ispeed) const {
  rightSideMotor.moveVelocity(ispeed * maxOutput);
}

std::valarray<int> SkidSteerModel::getSensorVals() const {
  return std::valarray<int>{leftSensor.get(), rightSensor.get()};
}

void SkidSteerModel::resetSensors() const {
  leftSensor.reset();
  rightSensor.reset();
}

void SkidSteerModel::setBrakeMode(const motor_brake_mode_e_t mode) const {
  leftSideMotor.setBrakeMode(mode);
  rightSideMotor.setBrakeMode(mode);
}

void SkidSteerModel::setEncoderUnits(const motor_encoder_units_e_t units) const {
  leftSideMotor.setEncoderUnits(units);
  rightSideMotor.setEncoderUnits(units);
}

void SkidSteerModel::setGearing(const motor_gearset_e_t gearset) const {
  leftSideMotor.setGearing(gearset);
  rightSideMotor.setGearing(gearset);
}

const AbstractMotor &SkidSteerModel::getLeftSideMotor() const {
  return leftSideMotor;
}

const AbstractMotor &SkidSteerModel::getRightSideMotor() const {
  return rightSideMotor;
}
} // namespace okapi
