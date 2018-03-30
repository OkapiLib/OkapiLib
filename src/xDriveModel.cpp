/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/chassis/model/xDriveModel.hpp"
#include <utility>

namespace okapi {
XDriveModelArgs::XDriveModelArgs(const AbstractMotor &itopLeftMotor,
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

XDriveModelArgs::XDriveModelArgs(const AbstractMotor &itopLeftMotor,
                                     const AbstractMotor &itopRightMotor,
                                     const AbstractMotor &ibottomRightMotor,
                                     const AbstractMotor &ibottomLeftMotor, const double imaxOutput)
  : topLeftMotor(itopLeftMotor),
    topRightMotor(itopRightMotor),
    bottomRightMotor(ibottomRightMotor),
    bottomLeftMotor(ibottomLeftMotor),
    leftSensor(std::move(itopLeftMotor.getEncoder())),
    rightSensor(std::move(itopRightMotor.getEncoder())),
    maxOutput(imaxOutput) {
}

XDriveModelArgs::~XDriveModelArgs() = default;

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
    leftSensor(std::move(itopLeftMotor.getEncoder())),
    rightSensor(std::move(itopRightMotor.getEncoder())),
    maxOutput(imaxOutput) {
}

XDriveModel::XDriveModel(const XDriveModelArgs &iparams)
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

void XDriveModel::forward(const double ispeed) const {
  topLeftMotor.move_velocity(ispeed * maxOutput);
  topRightMotor.move_velocity(ispeed * maxOutput);
  bottomRightMotor.move_velocity(ispeed * maxOutput);
  bottomLeftMotor.move_velocity(ispeed * maxOutput);
}

void XDriveModel::driveVector(const double iySpeed, const double izRotation) const {
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

  topLeftMotor.move_velocity(leftOutput * maxOutput);
  topRightMotor.move_velocity(rightOutput * maxOutput);
  bottomRightMotor.move_velocity(rightOutput * maxOutput);
  bottomLeftMotor.move_velocity(leftOutput * maxOutput);
}

void XDriveModel::rotate(const double ispeed) const {
  topLeftMotor.move_velocity(ispeed * maxOutput);
  topRightMotor.move_velocity(-1 * ispeed * maxOutput);
  bottomRightMotor.move_velocity(-1 * ispeed * maxOutput);
  bottomLeftMotor.move_velocity(ispeed * maxOutput);
}

void XDriveModel::stop() const {
  topLeftMotor.move_velocity(0);
  topRightMotor.move_velocity(0);
  bottomRightMotor.move_velocity(0);
  bottomLeftMotor.move_velocity(0);
}

void XDriveModel::tank(const double ileftSpeed, const double irightSpeed,
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

  topLeftMotor.move_voltage(leftSpeed * maxOutput);
  topRightMotor.move_voltage(rightSpeed * maxOutput);
  bottomRightMotor.move_voltage(rightSpeed * maxOutput);
  bottomLeftMotor.move_voltage(leftSpeed * maxOutput);
}

void XDriveModel::arcade(const double iySpeed, const double izRotation,
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

  leftOutput = std::clamp(leftOutput, -1.0, 1.0);
  rightOutput = std::clamp(rightOutput, -1.0, 1.0);

  topLeftMotor.move_voltage(leftOutput * maxOutput);
  topRightMotor.move_voltage(rightOutput * maxOutput);
  bottomRightMotor.move_voltage(rightOutput * maxOutput);
  bottomLeftMotor.move_voltage(leftOutput * maxOutput);
}

void XDriveModel::xArcade(const double ixSpeed, const double iySpeed, const double izRotation,
                          const double ithreshold) const {
  double xSpeed = std::clamp(ixSpeed, -1.0, 1.0);
  if (fabs(xSpeed) < ithreshold) {
    xSpeed = 0;
  }

  double ySpeed = std::clamp(iySpeed, -1.0, 1.0);
  if (fabs(ySpeed) < ithreshold) {
    ySpeed = 0;
  }

  double zRotation = std::clamp(izRotation, -1.0, 1.0);
  if (fabs(zRotation) < ithreshold) {
    zRotation = 0;
  }

  topLeftMotor.move_voltage(ySpeed + xSpeed + zRotation);
  topRightMotor.move_voltage(ySpeed - xSpeed - zRotation);
  bottomRightMotor.move_voltage(ySpeed + xSpeed - zRotation);
  bottomLeftMotor.move_voltage(ySpeed - xSpeed + zRotation);
}

void XDriveModel::left(const double ispeed) const {
  topLeftMotor.move_velocity(ispeed * maxOutput);
  bottomLeftMotor.move_velocity(ispeed * maxOutput);
}

void XDriveModel::right(const double ispeed) const {
  topRightMotor.move_velocity(ispeed * maxOutput);
  bottomRightMotor.move_velocity(ispeed * maxOutput);
}

std::valarray<int> XDriveModel::getSensorVals() const {
  return std::valarray<int>{leftSensor.get(), rightSensor.get()};
}

void XDriveModel::resetSensors() const {
  leftSensor.reset();
  rightSensor.reset();
}

void XDriveModel::setBrakeMode(const motor_brake_mode_e_t mode) const {
  topLeftMotor.set_brake_mode(mode);
  topRightMotor.set_brake_mode(mode);
  bottomRightMotor.set_brake_mode(mode);
  bottomLeftMotor.set_brake_mode(mode);
}

void XDriveModel::setEncoderUnits(const motor_encoder_units_e_t units) const {
  topLeftMotor.set_encoder_units(units);
  topRightMotor.set_encoder_units(units);
  bottomRightMotor.set_encoder_units(units);
  bottomLeftMotor.set_encoder_units(units);
}

void XDriveModel::setGearing(const motor_gearset_e_t gearset) const {
  topLeftMotor.set_gearing(gearset);
  topRightMotor.set_gearing(gearset);
  bottomRightMotor.set_gearing(gearset);
  bottomLeftMotor.set_gearing(gearset);
}
} // namespace okapi
