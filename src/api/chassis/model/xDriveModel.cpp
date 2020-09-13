/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/chassis/model/xDriveModel.hpp"
#include "okapi/api/util/mathUtil.hpp"
#include <utility>

namespace okapi {
XDriveModel::XDriveModel(std::shared_ptr<AbstractMotor> itopLeftMotor,
                         std::shared_ptr<AbstractMotor> itopRightMotor,
                         std::shared_ptr<AbstractMotor> ibottomRightMotor,
                         std::shared_ptr<AbstractMotor> ibottomLeftMotor,
                         std::shared_ptr<ContinuousRotarySensor> ileftEnc,
                         std::shared_ptr<ContinuousRotarySensor> irightEnc,
                         const double imaxVelocity,
                         const double imaxVoltage)
  : maxVelocity(imaxVelocity),
    maxVoltage(imaxVoltage),
    topLeftMotor(std::move(itopLeftMotor)),
    topRightMotor(std::move(itopRightMotor)),
    bottomRightMotor(std::move(ibottomRightMotor)),
    bottomLeftMotor(std::move(ibottomLeftMotor)),
    leftSensor(std::move(ileftEnc)),
    rightSensor(std::move(irightEnc)) {
}

void XDriveModel::forward(const double ispeed) {
  const double speed = std::clamp(ispeed, -1.0, 1.0);
  topLeftMotor->moveVelocity(static_cast<int16_t>(speed * maxVelocity));
  topRightMotor->moveVelocity(static_cast<int16_t>(speed * maxVelocity));
  bottomRightMotor->moveVelocity(static_cast<int16_t>(speed * maxVelocity));
  bottomLeftMotor->moveVelocity(static_cast<int16_t>(speed * maxVelocity));
}

void XDriveModel::driveVector(const double iforwardSpeed, const double iyaw) {
  // This code is taken from WPIlib. All credit goes to them. Link:
  // https://github.com/wpilibsuite/allwpilib/blob/master/wpilibc/src/main/native/cpp/Drive/DifferentialDrive.cpp#L73
  const double forwardSpeed = std::clamp(iforwardSpeed, -1.0, 1.0);
  const double yaw = std::clamp(iyaw, -1.0, 1.0);

  double leftOutput = forwardSpeed + yaw;
  double rightOutput = forwardSpeed - yaw;
  if (const double maxInputMag = std::max<double>(std::abs(leftOutput), std::abs(rightOutput));
      maxInputMag > 1) {
    leftOutput /= maxInputMag;
    rightOutput /= maxInputMag;
  }

  topLeftMotor->moveVelocity(static_cast<int16_t>(leftOutput * maxVelocity));
  topRightMotor->moveVelocity(static_cast<int16_t>(rightOutput * maxVelocity));
  bottomRightMotor->moveVelocity(static_cast<int16_t>(rightOutput * maxVelocity));
  bottomLeftMotor->moveVelocity(static_cast<int16_t>(leftOutput * maxVelocity));
}

void XDriveModel::driveVectorVoltage(const double iforwardSpeed, const double iyaw) {
  // This code is taken from WPIlib. All credit goes to them. Link:
  // https://github.com/wpilibsuite/allwpilib/blob/master/wpilibc/src/main/native/cpp/Drive/DifferentialDrive.cpp#L73
  const double forwardSpeed = std::clamp(iforwardSpeed, -1.0, 1.0);
  const double yaw = std::clamp(iyaw, -1.0, 1.0);

  double leftOutput = forwardSpeed + yaw;
  double rightOutput = forwardSpeed - yaw;
  if (const double maxInputMag = std::max<double>(std::abs(leftOutput), std::abs(rightOutput));
      maxInputMag > 1) {
    leftOutput /= maxInputMag;
    rightOutput /= maxInputMag;
  }

  topLeftMotor->moveVoltage(static_cast<int16_t>(leftOutput * maxVoltage));
  topRightMotor->moveVoltage(static_cast<int16_t>(rightOutput * maxVoltage));
  bottomRightMotor->moveVoltage(static_cast<int16_t>(rightOutput * maxVoltage));
  bottomLeftMotor->moveVoltage(static_cast<int16_t>(leftOutput * maxVoltage));
}

void XDriveModel::rotate(const double ispeed) {
  const double speed = std::clamp(ispeed, -1.0, 1.0);
  topLeftMotor->moveVelocity(static_cast<int16_t>(speed * maxVelocity));
  topRightMotor->moveVelocity(static_cast<int16_t>(-1 * speed * maxVelocity));
  bottomRightMotor->moveVelocity(static_cast<int16_t>(-1 * speed * maxVelocity));
  bottomLeftMotor->moveVelocity(static_cast<int16_t>(speed * maxVelocity));
}

void XDriveModel::strafe(const double ispeed) {
  const double speed = std::clamp(ispeed, -1.0, 1.0);
  topLeftMotor->moveVelocity(static_cast<int16_t>(-1 * speed * maxVelocity));
  topRightMotor->moveVelocity(static_cast<int16_t>(speed * maxVelocity));
  bottomRightMotor->moveVelocity(static_cast<int16_t>(-1 * speed * maxVelocity));
  bottomLeftMotor->moveVelocity(static_cast<int16_t>(speed * maxVelocity));
}

void XDriveModel::strafeVector(const double istrafeSpeed, const double iyaw) {
  // This code is taken from WPIlib. All credit goes to them. Link:
  // https://github.com/wpilibsuite/allwpilib/blob/master/wpilibc/src/main/native/cpp/Drive/DifferentialDrive.cpp#L73
  const double forwardSpeed = std::clamp(istrafeSpeed, -1.0, 1.0);
  const double yaw = std::clamp(iyaw, -1.0, 1.0);

  double topLeftOutput = -1 * forwardSpeed + yaw;
  double topRightOutput = forwardSpeed - yaw;
  double bottomRightOutput = -1 * forwardSpeed - yaw;
  double bottomLeftOutput = forwardSpeed + yaw;

  double maxInputMag = std::max<double>(std::abs(topLeftOutput), std::abs(topRightOutput));
  maxInputMag = std::max<double>(std::abs(maxInputMag), std::abs(bottomRightOutput));
  maxInputMag = std::max<double>(std::abs(maxInputMag), std::abs(bottomLeftOutput));
  if (maxInputMag > 1) {
    topLeftOutput /= maxInputMag;
    topRightOutput /= maxInputMag;
    bottomRightOutput /= maxInputMag;
    bottomLeftOutput /= maxInputMag;
  }

  topLeftMotor->moveVelocity(static_cast<int16_t>(topLeftOutput * maxVelocity));
  topRightMotor->moveVelocity(static_cast<int16_t>(topRightOutput * maxVelocity));
  bottomRightMotor->moveVelocity(static_cast<int16_t>(bottomRightOutput * maxVelocity));
  bottomLeftMotor->moveVelocity(static_cast<int16_t>(bottomLeftOutput * maxVelocity));
}

void XDriveModel::stop() {
  topLeftMotor->moveVelocity(0);
  topRightMotor->moveVelocity(0);
  bottomRightMotor->moveVelocity(0);
  bottomLeftMotor->moveVelocity(0);
}

void XDriveModel::tank(const double ileftSpeed, const double irightSpeed, const double ithreshold) {
  // This code is taken from WPIlib. All credit goes to them. Link:
  // https://github.com/wpilibsuite/allwpilib/blob/master/wpilibc/src/main/native/cpp/Drive/DifferentialDrive.cpp#L73
  double leftSpeed = std::clamp(ileftSpeed, -1.0, 1.0);
  if (std::abs(leftSpeed) < ithreshold) {
    leftSpeed = 0;
  }

  double rightSpeed = std::clamp(irightSpeed, -1.0, 1.0);
  if (std::abs(rightSpeed) < ithreshold) {
    rightSpeed = 0;
  }

  topLeftMotor->moveVoltage(static_cast<int16_t>(leftSpeed * maxVoltage));
  topRightMotor->moveVoltage(static_cast<int16_t>(rightSpeed * maxVoltage));
  bottomRightMotor->moveVoltage(static_cast<int16_t>(rightSpeed * maxVoltage));
  bottomLeftMotor->moveVoltage(static_cast<int16_t>(leftSpeed * maxVoltage));
}

void XDriveModel::arcade(const double iforwardSpeed, const double iyaw, const double ithreshold) {
  // This code is taken from WPIlib. All credit goes to them. Link:
  // https://github.com/wpilibsuite/allwpilib/blob/master/wpilibc/src/main/native/cpp/Drive/DifferentialDrive.cpp#L73
  double forwardSpeed = std::clamp(iforwardSpeed, -1.0, 1.0);
  if (std::abs(forwardSpeed) <= ithreshold) {
    forwardSpeed = 0;
  }

  double yaw = std::clamp(iyaw, -1.0, 1.0);
  if (std::abs(yaw) <= ithreshold) {
    yaw = 0;
  }

  double maxInput = std::copysign(std::max(std::abs(forwardSpeed), std::abs(yaw)), forwardSpeed);
  double leftOutput = 0;
  double rightOutput = 0;

  if (forwardSpeed >= 0) {
    if (yaw >= 0) {
      leftOutput = maxInput;
      rightOutput = forwardSpeed - yaw;
    } else {
      leftOutput = forwardSpeed + yaw;
      rightOutput = maxInput;
    }
  } else {
    if (yaw >= 0) {
      leftOutput = forwardSpeed + yaw;
      rightOutput = maxInput;
    } else {
      leftOutput = maxInput;
      rightOutput = forwardSpeed - yaw;
    }
  }

  leftOutput = std::clamp(leftOutput, -1.0, 1.0);
  rightOutput = std::clamp(rightOutput, -1.0, 1.0);

  topLeftMotor->moveVoltage(static_cast<int16_t>(leftOutput * maxVoltage));
  topRightMotor->moveVoltage(static_cast<int16_t>(rightOutput * maxVoltage));
  bottomRightMotor->moveVoltage(static_cast<int16_t>(rightOutput * maxVoltage));
  bottomLeftMotor->moveVoltage(static_cast<int16_t>(leftOutput * maxVoltage));
}

void XDriveModel::xArcade(const double ixSpeed,
                          const double iforwardSpeed,
                          const double iyaw,
                          const double ithreshold) {
  double xSpeed = std::clamp(ixSpeed, -1.0, 1.0);
  if (std::abs(xSpeed) < ithreshold) {
    xSpeed = 0;
  }

  double forwardSpeed = std::clamp(iforwardSpeed, -1.0, 1.0);
  if (std::abs(forwardSpeed) < ithreshold) {
    forwardSpeed = 0;
  }

  double yaw = std::clamp(iyaw, -1.0, 1.0);
  if (std::abs(yaw) < ithreshold) {
    yaw = 0;
  }

  topLeftMotor->moveVoltage(
    static_cast<int16_t>(std::clamp(forwardSpeed + xSpeed + yaw, -1.0, 1.0) * maxVoltage));
  topRightMotor->moveVoltage(
    static_cast<int16_t>(std::clamp(forwardSpeed - xSpeed - yaw, -1.0, 1.0) * maxVoltage));
  bottomRightMotor->moveVoltage(
    static_cast<int16_t>(std::clamp(forwardSpeed + xSpeed - yaw, -1.0, 1.0) * maxVoltage));
  bottomLeftMotor->moveVoltage(
    static_cast<int16_t>(std::clamp(forwardSpeed - xSpeed + yaw, -1.0, 1.0) * maxVoltage));
}

void XDriveModel::left(const double ispeed) {
  const double speed = std::clamp(ispeed, -1.0, 1.0);
  topLeftMotor->moveVelocity(static_cast<int16_t>(speed * maxVelocity));
  bottomLeftMotor->moveVelocity(static_cast<int16_t>(speed * maxVelocity));
}

void XDriveModel::right(const double ispeed) {
  const double speed = std::clamp(ispeed, -1.0, 1.0);
  topRightMotor->moveVelocity(static_cast<int16_t>(speed * maxVelocity));
  bottomRightMotor->moveVelocity(static_cast<int16_t>(speed * maxVelocity));
}

std::valarray<std::int32_t> XDriveModel::getSensorVals() const {
  return std::valarray<std::int32_t>{static_cast<std::int32_t>(leftSensor->get()),
                                     static_cast<std::int32_t>(rightSensor->get())};
}

void XDriveModel::resetSensors() {
  leftSensor->reset();
  rightSensor->reset();
}

void XDriveModel::setBrakeMode(const AbstractMotor::brakeMode mode) {
  topLeftMotor->setBrakeMode(mode);
  topRightMotor->setBrakeMode(mode);
  bottomRightMotor->setBrakeMode(mode);
  bottomLeftMotor->setBrakeMode(mode);
}

void XDriveModel::setEncoderUnits(const AbstractMotor::encoderUnits units) {
  topLeftMotor->setEncoderUnits(units);
  topRightMotor->setEncoderUnits(units);
  bottomRightMotor->setEncoderUnits(units);
  bottomLeftMotor->setEncoderUnits(units);
}

void XDriveModel::setGearing(const AbstractMotor::gearset gearset) {
  topLeftMotor->setGearing(gearset);
  topRightMotor->setGearing(gearset);
  bottomRightMotor->setGearing(gearset);
  bottomLeftMotor->setGearing(gearset);
}

void XDriveModel::setMaxVelocity(double imaxVelocity) {
  if (imaxVelocity < 0) {
    maxVelocity = 0;
  } else {
    maxVelocity = imaxVelocity;
  }
}

double XDriveModel::getMaxVelocity() const {
  return maxVelocity;
}

void XDriveModel::setMaxVoltage(double imaxVoltage) {
  maxVoltage = std::clamp(imaxVoltage, 0.0, v5MotorMaxVoltage);
}

double XDriveModel::getMaxVoltage() const {
  return maxVoltage;
}

std::shared_ptr<AbstractMotor> XDriveModel::getTopLeftMotor() const {
  return topLeftMotor;
}

std::shared_ptr<AbstractMotor> XDriveModel::getTopRightMotor() const {
  return topRightMotor;
}

std::shared_ptr<AbstractMotor> XDriveModel::getBottomRightMotor() const {
  return bottomRightMotor;
}

std::shared_ptr<AbstractMotor> XDriveModel::getBottomLeftMotor() const {
  return bottomLeftMotor;
}
} // namespace okapi
