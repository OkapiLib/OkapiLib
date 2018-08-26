/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/chassis/model/xDriveModel.hpp"
#include <utility>

namespace okapi {
XDriveModel::XDriveModel(std::shared_ptr<AbstractMotor> itopLeftMotor,
                         std::shared_ptr<AbstractMotor> itopRightMotor,
                         std::shared_ptr<AbstractMotor> ibottomRightMotor,
                         std::shared_ptr<AbstractMotor> ibottomLeftMotor,
                         std::shared_ptr<ContinuousRotarySensor> ileftEnc,
                         std::shared_ptr<ContinuousRotarySensor> irightEnc,
                         const double imaxOutput)
  : topLeftMotor(itopLeftMotor),
    topRightMotor(itopRightMotor),
    bottomRightMotor(ibottomRightMotor),
    bottomLeftMotor(ibottomLeftMotor),
    leftSensor(ileftEnc),
    rightSensor(irightEnc),
    maxOutput(imaxOutput) {
}

XDriveModel::XDriveModel(std::shared_ptr<AbstractMotor> itopLeftMotor,
                         std::shared_ptr<AbstractMotor> itopRightMotor,
                         std::shared_ptr<AbstractMotor> ibottomRightMotor,
                         std::shared_ptr<AbstractMotor> ibottomLeftMotor,
                         const double imaxOutput)
  : topLeftMotor(itopLeftMotor),
    topRightMotor(itopRightMotor),
    bottomRightMotor(ibottomRightMotor),
    bottomLeftMotor(ibottomLeftMotor),
    leftSensor(itopLeftMotor->getEncoder()),
    rightSensor(itopRightMotor->getEncoder()),
    maxOutput(imaxOutput) {
}

void XDriveModel::forward(const double ispeed) const {
  const double speed = std::clamp(ispeed, -1.0, 1.0);
  topLeftMotor->moveVelocity(static_cast<int16_t>(speed * maxOutput));
  topRightMotor->moveVelocity(static_cast<int16_t>(speed * maxOutput));
  bottomRightMotor->moveVelocity(static_cast<int16_t>(speed * maxOutput));
  bottomLeftMotor->moveVelocity(static_cast<int16_t>(speed * maxOutput));
}

void XDriveModel::driveVector(const double iySpeed, const double izRotation) const {
  // This code is taken from WPIlib. All credit goes to them. Link:
  // https://github.com/wpilibsuite/allwpilib/blob/master/wpilibc/src/main/native/cpp/Drive/DifferentialDrive.cpp#L73
  const double ySpeed = std::clamp(iySpeed, -1.0, 1.0);
  const double zRotation = std::clamp(izRotation, -1.0, 1.0);

  double leftOutput = ySpeed + zRotation;
  double rightOutput = ySpeed - zRotation;
  if (const double maxInputMag = std::max<double>(std::abs(leftOutput), std::abs(rightOutput));
      maxInputMag > 1) {
    leftOutput /= maxInputMag;
    rightOutput /= maxInputMag;
  }

  topLeftMotor->moveVelocity(static_cast<int16_t>(leftOutput * maxOutput));
  topRightMotor->moveVelocity(static_cast<int16_t>(rightOutput * maxOutput));
  bottomRightMotor->moveVelocity(static_cast<int16_t>(rightOutput * maxOutput));
  bottomLeftMotor->moveVelocity(static_cast<int16_t>(leftOutput * maxOutput));
}

void XDriveModel::rotate(const double ispeed) const {
  const double speed = std::clamp(ispeed, -1.0, 1.0);
  topLeftMotor->moveVelocity(static_cast<int16_t>(speed * maxOutput));
  topRightMotor->moveVelocity(static_cast<int16_t>(-1 * speed * maxOutput));
  bottomRightMotor->moveVelocity(static_cast<int16_t>(-1 * speed * maxOutput));
  bottomLeftMotor->moveVelocity(static_cast<int16_t>(speed * maxOutput));
}

void XDriveModel::stop() {
  topLeftMotor->moveVelocity(0);
  topRightMotor->moveVelocity(0);
  bottomRightMotor->moveVelocity(0);
  bottomLeftMotor->moveVelocity(0);
}

void XDriveModel::tank(const double ileftSpeed,
                       const double irightSpeed,
                       const double ithreshold) const {
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

  topLeftMotor->moveVoltage(static_cast<int16_t>(leftSpeed * maxOutput));
  topRightMotor->moveVoltage(static_cast<int16_t>(rightSpeed * maxOutput));
  bottomRightMotor->moveVoltage(static_cast<int16_t>(rightSpeed * maxOutput));
  bottomLeftMotor->moveVoltage(static_cast<int16_t>(leftSpeed * maxOutput));
}

void XDriveModel::arcade(const double iySpeed,
                         const double izRotation,
                         const double ithreshold) const {
  // This code is taken from WPIlib. All credit goes to them. Link:
  // https://github.com/wpilibsuite/allwpilib/blob/master/wpilibc/src/main/native/cpp/Drive/DifferentialDrive.cpp#L73
  double ySpeed = std::clamp(iySpeed, -1.0, 1.0);
  if (std::abs(ySpeed) < ithreshold) {
    ySpeed = 0;
  }

  double zRotation = std::clamp(izRotation, -1.0, 1.0);
  if (std::abs(zRotation) < ithreshold) {
    zRotation = 0;
  }

  double maxInput = std::copysign(std::max(std::abs(ySpeed), std::abs(zRotation)), ySpeed);
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

  topLeftMotor->moveVoltage(static_cast<int16_t>(leftOutput * maxOutput));
  topRightMotor->moveVoltage(static_cast<int16_t>(rightOutput * maxOutput));
  bottomRightMotor->moveVoltage(static_cast<int16_t>(rightOutput * maxOutput));
  bottomLeftMotor->moveVoltage(static_cast<int16_t>(leftOutput * maxOutput));
}

void XDriveModel::xArcade(const double ixSpeed,
                          const double iySpeed,
                          const double izRotation,
                          const double ithreshold) const {
  double xSpeed = std::clamp(ixSpeed, -1.0, 1.0);
  if (std::abs(xSpeed) < ithreshold) {
    xSpeed = 0;
  }

  double ySpeed = std::clamp(iySpeed, -1.0, 1.0);
  if (std::abs(ySpeed) < ithreshold) {
    ySpeed = 0;
  }

  double zRotation = std::clamp(izRotation, -1.0, 1.0);
  if (std::abs(zRotation) < ithreshold) {
    zRotation = 0;
  }

  topLeftMotor->moveVoltage(
    static_cast<int16_t>(std::clamp(ySpeed + xSpeed + zRotation, -1.0, 1.0) * maxOutput));
  topRightMotor->moveVoltage(
    static_cast<int16_t>(std::clamp(ySpeed - xSpeed - zRotation, -1.0, 1.0) * maxOutput));
  bottomRightMotor->moveVoltage(
    static_cast<int16_t>(std::clamp(ySpeed + xSpeed - zRotation, -1.0, 1.0) * maxOutput));
  bottomLeftMotor->moveVoltage(
    static_cast<int16_t>(std::clamp(ySpeed - xSpeed + zRotation, -1.0, 1.0) * maxOutput));
}

void XDriveModel::left(const double ispeed) const {
  topLeftMotor->moveVelocity(static_cast<int16_t>(ispeed * maxOutput));
  bottomLeftMotor->moveVelocity(static_cast<int16_t>(ispeed * maxOutput));
}

void XDriveModel::right(const double ispeed) const {
  topRightMotor->moveVelocity(static_cast<int16_t>(ispeed * maxOutput));
  bottomRightMotor->moveVelocity(static_cast<int16_t>(ispeed * maxOutput));
}

std::valarray<std::int32_t> XDriveModel::getSensorVals() const {
  return std::valarray<std::int32_t>{static_cast<std::int32_t>(leftSensor->get()),
                                     static_cast<std::int32_t>(rightSensor->get())};
}

void XDriveModel::resetSensors() const {
  leftSensor->reset();
  rightSensor->reset();
}

void XDriveModel::setBrakeMode(const AbstractMotor::brakeMode mode) const {
  topLeftMotor->setBrakeMode(mode);
  topRightMotor->setBrakeMode(mode);
  bottomRightMotor->setBrakeMode(mode);
  bottomLeftMotor->setBrakeMode(mode);
}

void XDriveModel::setEncoderUnits(const AbstractMotor::encoderUnits units) const {
  topLeftMotor->setEncoderUnits(units);
  topRightMotor->setEncoderUnits(units);
  bottomRightMotor->setEncoderUnits(units);
  bottomLeftMotor->setEncoderUnits(units);
}

void XDriveModel::setGearing(const AbstractMotor::gearset gearset) const {
  topLeftMotor->setGearing(gearset);
  topRightMotor->setGearing(gearset);
  bottomRightMotor->setGearing(gearset);
  bottomLeftMotor->setGearing(gearset);
}

void XDriveModel::setPosPID(const double ikF,
                            const double ikP,
                            const double ikI,
                            const double ikD) const {
  topLeftMotor->setPosPID(ikF, ikP, ikI, ikD);
  topRightMotor->setPosPID(ikF, ikP, ikI, ikD);
  bottomRightMotor->setPosPID(ikF, ikP, ikI, ikD);
  bottomLeftMotor->setPosPID(ikF, ikP, ikI, ikD);
}

void XDriveModel::setPosPIDFull(const double ikF,
                                const double ikP,
                                const double ikI,
                                const double ikD,
                                const double ifilter,
                                const double ilimit,
                                const double ithreshold,
                                const double iloopSpeed) const {
  topLeftMotor->setPosPIDFull(ikF, ikP, ikI, ikD, ifilter, ilimit, ithreshold, iloopSpeed);
  topRightMotor->setPosPIDFull(ikF, ikP, ikI, ikD, ifilter, ilimit, ithreshold, iloopSpeed);
  bottomRightMotor->setPosPIDFull(ikF, ikP, ikI, ikD, ifilter, ilimit, ithreshold, iloopSpeed);
  bottomLeftMotor->setPosPIDFull(ikF, ikP, ikI, ikD, ifilter, ilimit, ithreshold, iloopSpeed);
}

void XDriveModel::setVelPID(const double ikF,
                            const double ikP,
                            const double ikI,
                            const double ikD) const {
  topLeftMotor->setVelPID(ikF, ikP, ikI, ikD);
  topRightMotor->setVelPID(ikF, ikP, ikI, ikD);
  bottomRightMotor->setVelPID(ikF, ikP, ikI, ikD);
  bottomLeftMotor->setVelPID(ikF, ikP, ikI, ikD);
}

void XDriveModel::setVelPIDFull(const double ikF,
                                const double ikP,
                                const double ikI,
                                const double ikD,
                                const double ifilter,
                                const double ilimit,
                                const double ithreshold,
                                const double iloopSpeed) const {
  topLeftMotor->setVelPIDFull(ikF, ikP, ikI, ikD, ifilter, ilimit, ithreshold, iloopSpeed);
  topRightMotor->setVelPIDFull(ikF, ikP, ikI, ikD, ifilter, ilimit, ithreshold, iloopSpeed);
  bottomRightMotor->setVelPIDFull(ikF, ikP, ikI, ikD, ifilter, ilimit, ithreshold, iloopSpeed);
  bottomLeftMotor->setVelPIDFull(ikF, ikP, ikI, ikD, ifilter, ilimit, ithreshold, iloopSpeed);
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
