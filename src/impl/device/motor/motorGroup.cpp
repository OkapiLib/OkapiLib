/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/impl/device/motor/motorGroup.hpp"
#include "okapi/api/util/logging.hpp"

namespace okapi {
MotorGroup::MotorGroup(const std::initializer_list<Motor> &imotors) : motors(imotors) {
  if (motors.empty()) {
    Logger::instance()->error(
      "MotorGroup: A MotorGroup must be created with at least one motor. No motors were given.");
    throw std::invalid_argument(
      "MotorGroup: A MotorGroup must be created with at least one motor. No motors were given.");
  }
}

std::int32_t MotorGroup::moveAbsolute(const double iposition, const std::int32_t ivelocity) const {
  auto out = 1;
  for (auto &&elem : motors) {
    const auto errorCode = elem.moveAbsolute(iposition, ivelocity);
    if (errorCode != 1) {
      out = errorCode;
    }
  }
  return out;
}

std::int32_t MotorGroup::moveRelative(const double iposition, const std::int32_t ivelocity) const {
  auto out = 1;
  for (auto &&elem : motors) {
    const auto errorCode = elem.moveRelative(iposition, ivelocity);
    if (errorCode != 1) {
      out = errorCode;
    }
  }
  return out;
}

std::int32_t MotorGroup::moveVelocity(const std::int16_t ivelocity) const {
  auto out = 1;
  for (auto &&elem : motors) {
    const auto errorCode = elem.moveVelocity(ivelocity);
    if (errorCode != 1) {
      out = errorCode;
    }
  }
  return out;
}

std::int32_t MotorGroup::moveVoltage(const std::int16_t ivoltage) const {
  auto out = 1;
  for (auto &&elem : motors) {
    const auto errorCode = elem.moveVoltage(ivoltage);
    if (errorCode != 1) {
      out = errorCode;
    }
  }
  return out;
}

std::int32_t MotorGroup::modifyProfiledVelocity(std::int32_t ivelocity) const {
  auto out = 1;
  for (auto &&elem : motors) {
    const auto errorCode = elem.modifyProfiledVelocity(ivelocity);
    if (errorCode != 1) {
      out = errorCode;
    }
  }
  return out;
}

double MotorGroup::getTargetPosition() const {
  return motors[0].getTargetPosition();
}

double MotorGroup::getPosition() const {
  return motors[0].getPosition();
}

std::int32_t MotorGroup::tarePosition() const {
  auto out = 1;
  for (auto &&elem : motors) {
    const auto errorCode = elem.tarePosition();
    if (errorCode != 1) {
      out = errorCode;
    }
  }
  return out;
}

std::int32_t MotorGroup::getTargetVelocity() const {
  return motors[0].getTargetVelocity();
}

double MotorGroup::getActualVelocity() const {
  return motors[0].getActualVelocity();
}

std::int32_t MotorGroup::getCurrentDraw() const {
  return motors[0].get_current_draw();
}

std::int32_t MotorGroup::getDirection() const {
  return motors[0].get_direction();
}

double MotorGroup::getEfficiency() const {
  return motors[0].get_efficiency();
}

std::int32_t MotorGroup::isOverCurrent() const {
  return motors[0].is_over_current();
}

std::int32_t MotorGroup::isOverTemp() const {
  return motors[0].is_over_temp();
}

std::int32_t MotorGroup::isStopped() const {
  return motors[0].is_stopped();
}

std::int32_t MotorGroup::getZeroPositionFlag() const {
  return motors[0].get_zero_position_flag();
}

uint32_t MotorGroup::getFaults() const {
  return motors[0].get_faults();
}

uint32_t MotorGroup::getFlags() const {
  return motors[0].get_flags();
}

std::int32_t MotorGroup::getRawPosition(std::uint32_t *timestamp) const {
  return motors[0].get_raw_position(timestamp);
}

double MotorGroup::getPower() const {
  return motors[0].get_power();
}

double MotorGroup::getTemperature() const {
  return motors[0].get_temperature();
}

double MotorGroup::getTorque() const {
  return motors[0].get_torque();
}

std::int32_t MotorGroup::getVoltage() const {
  return motors[0].get_voltage();
}

std::int32_t MotorGroup::setBrakeMode(const AbstractMotor::brakeMode imode) {
  auto out = 1;
  for (auto &&elem : motors) {
    const auto errorCode = elem.setBrakeMode(imode);
    if (errorCode != 1) {
      out = errorCode;
    }
  }
  return out;
}

AbstractMotor::brakeMode MotorGroup::getBrakeMode() const {
  return motors[0].getBrakeMode();
}

std::int32_t MotorGroup::setCurrentLimit(const std::int32_t ilimit) const {
  auto out = 1;
  for (auto &&elem : motors) {
    const auto errorCode = elem.setCurrentLimit(ilimit);
    if (errorCode != 1) {
      out = errorCode;
    }
  }
  return out;
}

std::int32_t MotorGroup::getCurrentLimit() const {
  return motors[0].getCurrentLimit();
}

std::int32_t MotorGroup::setEncoderUnits(const AbstractMotor::encoderUnits iunits) {
  auto out = 1;
  for (auto &&elem : motors) {
    const auto errorCode = elem.setEncoderUnits(iunits);
    if (errorCode != 1) {
      out = errorCode;
    }
  }
  return out;
}

AbstractMotor::encoderUnits MotorGroup::getEncoderUnits() const {
  return motors[0].getEncoderUnits();
}

std::int32_t MotorGroup::setGearing(const AbstractMotor::gearset igearset) {
  auto out = 1;
  for (auto &&elem : motors) {
    const auto errorCode = elem.setGearing(igearset);
    if (errorCode != 1) {
      out = errorCode;
    }
  }
  return out;
}

AbstractMotor::gearset MotorGroup::getGearing() const {
  return motors[0].getGearing();
}

std::int32_t MotorGroup::setReversed(const bool ireverse) const {
  auto out = 1;
  for (auto &&elem : motors) {
    const auto errorCode = elem.setReversed(ireverse);
    if (errorCode != 1) {
      out = errorCode;
    }
  }
  return out;
}

std::int32_t MotorGroup::setVoltageLimit(const std::int32_t ilimit) const {
  auto out = 1;
  for (auto &&elem : motors) {
    const auto errorCode = elem.setVoltageLimit(ilimit);
    if (errorCode != 1) {
      out = errorCode;
    }
  }
  return out;
}

void MotorGroup::controllerSet(const double ivalue) {
  for (auto &&elem : motors) {
    elem.moveVelocity(ivalue);
  }
}

std::int32_t MotorGroup::setPosPID(const double ikF,
                                   const double ikP,
                                   const double ikI,
                                   const double ikD) const {
  auto out = 1;
  for (auto &&elem : motors) {
    const auto errorCode = elem.setPosPID(ikF, ikP, ikI, ikD);
    if (errorCode != 1) {
      out = errorCode;
    }
  }
  return out;
}

std::int32_t MotorGroup::setPosPIDFull(const double ikF,
                                       const double ikP,
                                       const double ikI,
                                       const double ikD,
                                       const double ifilter,
                                       const double ilimit,
                                       const double ithreshold,
                                       const double iloopSpeed) const {
  auto out = 1;
  for (auto &&elem : motors) {
    const auto errorCode =
      elem.setPosPIDFull(ikF, ikP, ikI, ikD, ifilter, ilimit, ithreshold, iloopSpeed);
    if (errorCode != 1) {
      out = errorCode;
    }
  }
  return out;
}

std::int32_t MotorGroup::setVelPID(const double ikF,
                                   const double ikP,
                                   const double ikI,
                                   const double ikD) const {
  auto out = 1;
  for (auto &&elem : motors) {
    const auto errorCode = elem.setVelPID(ikF, ikP, ikI, ikD);
    if (errorCode != 1) {
      out = errorCode;
    }
  }
  return out;
}

std::int32_t MotorGroup::setVelPIDFull(const double ikF,
                                       const double ikP,
                                       const double ikI,
                                       const double ikD,
                                       const double ifilter,
                                       const double ilimit,
                                       const double ithreshold,
                                       const double iloopSpeed) const {
  auto out = 1;
  for (auto &&elem : motors) {
    const auto errorCode =
      elem.setVelPIDFull(ikF, ikP, ikI, ikD, ifilter, ilimit, ithreshold, iloopSpeed);
    if (errorCode != 1) {
      out = errorCode;
    }
  }
  return out;
}

std::shared_ptr<ContinuousRotarySensor> MotorGroup::getEncoder() const {
  return motors[0].getEncoder();
}
} // namespace okapi
