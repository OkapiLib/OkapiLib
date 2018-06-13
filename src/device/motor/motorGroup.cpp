/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/device/motor/motorGroup.hpp"

namespace okapi {
MotorGroup::MotorGroup(const std::initializer_list<Motor> &imotors) : motors(imotors) {
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

std::int32_t MotorGroup::move(const std::int8_t ivoltage) const {
  auto out = 1;
  for (auto &&elem : motors) {
    const auto errorCode = elem.pros::Motor::move(ivoltage);
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

std::int32_t MotorGroup::getTargetVelocity() const {
  return motors[0].getTargetVelocity();
}

double MotorGroup::getActualVelocity() const {
  return motors[0].getActualVelocity();
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

std::int32_t MotorGroup::setBrakeMode(const pros::c::motor_brake_mode_e_t imode) const {
  auto out = 1;
  for (auto &&elem : motors) {
    const auto errorCode = elem.setBrakeMode(imode);
    if (errorCode != 1) {
      out = errorCode;
    }
  }
  return out;
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

std::int32_t MotorGroup::setEncoderUnits(const pros::c::motor_encoder_units_e_t iunits) const {
  auto out = 1;
  for (auto &&elem : motors) {
    const auto errorCode = elem.setEncoderUnits(iunits);
    if (errorCode != 1) {
      out = errorCode;
    }
  }
  return out;
}

std::int32_t MotorGroup::setGearing(const pros::c::motor_gearset_e_t igearset) const {
  auto out = 1;
  for (auto &&elem : motors) {
    const auto errorCode = elem.setGearing(igearset);
    if (errorCode != 1) {
      out = errorCode;
    }
  }
  return out;
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

IntegratedEncoder MotorGroup::getEncoder() const {
  return motors[0].getEncoder();
}
} // namespace okapi
