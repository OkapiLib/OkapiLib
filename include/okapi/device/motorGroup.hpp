/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#ifndef _OKAPI_MOTORGROUP_HPP_
#define _OKAPI_MOTORGROUP_HPP_

#include "okapi/device/abstractMotor.hpp"
#include "okapi/device/motor.hpp"
#include <array>
#include <initializer_list>

namespace okapi {
template <size_t motorNum> class MotorGroup : public AbstractMotor {
  public:
  MotorGroup(const std::array<Motor, motorNum> &imotors) : motors(imotors) {
  }

  int32_t moveAbsolute(const double iposition, const int32_t ivelocity) const override {
    for (size_t i = 0; i < motorNum; i++) {
      motors[i].moveAbsolute(iposition, ivelocity);
    }
  }

  int32_t moveRelative(const double iposition, const int32_t ivelocity) const override {
    for (size_t i = 0; i < motorNum; i++) {
      motors[i].moveRelative(iposition, ivelocity);
    }
  }

  int32_t moveVelocity(const int16_t ivelocity) const override {
    for (size_t i = 0; i < motorNum; i++) {
      motors[i].moveVelocity(ivelocity);
    }
  }

  int32_t moveVoltage(const int16_t ivoltage) const override {
    for (size_t i = 0; i < motorNum; i++) {
      motors[i].moveVoltage(ivoltage);
    }
  }

  private:
  const std::array<Motor, motorNum> motors;
};
} // namespace okapi

#endif
