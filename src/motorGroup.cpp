/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#include "okapi/device/motorGroup.hpp"

namespace okapi {
MotorGroup::MotorGroup(const std::array<okapi::Motor, motorNum> &imotors)
    : AbstractMotor(imotors[0]), motors(imotors) {
  }

  MotorGroup::~MotorGroup() = default;

  int32_t MotorGroup::move_absolute(const double iposition, const int32_t ivelocity) const override {
    int32_t out = 0;
    
    for (size_t i = 0; i < motorNum; i++) {
      if (motors[i].move_absolute(iposition, ivelocity) == PROS_ERR) {
        out = PROS_ERR;
      }
    }

    return out;
  }

  int32_t MotorGroup::move_relative(const double iposition, const int32_t ivelocity) const override {
    int32_t out = 0;
    
    for (size_t i = 0; i < motorNum; i++) {
      if (motors[i].move_relative(iposition, ivelocity) == PROS_ERR) {
        out = PROS_ERR;
      }
    }

    return out;
  }

  int32_t MotorGroup::move_velocity(const int16_t ivelocity) const override {
    int32_t out = 0;
    
    for (size_t i = 0; i < motorNum; i++) {
      if (motors[i].move_velocity(ivelocity) == PROS_ERR) {
        out = PROS_ERR;
      }
    }

    return out;
  }

  int32_t MotorGroup::move_voltage(const int16_t ivoltage) const override {
    int32_t out = 0;
    
    for (size_t i = 0; i < motorNum; i++) {
      if (motors[i].move_voltage(ivoltage) == PROS_ERR) {
        out = PROS_ERR;
      }
    }

    return out;
  }

  IntegratedEncoder MotorGroup::getEncoder() const override {
    return motors[0].getEncoder();
  }
  }
