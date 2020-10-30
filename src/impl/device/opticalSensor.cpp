/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/impl/device/opticalSensor.hpp"

namespace okapi {
OpticalSensor::OpticalSensor(std::uint8_t iport) : opt(iport) {
  opt.disable_gesture();
}

OpticalSensor::~OpticalSensor() = default;

double OpticalSensor::get() {
  return opt.get_hue();
}

double OpticalSensor::controllerGet() {
  return get();
}

double OpticalSensor::get_brightness() {
  return opt.get_brightness();
}

double OpticalSensor::get_saturation() {
  return opt.get_saturation();
}

int32_t OpticalSensor::get_led_brightness() {
  return opt.get_led_pwm();
}

int32_t OpticalSensor::set_led_brightness(int32_t v) {
  return opt.set_led_pwm(v);
}

int32_t OpticalSensor::get_proximity() {
  return opt.get_proximity();
}

int32_t OpticalSensor::disable_gesture() {
  return opt.disable_gesture();
}

int32_t OpticalSensor::enable_gesture() {
  return opt.enable_gesture();
}

pros::c::optical_rgb_s_t OpticalSensor::get_rgb() {
  return opt.get_rgb();
}
} // namespace okapi
