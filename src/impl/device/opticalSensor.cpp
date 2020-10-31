/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/impl/device/opticalSensor.hpp"

namespace okapi {
OpticalSensor::OpticalSensor(std::uint8_t iport) {
  port = iport;
  pros::c::optical_disable_gesture(port);
}

OpticalSensor::~OpticalSensor() = default;

double OpticalSensor::get() {
  return pros::c::optical_get_hue(port);
}

double OpticalSensor::controllerGet() {
  return get();
}

double OpticalSensor::get_brightness() {
  return pros::c::optical_get_brightness(port);
}

double OpticalSensor::get_saturation() {
  return pros::c::optical_get_saturation(port);
}

int32_t OpticalSensor::get_led_brightness() {
  return pros::c::optical_get_brightness(port);
}

int32_t OpticalSensor::set_led_brightness(int32_t v) {
  return pros::c::optical_set_led_pwm(port, v);
}

int32_t OpticalSensor::get_proximity() {
  return pros::c::optical_get_proximity(port);
}

int32_t OpticalSensor::disable_gesture() {
  return pros::c::optical_disable_gesture(port);
}

int32_t OpticalSensor::enable_gesture() {
  return pros::c::optical_enable_gesture(port);
}

pros::c::optical_rgb_s_t OpticalSensor::get_rgb() {
  return pros::c::optical_get_rgb(port);
}
} // namespace okapi
