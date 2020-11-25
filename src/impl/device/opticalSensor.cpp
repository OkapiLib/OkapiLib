/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/impl/device/opticalSensor.hpp"

namespace okapi {
OpticalSensor::OpticalSensor(const std::uint8_t iport,
                             const OpticalSensorOutput ioutput,
                             const bool idisableGestures,
                             std::unique_ptr<Filter> ifilter)
  : port(iport), output(ioutput), filter(std::move(ifilter)) {
  if (idisableGestures) {
    disableGestures();
  }
}

double OpticalSensor::getSelectedOutput() {
  switch (output) {
  case OpticalSensorOutput::hue:
    return getHue();
  case OpticalSensorOutput::saturation:
    return getSaturation();
  case OpticalSensorOutput::brightness:
    return getBrightness();
  }

  // This should not run
  return PROS_ERR_F;
}

double OpticalSensor::get() {
  return filter->filter(getSelectedOutput());
}

double OpticalSensor::controllerGet() {
  return get();
}

double OpticalSensor::getHue() const {
  return pros::c::optical_get_hue(port);
}

double OpticalSensor::getBrightness() const {
  return pros::c::optical_get_brightness(port);
}

double OpticalSensor::getSaturation() const {
  return pros::c::optical_get_saturation(port);
}

int32_t OpticalSensor::setLedPWM(const std::uint8_t ivalue) const {
  return pros::c::optical_set_led_pwm(port, ivalue);
}

int32_t OpticalSensor::getLedPWM() const {
  return pros::c::optical_get_led_pwm(port);
}

int32_t OpticalSensor::getProximity() const {
  return pros::c::optical_get_proximity(port);
}

int32_t OpticalSensor::enableGestures() const {
  return pros::c::optical_enable_gesture(port);
}

int32_t OpticalSensor::disableGestures() const {
  return pros::c::optical_disable_gesture(port);
}

pros::c::optical_rgb_s_t OpticalSensor::getRGB() const {
  return pros::c::optical_get_rgb(port);
}
} // namespace okapi
