/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#pragma once

#include "api.h"
#include "okapi/api/control/controllerInput.hpp"
#include "okapi/api/filter/passthroughFilter.hpp"
#include <memory>

namespace okapi {
class OpticalSensor : public ControllerInput<double> {
  public:
  /**
   * An optical sensor on a V5 port.
   *
   * ```cpp
   * auto os = OpticalSensor(1);
   * ```
   *
   * @param iport The V5 port the device uses
   */
  OpticalSensor(std::uint8_t iport);

  virtual ~OpticalSensor();

  /**
   * Returns the current sensor's hue value (0-359.99).
   *
   * @return current value
   */
  virtual double get();

  /**
   * Returns the current sensor's brightness value (0-1.0).
   *
   * @return current value
   */
  double getBrightness();

  /**
   * Returns the current sensor's saturation value (0-1.0).
   *
   * @return current value
   */
  double getSaturation();

  /**
   * Returns the current sensor's LED brightness
   *
   * value that ranges from 0 to 100
   *
   * @return current value
   */
  int32_t getLedBrightness();

  /**
   * Set the pwm value of the White LED on the sensor
   *
   * value that ranges from 0 to 100
   */
  int32_t setLedBrightness(int32_t v);

  /**
   * This is not available if gestures are being detected. proximity has a range of 0 to 255.
   *
   * This function uses the following values of errno when an error state is reached:
   * ENXIO - The given value is not within the range of V5 ports (1-21).
   * ENODEV - The port cannot be configured as an Optical Sensor
   *
   * @return poximity value if the operation was successful or PROS_ERR if
   * the operation failed, setting errno.
   */
  int32_t getProximity();

  /**
   * Get the processed RGBC data from the sensor
   *
   * This function uses the following values of errno when an error state is reached:
   * ENXIO - The given value is not within the range of V5 ports (1-21).
   * ENODEV - The port cannot be configured as an Optical Sensor
   *
   * @return rgb value if the operation was successful or an optical_rgb_s_t
   * with all fields set to PROS_ERR if the operation failed, setting errno.
   */
  pros::c::optical_rgb_s_t getRGB();

  /**
   * Get the sensor value for use in a control loop. This method might be automatically 
   * called in another thread by the controller. Calls get().
   */
  virtual double controllerGet() override;

  /**
   * Disables gestures
   *
   * @return current value
   */
  int32_t disableGesture();

  /**
   * Enables gestures
   *
   * @return current value
   */
  int32_t enableGesture();

  protected:
  std::uint8_t port;
};
} // namespace okapi
