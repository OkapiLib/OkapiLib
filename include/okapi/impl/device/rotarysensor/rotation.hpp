/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#pragma once

#include "api.h"
#include "okapi/api/device/rotarysensor/continuousRotarySensor.hpp"

namespace okapi {
class RotationSensor : public ContinuousRotarySensor {
  public:
  /**
   * A rotation sensor in a V5 port.
   *
   * ```cpp
   * auto r = RotationSensor(1, false);
   * auto reversedR = RotationSensor(1, true);
   * ```
   *
   * @param iport The V5 port the device is on
   * @param ireversed Whether the sensor is reversed.
   */
  RotationSensor(std::uint8_t iport, bool ireversed = false);

  /**
   * Get the current sensor value.
   *
   * @return the current sensor value, or `PROS_ERR` on a failure.
   */
  virtual double get() const override;

  /**
   * Reset the sensor to zero.
   *
   * @return `1` on success, `PROS_ERR` on fail
   */
  virtual std::int32_t reset() override;

  /**
   * Get the sensor value for use in a control loop. This method might be automatically called in
   * another thread by the controller.
   *
   * @return the current sensor value, or `PROS_ERR` on a failure.
   */
  virtual double controllerGet() override;

  /**
   * Get the current sensor value for velocity
   *
   * @return the current sensor value, or `PROS_ERR` on a failure.
   */
  double getVelocity() const;

  protected:
  std::uint8_t port;
};
} // namespace okapi
