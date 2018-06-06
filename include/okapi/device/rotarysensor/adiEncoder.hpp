/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_ADIENCODER_HPP_
#define _OKAPI_ADIENCODER_HPP_

#include "okapi/device/rotarysensor/continuousRotarySensor.hpp"

namespace okapi {
class ADIEncoder : public ContinuousRotarySensor {
  public:
  ADIEncoder(const std::uint8_t iportTop, const std::uint8_t iportBottom,
             const bool ireversed = false);

  /**
   * Get the current sensor value.
   *
   * @return the current sensor value, or ``PROS_ERR`` on a failure.
   */
  virtual std::int32_t get() const override;

  /**
   * Reset the sensor to zero.
   *
   * @return 1 on suceess, PROS_ERR on fail
   */
  virtual std::int32_t reset() const override;

  /**
   * Get the sensor value for use in a control loop. This method might be automatically called in
   * another thread by the controller.
   *
   * @return the current sensor value, or ``PROS_ERR`` on a failure.
   */
  virtual double controllerGet() override;

  protected:
  pros::ADIEncoder enc;
};
} // namespace okapi

#endif
