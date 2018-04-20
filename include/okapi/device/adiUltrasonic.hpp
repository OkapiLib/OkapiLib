/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_ADIULTRASONIC_HPP_
#define _OKAPI_ADIULTRASONIC_HPP_

#include "api.h"
#include "okapi/control/controllerInput.hpp"
#include "okapi/filter/medianFilter.hpp"
#include <memory>

namespace okapi {
class ADIUltrasonic : public ControllerInput {
  public:
  /**
   * An ultrasonic sensor in the ADI (3-wire) ports. Uses a 5-tap MedianFilter by default.
   *
   * @param iportTop top port
   * @param iportBottom bottom port
   */
  ADIUltrasonic(const std::uint8_t iportTop, const std::uint8_t iportBottom);

  /**
   * An ultrasonic sensor in the ADI (3-wire) ports.
   *
   * @param iportTop top port
   * @param iportBottom bottom port
   * @param ifilter the filter to use for filtering measurements
   */
  ADIUltrasonic(const std::uint8_t iportTop, const std::uint8_t iportBottom,
                std::unique_ptr<Filter> ifilter);

  virtual ~ADIUltrasonic();

  /**
   * Returns the current filtered sensor value.
   *
   * @return current value
   */
  virtual std::int32_t get();

  /**
   * Get the sensor value for use in a control loop. This method might be automatically called in
   * another thread by the controller. Calls get().
   */
  virtual double controllerGet() override;

  protected:
  pros::ADIUltrasonic ultra;
  std::unique_ptr<Filter> filter;
};
} // namespace okapi

#endif
