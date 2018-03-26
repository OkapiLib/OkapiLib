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

namespace okapi {
class ADIUltrasonic : public ControllerInput {
  public:
  ADIUltrasonic(const uint8_t iportTop, const uint8_t iportBottom);

  virtual ~ADIUltrasonic();

  /**
   * Get the current sensor value. Uses a median filter to remove outliers.
   *
   * @return current value, PROS_ERR on fail
   */
  virtual int32_t get();

  /**
   * Get the sensor value for use in a control loop. This method might be automatically called in
   * another thread by the controller. Uses a median filter to remove outliers.
   */
  virtual double controllerGet() override;

  protected:
  pros::ADIUltrasonic ultra;
  MedianFilter<5> filter;
};
} // namespace okapi

#endif
