/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_GYRO_HPP_
#define _OKAPI_GYRO_HPP_

#include "api.h"
#include "okapi/api/control/controllerInput.hpp"
#include "okapi/api/device/rotarysensor/continuousRotarySensor.hpp"

namespace okapi {
class ADIGyro : public ContinuousRotarySensor {
  public:
  /**
   * A gyroscope on the given ADI port. If the port has not previously been configured as a gyro,
   * then the constructor will block for 1 second for calibration. The gyro measures in tenths of a
   * degree, so there are 3600 measurement points per revolution.
   *
   * @param iport the ADI port number
   * @param imultiplier a value multiplied by the gyro heading value
   */
  ADIGyro(std::uint8_t iport, double imultiplier = 1);

  virtual ~ADIGyro();

  /**
   * Get the current sensor value.
   *
   * @return the current sensor value, or ``PROS_ERR`` on a failure.
   */
  double get() const override;

  /**
   * Reset the sensor to zero.
   *
   * @return 1 on success, PROS_ERR on fail
   */
  std::int32_t reset() override;

  /**
   * Get the sensor value for use in a control loop. This method might be automatically called in
   * another thread by the controller.
   *
   * @return the current sensor value, or ``PROS_ERR`` on a failure.
   */
  double controllerGet() override;

  protected:
  pros::ADIGyro gyro;
};
} // namespace okapi

#endif
