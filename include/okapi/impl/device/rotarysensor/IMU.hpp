/*
 * @author Alex Riensche, UNL
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#pragma once

#include "api.h"
#include "okapi/api/control/controllerInput.hpp"
#include "okapi/api/device/rotarysensor/continuousRotarySensor.hpp"

namespace okapi {
enum class IMUAxes {
  z, ///< Yaw Axis
  y, ///< Pitch Axis
  x  ///< Roll Axis
};

class IMU : public ContinuousRotarySensor {
  public:
  /**
   * An inertial sensor on the given port. The IMU returns an angle about the selected axis in
   * degrees.
   *
   * @param iport The port to use the inertial sensor from.
   * @param iaxis The axis of the inertial sensor to measure, default `IMUAxes::z`.
   */
  IMU(std::uint8_t iport, IMUAxes iaxis = IMUAxes::z);

  virtual ~IMU();

  /**
   * Get the current rotation about `iaxis`.
   *
   * This may set the following values of `errno` on error:
   * - `ENXIO` - The given `iport` is not in range of the V5 ports (1-21).
   * - `ENODEV` - The port cannot be configured as an IMU.
   * - `EAGAIN` - The sensor is calibrating.
   *
   * @return The current sensor value or `PROS_ERR`.
   */
  double get() const override;

  /**
   * Get the current sensor value remapped into the target range (`[-1800, 1800]` by default).
   *
   * @param iupperBound The upper bound of the range.
   * @param ilowerBound The lower bound of the range.
   * @return The remapped sensor value.
   */
  double getRemapped(double iupperBound = 1800, double ilowerBound = -1800) const;

  /**
   * Get the current acceleration along `iaxis`.
   *
   * This may set the following values of `errno` on error:
   * - `ENXIO` - The given `iport` is not in range of the V5 ports (1-21).
   * - `ENODEV` - The port cannot be configured as an IMU.
   * - `EAGAIN` - The sensor is calibrating.
   *
   * @return The current sensor value or `PROS_ERR`.
   */
  double getAcceleration();

  /**
   * Reset the rotation value to zero.
   *
   * @return This always returns `1`.
   */
  std::int32_t reset() override;

  /**
   * Calibrate the IMU. Resets the rotation value to zero. Calibration is expected to take two
   * seconds, but is bounded to five seconds.
   *
   * This may set the following values of `errno` on error:
   * - `ENXIO` - The given `iport` is not in range of the V5 ports (1-21).
   * - `ENODEV` - The port cannot be configured as an IMU.
   * - `EAGAIN` - The sensor is calibrating.
   *
   * @return `1` or `PROS_ERR`.
   */
  std::int32_t calibrate();

  /**
   * Get the sensor value for use in a control loop. This method might be automatically called in
   * another thread by the controller.
   *
   * This may set the following values of `errno` on error:
   * - `ENXIO` - The given `iport` is not in range of the V5 ports (1-21).
   * - `ENODEV` - The port cannot be configured as an IMU.
   * - `EAGAIN` - The sensor is calibrating.
   *
   * @return The current sensor value or `PROS_ERR`.
   */
  double controllerGet() override;

  protected:
  std::uint8_t port;
  IMUAxes axis;

  /**
   * Get the current rotation about `iaxis` from the IMU. The internal offset is not accounted for.
   *
   * This may set the following values of `errno` on error:
   * - `ENXIO` - The given `iport` is not in range of the V5 ports (1-21).
   * - `ENODEV` - The port cannot be configured as an IMU.
   * - `EAGAIN` - The sensor is calibrating.
   *
   * @return The current sensor value or `PROS_ERR`.
   */
  double readAngle() const;

  /**
   * @return Whether the IMU is calibrating.
   */
  bool isCalibrating() const;

  private:
  double offset = 0;
};
} // namespace okapi
