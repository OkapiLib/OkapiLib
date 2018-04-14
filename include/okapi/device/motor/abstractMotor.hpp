/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_ABSTRACTMOTOR_HPP_
#define _OKAPI_ABSTRACTMOTOR_HPP_

#include "api.h"
#include "okapi/control/controllerOutput.hpp"
#include "okapi/device/rotarysensor/integratedEncoder.hpp"

namespace okapi {
class AbstractMotor : public ControllerOutput {
  public:
  virtual ~AbstractMotor();

  /**
   * Sets the target absolute position for the motor to move to.
   *
   * This movement is relative to the position of the motor when initialized or
   * the position when it was most recently reset with setZeroPosition().
   *
   * This function uses the following values of errno when an error state is reached:
   * EACCES - Another resource is currently trying to access the port.
   *
   * @param position The absolute position to move to in the motor's encoder units
   * @param velocity The maximum allowable velocity for the movement in RPM
   * @return 1 if the operation was successful or PROS_ERR if the operation failed, setting errno.
   */
  virtual std::int32_t moveAbsolute(const double iposition, const std::int32_t ivelocity) const = 0;

  /**
   * Sets the relative target position for the motor to move to.
   *
   * This movement is relative to the current position of the motor. Providing 10.0 as the position
   * parameter would result in the motor moving clockwise 10 units, no matter what the current
   * position is.
   *
   * This function uses the following values of errno when an error state is reached:
   * EACCES - Another resource is currently trying to access the port.
   *
   * @param iposition The relative position to move to in the motor's encoder units
   * @param ivelocity The maximum allowable velocity for the movement in RPM
   * @return 1 if the operation was successful or PROS_ERR if the operation failed, setting errno.
   */
  virtual std::int32_t moveRelative(const double iposition, const std::int32_t ivelocity) const = 0;

  /**
   * Sets the velocity for the motor.
   *
   * This velocity corresponds to different actual speeds depending on the gearset
   * used for the motor. This results in a range of +-100 for E_MOTOR_GEARSET_36,
   * +-200 for E_MOTOR_GEARSET_18, and +-600 for E_MOTOR_GEARSET_6. The velocity
   * is held with PID to ensure consistent speed, as opposed to setting the motor's
   * voltage.
   *
   * This function uses the following values of errno when an error state is reached:
   * EACCES - Another resource is currently trying to access the port.
   *
   * @param ivelocity The new motor velocity from -+-100, +-200, or +-600 depending on the motor's
   * gearset
   * @return 1 if the operation was successful or PROS_ERR if the operation failed, setting errno.
   */
  virtual std::int32_t moveVelocity(const std::int16_t ivelocity) const = 0;

  /**
   * Sets the voltage for the motor from -127 to 127.
   *
   * This function uses the following values of errno when an error state is reached:
   * EACCES - Another resource is currently trying to access the port.
   *
   * @param iport The V5 port number from 1-21
   * @param ivoltage The new voltage value from -127 to 127
   * @return 1 if the operation was successful or PROS_ERR if the operation failed, setting errno.
   */
  virtual std::int32_t moveVoltage(const std::int16_t ivoltage) const = 0;

  /**
   * Gets the target position set for the motor by the user.
   *
   * This function uses the following values of errno when an error state is reached:
   * EACCES - Another resource is currently trying to access the port.
   *
   * @return The target position in its encoder units or PROS_ERR_F if the operation failed,
   * setting errno.
   */
  virtual double getTargetPosition() const = 0;

  /**
   * Gets the velocity commanded to the motor by the user.
   *
   * This function uses the following values of errno when an error state is reached:
   * EACCES - Another resource is currently trying to access the port.
   *
   * @return The commanded motor velocity from +-100, +-200, or +-600, or PROS_ERR if the operation
   * failed, setting errno.
   */
  virtual std::int32_t getTargetVelocity() const = 0;

  /**
   * Sets the position for the motor in its encoder units.
   *
   * This will be the future reference point for the motor's "absolute" position.
   *
   * This function uses the following values of errno when an error state is reached:
   * EACCES - Another resource is currently trying to access the port.
   *
   * @param position The new reference position in its encoder units
   * @return 1 if the operation was successful or PROS_ERR if the operation failed, setting errno.
   */
  virtual std::int32_t setZeroPosition(const double position) const = 0;

  /**
   * Returns the encoder associated with this motor.
   *
   * @return the encoder for this motor
   */
  virtual IntegratedEncoder getEncoder() const = 0;
};
} // namespace okapi

#endif
