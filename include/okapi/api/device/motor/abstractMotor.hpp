/**
  std::unique_ptr<ChassisModel> model =
    std::make_unique<SkidSteerModel>(leftMotor, rightMotor);
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_ABSTRACTMOTOR_HPP_
#define _OKAPI_ABSTRACTMOTOR_HPP_

#include "okapi/api/control/controllerOutput.hpp"
#include "okapi/api/device/rotarysensor/continuousRotarySensor.hpp"
#include <memory>

namespace okapi {
class AbstractMotor : public ControllerOutput {
  public:
  /**
   * Indicates the 'brake mode' of a motor.
   */
  enum class brakeMode {
    coast = 0, // Motor coasts when stopped, traditional behavior
    brake = 1, // Motor brakes when stopped
    hold = 2,  // Motor actively holds position when stopped
    invalid = INT32_MAX
  };

  /**
   * Indicates the units used by the motor encoders.
   */
  enum class encoderUnits { degrees = 0, rotations = 1, counts = 2, invalid = INT32_MAX };

  /**
   * Indicates the internal gear ratio of a motor.
   */
  enum class gearset {
    red = 100,   // 36:1, 100 RPM, Red gear set
    green = 200, // 18:1, 200 RPM, Green gear set
    blue = 600,  // 6:1,  600 RPM, Blue gear set
    invalid = INT32_MAX
  };

  struct GearsetRatioPair {
    GearsetRatioPair(const gearset igearset, const double iratio = 1)
      : internalGearset(igearset), ratio(iratio) {
    }

    ~GearsetRatioPair() = default;

    const gearset internalGearset;
    const double ratio = 1;
  };

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
   * @param iposition The absolute position to move to in the motor's encoder units
   * @param ivelocity The maximum allowable velocity for the movement in RPM
   * @return 1 if the operation was successful or PROS_ERR if the operation failed, setting errno.
   */
  virtual std::int32_t moveAbsolute(double iposition, std::int32_t ivelocity) const = 0;

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
  virtual std::int32_t moveRelative(double iposition, std::int32_t ivelocity) const = 0;

  /**
   * Sets the velocity for the motor.
   *
   * This velocity corresponds to different actual speeds depending on the gearset
   * used for the motor. This results in a range of +-100 for pros::c::red,
   * +-200 for green, and +-600 for blue. The velocity
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
  virtual std::int32_t moveVelocity(std::int16_t ivelocity) const = 0;

  /**
   * Sets the voltage for the motor from -127 to 127.
   *
   * This function uses the following values of errno when an error state is reached:
   * EACCES - Another resource is currently trying to access the port.
   *
   * @param ivoltage The new voltage value from -127 to 127
   * @return 1 if the operation was successful or PROS_ERR if the operation failed, setting errno.
   */
  virtual std::int32_t moveVoltage(std::int16_t ivoltage) const = 0;

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
   * Gets the absolute position of the motor in its encoder units.
   *
   * This function uses the following values of errno when an error state is reached:
   * EACCES - Another resource is currently trying to access the port.
   *
   * @return The motor's absolute position in its encoder units or PROS_ERR_F if the operation
   * failed, setting errno.
   */
  virtual double getPosition() const = 0;

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
   * Gets the actual velocity of the motor.
   *
   * This function uses the following values of errno when an error state is reached:
   * EACCES - Another resource is currently trying to access the port.
   *
   * @return The motor's actual velocity in motor_encoder_units_e_t per second or PROS_ERR_F if the
   * operation failed, setting errno.
   */
  virtual double getActualVelocity() const = 0;

  /**
   * Sets the "absolute" zero position of the motor to its current position.
   *
   * This function uses the following values of errno when an error state is reached:
   * EACCES - Another resource is currently trying to access the port.
   *
   * @return 1 if the operation was successful or PROS_ERR if the operation failed, setting errno.
   */
  virtual std::int32_t tarePosition() const = 0;

  /**
   * Sets one of brakeMode to the motor.
   *
   * This function uses the following values of errno when an error state is reached:
   * EACCES - Another resource is currently trying to access the port.
   *
   * @param imode The new motor brake mode to set for the motor
   * @return 1 if the operation was successful or PROS_ERR if the operation failed, setting errno.
   */
  virtual std::int32_t setBrakeMode(brakeMode imode) const = 0;

  /**
   * Sets the current limit for the motor in mA.
   *
   * This function uses the following values of errno when an error state is reached:
   * EACCES - Another resource is currently trying to access the port.
   *
   * @param ilimit The new current limit in mA
   * @return 1 if the operation was successful or PROS_ERR if the operation failed, setting errno.
   */
  virtual std::int32_t setCurrentLimit(std::int32_t ilimit) const = 0;

  /**
   * Sets one of encoderUnits for the motor encoder.
   *
   * This function uses the following values of errno when an error state is reached:
   * EACCES - Another resource is currently trying to access the port.
   *
   * @param iunits The new motor encoder units
   * @return 1 if the operation was successful or PROS_ERR if the operation failed, setting errno.
   */
  virtual std::int32_t setEncoderUnits(encoderUnits iunits) const = 0;

  /**
   * Sets one of gearset for the motor.
   *
   * This function uses the following values of errno when an error state is reached:
   * EACCES - Another resource is currently trying to access the port.
   *
   * @param igearset The new motor gearset
   * @return 1 if the operation was successful or PROS_ERR if the operation failed, setting errno.
   */
  virtual std::int32_t setGearing(gearset igearset) const = 0;

  /**
   * Sets the reverse flag for the motor.
   *
   * This will invert its movements and the values returned for its position.
   *
   * This function uses the following values of errno when an error state is reached:
   * EACCES - Another resource is currently trying to access the port.
   *
   * @param ireverse True reverses the motor, false is default
   * @return 1 if the operation was successful or PROS_ERR if the operation failed, setting errno.
   */
  virtual std::int32_t setReversed(bool ireverse) const = 0;

  /**
   * Sets the voltage limit for the motor in Volts.
   *
   * This function uses the following values of errno when an error state is reached:
   * EACCES - Another resource is currently trying to access the port.
   *
   * @param ilimit The new voltage limit in Volts
   * @return 1 if the operation was successful or PROS_ERR if the operation failed, setting errno.
   */
  virtual std::int32_t setVoltageLimit(std::int32_t ilimit) const = 0;

  /**
   * Returns the encoder associated with this motor.
   *
   * @return the encoder for this motor
   */
  virtual std::shared_ptr<ContinuousRotarySensor> getEncoder() const = 0;
};

AbstractMotor::GearsetRatioPair operator*(AbstractMotor::gearset gearset, double ratio);

} // namespace okapi

#endif
