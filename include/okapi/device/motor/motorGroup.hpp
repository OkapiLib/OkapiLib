/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_MOTORGROUP_HPP_
#define _OKAPI_MOTORGROUP_HPP_

#include "okapi/device/motor/abstractMotor.hpp"
#include "okapi/device/motor/motor.hpp"
#include <array>
#include <initializer_list>

namespace okapi {
template <size_t motorNum> class MotorGroup : public AbstractMotor {
  public:
  MotorGroup(const std::array<okapi::Motor, motorNum> &imotors)
    : AbstractMotor(imotors[0]), motors(imotors) {
    printf("group\n");
  }

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
  virtual std::int32_t moveAbsolute(const double iposition,
                                    const std::int32_t ivelocity) const override {
    auto out = 1;
    for (size_t i = 0; i < motorNum; i++) {
      const auto errorCode = motors[i].move_absolute(iposition, ivelocity);
      if (errorCode != 1) {
        out = errorCode;
      }
    }
    return out;
  }

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
  virtual std::int32_t moveRelative(const double iposition,
                                    const std::int32_t ivelocity) const override {
    auto out = 1;
    for (size_t i = 0; i < motorNum; i++) {
      const auto errorCode = motors[i].move_relative(iposition, ivelocity);
      if (errorCode != 1) {
        out = errorCode;
      }
    }
    return out;
  }

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
  virtual std::int32_t moveVelocity(const std::int16_t ivelocity) const override {
    printf("velocity: %d\n", ivelocity);
    auto out = 1;
    for (size_t i = 0; i < motorNum; i++) {
      const auto errorCode = motors[i].move_velocity(ivelocity);
      if (errorCode != 1) {
        out = errorCode;
      }
    }
    return out;
  }

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
  virtual std::int32_t moveVoltage(const std::int16_t ivoltage) const override {
    printf("voltage: %d\n", ivoltage);
    auto out = 1;
    for (size_t i = 0; i < motorNum; i++) {
      const auto errorCode = motors[i].move_voltage(ivoltage);
      if (errorCode != 1) {
        out = errorCode;
      }
    }
    return out;
  }

  /**
   * Gets the target position set for the motor by the user.
   *
   * This function uses the following values of errno when an error state is reached:
   * EACCES - Another resource is currently trying to access the port.
   *
   * @return The target position in its encoder units or PROS_ERR_F if the operation failed,
   * setting errno.
   */
  virtual double getTargetPosition() const override {
    return motors[0].get_target_position();
  }

  /**
   * Gets the absolute position of the motor in its encoder units.
   *
   * This function uses the following values of errno when an error state is reached:
   * EACCES - Another resource is currently trying to access the port.
   *
   * @return The motor's absolute position in its encoder units or PROS_ERR_F if the operation
   * failed, setting errno.
   */
  virtual double getPosition() const override {
    return motors[0].get_position();
  }

  /**
   * Gets the velocity commanded to the motor by the user.
   *
   * This function uses the following values of errno when an error state is reached:
   * EACCES - Another resource is currently trying to access the port.
   *
   * @return The commanded motor velocity from +-100, +-200, or +-600, or PROS_ERR if the operation
   * failed, setting errno.
   */
  virtual std::int32_t getTargetVelocity() const override {
    return motors[0].get_target_velocity();
  }

  /**
   * Gets the actual velocity of the motor.
   *
   * This function uses the following values of errno when an error state is reached:
   * EACCES - Another resource is currently trying to access the port.
   *
   * @return The motor's actual velocity in motor_encoder_units_e_t per second or PROS_ERR_F if the
   * operation failed, setting errno.
   */
  virtual double getActualVelocity() const override {
    return motors[0].get_actual_velocity();
  }

  /**
   * Sets the "absolute" zero position of the motor to its current position.
   *
   * This function uses the following values of errno when an error state is reached:
   * EACCES - Another resource is currently trying to access the port.
   *
   * @return 1 if the operation was successful or PROS_ERR if the operation failed, setting errno.
   */
  virtual std::int32_t tarePosition() const override {
    auto out = 1;
    for (size_t i = 0; i < motorNum; i++) {
      const auto errorCode = motors[i].tare_position();
      if (errorCode != 1) {
        out = errorCode;
      }
    }
    return out;
  }

  /**
   * Sets one of motor_brake_mode_e_t to the motor.
   *
   * This function uses the following values of errno when an error state is reached:
   * EACCES - Another resource is currently trying to access the port.
   *
   * @param imode The motor_brake_mode_e_t to set for the motor
   * @return 1 if the operation was successful or PROS_ERR if the operation failed, setting errno.
   */
  virtual std::int32_t setBrakeMode(const motor_brake_mode_e_t imode) const override {
    auto out = 1;
    for (size_t i = 0; i < motorNum; i++) {
      const auto errorCode = motors[i].set_brake_mode(imode);
      if (errorCode != 1) {
        out = errorCode;
      }
    }
    return out;
  }

  /**
   * Sets the current limit for the motor in mA.
   *
   * This function uses the following values of errno when an error state is reached:
   * EACCES - Another resource is currently trying to access the port.
   *
   * @param ilimit The new current limit in mA
   * @return 1 if the operation was successful or PROS_ERR if the operation failed, setting errno.
   */
  virtual std::int32_t setCurrentLimit(const std::int32_t ilimit) const override {
    auto out = 1;
    for (size_t i = 0; i < motorNum; i++) {
      const auto errorCode = motors[i].set_current_limit(ilimit);
      if (errorCode != 1) {
        out = errorCode;
      }
    }
    return out;
  }

  /**
   * Sets one of motor_encoder_units_e_t for the motor encoder.
   *
   * This function uses the following values of errno when an error state is reached:
   * EACCES - Another resource is currently trying to access the port.
   *
   * @param iunits The new motor encoder units
   * @return 1 if the operation was successful or PROS_ERR if the operation failed, setting errno.
   */
  virtual std::int32_t setEncoderUnits(const motor_encoder_units_e_t iunits) const override {
    auto out = 1;
    for (size_t i = 0; i < motorNum; i++) {
      const auto errorCode = motors[i].set_encoder_units(iunits);
      if (errorCode != 1) {
        out = errorCode;
      }
    }
    return out;
  }

  /**
   * Sets one of motor_gearset_e_t for the motor.
   *
   * This function uses the following values of errno when an error state is reached:
   * EACCES - Another resource is currently trying to access the port.
   *
   * @param igearset The new motor gearset
   * @return 1 if the operation was successful or PROS_ERR if the operation failed, setting errno.
   */
  virtual std::int32_t setGearing(const motor_gearset_e_t igearset) const override {
    auto out = 1;
    for (size_t i = 0; i < motorNum; i++) {
      const auto errorCode = motors[i].set_gearing(igearset);
      if (errorCode != 1) {
        out = errorCode;
      }
    }
    return out;
  }

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
  virtual std::int32_t setReversed(const bool ireverse) const override {
    auto out = 1;
    for (size_t i = 0; i < motorNum; i++) {
      const auto errorCode = motors[i].set_reversed(ireverse);
      if (errorCode != 1) {
        out = errorCode;
      }
    }
    return out;
  }

  /**
   * Sets the voltage limit for the motor in Volts.
   *
   * This function uses the following values of errno when an error state is reached:
   * EACCES - Another resource is currently trying to access the port.
   *
   * @param ilimit The new voltage limit in Volts
   * @return 1 if the operation was successful or PROS_ERR if the operation failed, setting errno.
   */
  virtual std::int32_t setVoltageLimit(const std::int32_t ilimit) const override {
    auto out = 1;
    for (size_t i = 0; i < motorNum; i++) {
      const auto errorCode = motors[i].set_voltage_limit(ilimit);
      if (errorCode != 1) {
        out = errorCode;
      }
    }
    return out;
  }

  /**
   * Writes the value of the controller output. This method might be automatically called in another
   * thread by the controller.
   *
   * @param ivalue the controller's output
   */
  virtual void controllerSet(const double ivalue) override {
    for (size_t i = 0; i < motorNum; i++) {
      motors[i].move_velocity(ivalue);
    }
  }

  /**
   * Get the encoder associated with this motor.
   *
   * @return encoder for this motor
   */
  virtual IntegratedEncoder getEncoder() const override {
    return motors[0].getEncoder();
  }

  protected:
  const std::array<okapi::Motor, motorNum> motors;
};
} // namespace okapi

#endif
