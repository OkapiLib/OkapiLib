/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#pragma once

#include "api.h"
#include "okapi/api/device/motor/abstractMotor.hpp"

namespace okapi {
class Motor : public AbstractMotor, public pros::Motor {
  public:
  /**
   * A V5 motor. A negative port number is shorthand for reversing the motor.
   *
   * @param port the port number
   */
  Motor(std::int8_t port);

  explicit Motor(std::uint8_t port,
                 bool reverse,
                 AbstractMotor::gearset igearset,
                 AbstractMotor::encoderUnits encoderUnits = AbstractMotor::encoderUnits::degrees);

  /******************************************************************************/
  /**                         Motor movement functions                         **/
  /**                                                                          **/
  /**          These functions allow programmers to make motors move           **/
  /******************************************************************************/

  /**
   * Sets the target absolute position for the motor to move to.
   *
   * This movement is relative to the position of the motor when initialized or
   * the position when it was most recently reset with setZeroPosition().
   *
   * @note This function simply sets the target for the motor, it does not block program execution
   * until the movement finishes.
   *
   * This function uses the following values of errno when an error state is reached:
   * EACCES - Another resource is currently trying to access the port.
   *
   * @param iposition The absolute position to move to in the motor's encoder units
   * @param ivelocity The maximum allowable velocity for the movement in RPM
   * @return 1 if the operation was successful or PROS_ERR if the operation failed, setting errno.
   */
  virtual std::int32_t moveAbsolute(double iposition, std::int32_t ivelocity) override;

  /**
   * Sets the relative target position for the motor to move to.
   *
   * This movement is relative to the current position of the motor. Providing 10.0 as the position
   * parameter would result in the motor moving clockwise 10 units, no matter what the current
   * position is.
   *
   * @note This function simply sets the target for the motor, it does not block program execution
   * until the movement finishes.
   *
   * This function uses the following values of errno when an error state is reached:
   * EACCES - Another resource is currently trying to access the port.
   *
   * @param iposition The relative position to move to in the motor's encoder units
   * @param ivelocity The maximum allowable velocity for the movement in RPM
   * @return 1 if the operation was successful or PROS_ERR if the operation failed, setting errno.
   */
  virtual std::int32_t moveRelative(double iposition, std::int32_t ivelocity) override;

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
  virtual std::int32_t moveVelocity(std::int16_t ivelocity) override;

  /**
   * Sets the voltage for the motor from -12000 to 12000.
   *
   * This function uses the following values of errno when an error state is reached:
   * EACCES - Another resource is currently trying to access the port.
   *
   * @param ivoltage The new voltage value from -12000 to 12000.
   * @return 1 if the operation was successful or PROS_ERR if the operation failed, setting errno.
   */
  virtual std::int32_t moveVoltage(std::int16_t ivoltage) override;

  /**
   * Changes the output velocity for a profiled movement (moveAbsolute or moveRelative). This will
   * have no effect if the motor is not following a profiled movement.
   *
   * This function uses the following values of errno when an error state is reached:
   * EACCES - Another resource is currently trying to access the port.
   *
   * @param ivelocity The new motor velocity from -+-100, +-200, or +-600 depending on the motor's
   * gearset
   * @return 1 if the operation was successful or PROS_ERR if the operation failed, setting errno.
   */
  virtual std::int32_t modifyProfiledVelocity(std::int32_t ivelocity) override;

  /******************************************************************************/
  /**                        Motor telemetry functions                         **/
  /**                                                                          **/
  /**    These functions allow programmers to collect telemetry from motors    **/
  /******************************************************************************/

  /**
   * Gets the target position set for the motor by the user.
   *
   * This function uses the following values of errno when an error state is reached:
   * EACCES - Another resource is currently trying to access the port.
   *
   * @return The target position in its encoder units or PROS_ERR_F if the operation failed,
   * setting errno.
   */
  virtual double getTargetPosition() override;

  /**
   * Gets the absolute position of the motor in its encoder units.
   *
   * This function uses the following values of errno when an error state is reached:
   * EACCES - Another resource is currently trying to access the port.
   *
   * @return The motor's absolute position in its encoder units or PROS_ERR_F if the operation
   * failed, setting errno.
   */
  virtual double getPosition() override;

  /**
   * Sets the "absolute" zero position of the motor to its current position.
   *
   * This function uses the following values of errno when an error state is reached:
   * EACCES - Another resource is currently trying to access the port.
   *
   * @return 1 if the operation was successful or PROS_ERR if the operation failed, setting errno.
   */
  virtual std::int32_t tarePosition() override;

  /**
   * Gets the velocity commanded to the motor by the user.
   *
   * This function uses the following values of errno when an error state is reached:
   * EACCES - Another resource is currently trying to access the port.
   *
   * @return The commanded motor velocity from +-100, +-200, or +-600, or PROS_ERR if the operation
   * failed, setting errno.
   */
  virtual std::int32_t getTargetVelocity() override;

  /**
   * Gets the actual velocity of the motor.
   *
   * This function uses the following values of errno when an error state is reached:
   * EACCES - Another resource is currently trying to access the port.
   *
   * @return The motor's actual velocity in RPM or PROS_ERR_F if the operation failed, setting
   * errno.
   */
  virtual double getActualVelocity() override;

  /**
   * Gets the current drawn by the motor in mA.
   *
   * This function uses the following values of errno when an error state is
   * reached:
   * EACCES - Another resource is currently trying to access the port.
   *
   * @return The motor's current in mA or PROS_ERR if the operation failed, setting errno.
   */
  virtual std::int32_t getCurrentDraw() override;

  /**
   * Gets the direction of movement for the motor.
   *
   * This function uses the following values of errno when an error state is
   * reached:
   * EACCES - Another resource is currently trying to access the port.
   *
   * @return 1 for moving in the positive direction, -1 for moving in the negative direction, and
   * PROS_ERR if the operation failed, setting errno.
   */
  virtual std::int32_t getDirection() override;

  /**
   * Gets the efficiency of the motor in percent.
   *
   * An efficiency of 100% means that the motor is moving electrically while
   * drawing no electrical power, and an efficiency of 0% means that the motor
   * is drawing power but not moving.
   *
   * This function uses the following values of errno when an error state is
   * reached:
   * EACCES - Another resource is currently trying to access the port.
   *
   * @return The motor's efficiency in percent or PROS_ERR_F if the operation failed, setting errno.
   */
  virtual double getEfficiency() override;

  /**
   * Checks if the motor is drawing over its current limit.
   *
   * This function uses the following values of errno when an error state is
   * reached:
   * EACCES - Another resource is currently trying to access the port.
   *
   * @return 1 if the motor's current limit is being exceeded and 0 if the current limit is not
   * exceeded, or PROS_ERR if the operation failed, setting errno.
   */
  virtual std::int32_t isOverCurrent() override;

  /**
   * Checks if the motor's temperature is above its limit.
   *
   * This function uses the following values of errno when an error state is
   * reached:
   * EACCES - Another resource is currently trying to access the port.
   *
   * @return 1 if the temperature limit is exceeded and 0 if the the temperature is below the limit,
   * or PROS_ERR if the operation failed, setting errno.
   */
  virtual std::int32_t isOverTemp() override;

  /**
   * Checks if the motor is stopped.
   *
   * Although this function forwards data from the motor, the motor presently does not provide any
   * value. This function returns PROS_ERR with errno set to ENOSYS.
   * @return 1 if the motor is not moving, 0 if the motor is moving, or PROS_ERR if the operation
   * failed, setting errno
   */
  virtual std::int32_t isStopped() override;

  /**
   * Checks if the motor is at its zero position.
   *
   * Although this function forwards data from the motor, the motor presently does not provide any
   * value. This function returns PROS_ERR with errno set to ENOSYS.
   *
   * @return 1 if the motor is at zero absolute position, 0 if the motor has moved from its absolute
   * zero, or PROS_ERR if the operation failed, setting errno
   */
  virtual std::int32_t getZeroPositionFlag() override;

  /**
   * Gets the faults experienced by the motor. Compare this bitfield to the bitmasks in
   * pros::motor_fault_e_t.
   *
   * This function uses the following values of errno when an error state is
   * reached:
   * EACCES - Another resource is currently trying to access the port.
   *
   * @return A currently unknown bitfield containing the motor's faults. 0b00000100 = Current Limit
   * Hit
   */
  virtual uint32_t getFaults() override;

  /**
   * Gets the flags set by the motor's operation. Compare this bitfield to the bitmasks in
   * pros::motor_flag_e_t.
   *
   * This function uses the following values of errno when an error state is
   * reached:
   * EACCES - Another resource is currently trying to access the port.
   *
   * @return A currently unknown bitfield containing the motor's flags. These seem to be unrelated
   * to the individual get_specific_flag functions
   */
  virtual uint32_t getFlags() override;

  /**
   * Gets the raw encoder count of the motor at a given timestamp.
   *
   * This function uses the following values of errno when an error state is
   * reached:
   * EACCES - Another resource is currently trying to access the port.
   *
   * @param timestamp A pointer to a time in milliseconds for which the encoder count will be
   * returned. If NULL, the timestamp at which the encoder count was read will not be supplied
   *
   * @return The raw encoder count at the given timestamp or PROS_ERR if the operation failed.
   */
  virtual std::int32_t getRawPosition(std::uint32_t *timestamp) override;

  /**
   * Gets the power drawn by the motor in Watts.
   *
   * This function uses the following values of errno when an error state is
   * reached:
   * EACCES - Another resource is currently trying to access the port.
   *
   * @return The motor's power draw in Watts or PROS_ERR_F if the operation failed, setting errno.
   */
  virtual double getPower() override;

  /**
   * Gets the temperature of the motor in degrees Celsius.
   *
   * This function uses the following values of errno when an error state is
   * reached:
   * EACCES - Another resource is currently trying to access the port.
   *
   * @return The motor's temperature in degrees Celsius or PROS_ERR_F if the operation failed,
   * setting errno.
   */
  virtual double getTemperature() override;

  /**
   * Gets the torque generated by the motor in Newton Metres (Nm).
   *
   * This function uses the following values of errno when an error state is
   * reached:
   * EACCES - Another resource is currently trying to access the port.
   *
   * @return The motor's torque in NM or PROS_ERR_F if the operation failed, setting errno.
   */
  virtual double getTorque() override;

  /**
   * Gets the voltage delivered to the motor in millivolts.
   *
   * This function uses the following values of errno when an error state is
   * reached:
   * EACCES - Another resource is currently trying to access the port.
   *
   * @return The motor's voltage in V or PROS_ERR_F if the operation failed, setting errno.
   */
  virtual std::int32_t getVoltage() override;

  /******************************************************************************/
  /**                      Motor configuration functions                       **/
  /**                                                                          **/
  /**  These functions allow programmers to configure the behavior of motors   **/
  /******************************************************************************/

  /**
   * Sets one of AbstractMotor::brakeMode to the motor.
   *
   * This function uses the following values of errno when an error state is reached:
   * EACCES - Another resource is currently trying to access the port.
   *
   * @param imode The new motor brake mode to set for the motor
   * @return 1 if the operation was successful or PROS_ERR if the operation failed, setting errno.
   */
  virtual std::int32_t setBrakeMode(AbstractMotor::brakeMode imode) override;

  /**
   * Gets the brake mode that was set for the motor.
   *
   * This function uses the following values of errno when an error state is reached:
   * EACCES - Another resource is currently trying to access the port.
   *
   * @return One of brakeMode, according to what was set for the motor, or brakeMode::invalid if the
   * operation failed, setting errno.
   */
  virtual brakeMode getBrakeMode() override;

  /**
   * Sets the current limit for the motor in mA.
   *
   * This function uses the following values of errno when an error state is reached:
   * EACCES - Another resource is currently trying to access the port.
   *
   * @param ilimit The new current limit in mA
   * @return 1 if the operation was successful or PROS_ERR if the operation failed, setting errno.
   */
  virtual std::int32_t setCurrentLimit(std::int32_t ilimit) override;

  /**
   * Gets the current limit for the motor in mA.
   *
   * The default value is 2500 mA.
   *
   * This function uses the following values of errno when an error state is reached:
   * EACCES - Another resource is currently trying to access the port.
   *
   * @return The motor's current limit in mA or PROS_ERR if the operation failed, setting errno.
   */
  virtual std::int32_t getCurrentLimit() override;

  /**
   * Sets one of AbstractMotor::encoderUnits for the motor encoder.
   *
   * This function uses the following values of errno when an error state is reached:
   * EACCES - Another resource is currently trying to access the port.
   *
   * @param iunits The new motor encoder units
   * @return 1 if the operation was successful or PROS_ERR if the operation failed, setting errno.
   */
  virtual std::int32_t setEncoderUnits(AbstractMotor::encoderUnits iunits) override;

  /**
   * Gets the encoder units that were set for the motor.
   *
   * This function uses the following values of errno when an error state is reached:
   * EACCES - Another resource is currently trying to access the port.
   *
   * @return One of encoderUnits according to what is set for the motor or encoderUnits::invalid if
   * the operation failed.
   */
  virtual encoderUnits getEncoderUnits() override;

  /**
   * Sets one of AbstractMotor::gearset for the motor.
   *
   * This function uses the following values of errno when an error state is reached:
   * EACCES - Another resource is currently trying to access the port.
   *
   * @param igearset The new motor gearset
   * @return 1 if the operation was successful or PROS_ERR if the operation failed, setting errno.
   */
  virtual std::int32_t setGearing(AbstractMotor::gearset igearset) override;

  /**
   * Gets the gearset that was set for the motor.
   *
   * This function uses the following values of errno when an error state is reached:
   * EACCES - Another resource is currently trying to access the port.
   *
   * @return One of gearset according to what is set for the motor, or gearset::invalid if the
   * operation failed.
   */
  virtual gearset getGearing() override;

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
  virtual std::int32_t setReversed(bool ireverse) override;

  /**
   * Sets the voltage limit for the motor in Volts.
   *
   * This function uses the following values of errno when an error state is reached:
   * EACCES - Another resource is currently trying to access the port.
   *
   * @param ilimit The new voltage limit in Volts
   * @return 1 if the operation was successful or PROS_ERR if the operation failed, setting errno.
   */
  virtual std::int32_t setVoltageLimit(std::int32_t ilimit) override;

  /**
   * Sets new PID constants.
   *
   * @param ikF the feed-forward constant
   * @param ikP the proportional constant
   * @param ikI the integral constant
   * @param ikD the derivative constant
   * @return 1 if the operation was successful or PROS_ERR if the operation failed, setting errno.
   */
  virtual std::int32_t setPosPID(double ikF, double ikP, double ikI, double ikD) override;

  /**
   * Sets new PID constants.
   *
   * @param ikF the feed-forward constant
   * @param ikP the proportional constant
   * @param ikI the integral constant
   * @param ikD the derivative constant
   * @param ifilter a constant used for filtering the profile acceleration
   * @param ilimit the integral limit
   * @param ithreshold the threshold for determining if a position movement has reached its goal
   * @param iloopSpeed the rate at which the PID computation is run (in ms)
   * @return 1 if the operation was successful or PROS_ERR if the operation failed, setting errno.
   */
  virtual std::int32_t setPosPIDFull(double ikF,
                                     double ikP,
                                     double ikI,
                                     double ikD,
                                     double ifilter,
                                     double ilimit,
                                     double ithreshold,
                                     double iloopSpeed) override;

  /**
   * Sets new PID constants.
   *
   * @param ikF the feed-forward constant
   * @param ikP the proportional constant
   * @param ikI the integral constant
   * @param ikD the derivative constant
   * @return 1 if the operation was successful or PROS_ERR if the operation failed, setting errno.
   */
  virtual std::int32_t setVelPID(double ikF, double ikP, double ikI, double ikD) override;

  /**
   * Sets new PID constants.
   *
   * @param ikF the feed-forward constant
   * @param ikP the proportional constant
   * @param ikI the integral constant
   * @param ikD the derivative constant
   * @param ifilter a constant used for filtering the profile acceleration
   * @param ilimit the integral limit
   * @param ithreshold the threshold for determining if a position movement has reached its goal
   * @param iloopSpeed the rate at which the PID computation is run (in ms)
   * @return 1 if the operation was successful or PROS_ERR if the operation failed, setting errno.
   */
  virtual std::int32_t setVelPIDFull(double ikF,
                                     double ikP,
                                     double ikI,
                                     double ikD,
                                     double ifilter,
                                     double ilimit,
                                     double ithreshold,
                                     double iloopSpeed) override;

  /**
   * Get the encoder associated with this motor.
   *
   * @return encoder for this motor
   */
  virtual std::shared_ptr<ContinuousRotarySensor> getEncoder() override;

  /**
   * Writes the value of the controller output. This method might be automatically called in another
   * thread by the controller. The range of input values is expected to be [-1, 1].
   *
   * @param ivalue the controller's output in the range [-1, 1]
   */
  virtual void controllerSet(double ivalue) override;

  protected:
  AbstractMotor::gearset gearset{AbstractMotor::gearset::green};
};

inline namespace literals {
/**
 * Non-reversed motor.
 **/
okapi::Motor operator"" _mtr(unsigned long long iport);

/**
 * Reversed motor.
 **/
okapi::Motor operator"" _rmtr(unsigned long long iport);
} // namespace literals
} // namespace okapi
