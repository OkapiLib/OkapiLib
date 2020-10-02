/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/impl/device/motor/motor.hpp"
#include "okapi/api/util/mathUtil.hpp"
#include "okapi/impl/device/rotarysensor/integratedEncoder.hpp"
#include <cmath>

namespace okapi {
Motor::Motor(const std::int8_t iport)
  : Motor(std::abs(iport),
          iport < 0,
          AbstractMotor::gearset::green,
          AbstractMotor::encoderUnits::counts) {
}

Motor::Motor(const std::uint8_t iport,
             const bool ireverse,
             const AbstractMotor::gearset igearset,
             const AbstractMotor::encoderUnits iencoderUnits,
             const std::shared_ptr<Logger> &logger)
  : port(iport), reversed(ireverse ? -1 : 1) {
  if (port < 1 || port > 21) {
    std::string msg = "Motor: The port number (" + std::to_string(port) +
                      ") is outside the expected range of values [1, 21].";
    LOG_ERROR(msg);
  }

  if (igearset == AbstractMotor::gearset::invalid) {
    LOG_WARN_S("Motor: The gearset is invalid.");
  }

  if (iencoderUnits == AbstractMotor::encoderUnits::invalid) {
    LOG_WARN_S("Motor: The encoder units are invalid.");
  }

  setGearing(igearset);
  setEncoderUnits(iencoderUnits);
}

std::int32_t Motor::moveAbsolute(const double iposition, const std::int32_t ivelocity) {
  return pros::c::motor_move_absolute(port, iposition * reversed, ivelocity);
}

std::int32_t Motor::moveRelative(const double iposition, const std::int32_t ivelocity) {
  return pros::c::motor_move_relative(port, iposition * reversed, ivelocity);
}

std::int32_t Motor::moveVelocity(const std::int16_t ivelocity) {
  return pros::c::motor_move_velocity(port, ivelocity * reversed);
}

std::int32_t Motor::moveVoltage(const std::int16_t ivoltage) {
  return pros::c::motor_move_voltage(port, ivoltage * reversed);
}

std::int32_t Motor::modifyProfiledVelocity(std::int32_t ivelocity) {
  return pros::c::motor_modify_profiled_velocity(port, ivelocity * reversed);
}

double Motor::getTargetPosition() {
  return pros::c::motor_get_target_position(port) * reversed;
}

double Motor::getPosition() {
  return pros::c::motor_get_position(port) * reversed;
}

std::int32_t Motor::tarePosition() {
  return pros::c::motor_tare_position(port);
}

std::int32_t Motor::getTargetVelocity() {
  return pros::c::motor_get_target_velocity(port) * reversed;
}

double Motor::getActualVelocity() {
  return pros::c::motor_get_actual_velocity(port) * reversed;
}

std::int32_t Motor::getCurrentDraw() {
  return pros::c::motor_get_current_draw(port);
}

std::int32_t Motor::getDirection() {
  return pros::c::motor_get_direction(port) * reversed;
}

double Motor::getEfficiency() {
  return pros::c::motor_get_efficiency(port);
}

std::int32_t Motor::isOverCurrent() {
  return pros::c::motor_is_over_current(port);
}

std::int32_t Motor::isOverTemp() {
  return pros::c::motor_is_over_temp(port);
}

std::int32_t Motor::isStopped() {
  return pros::c::motor_is_stopped(port);
}

std::int32_t Motor::getZeroPositionFlag() {
  return pros::c::motor_get_zero_position_flag(port);
}

uint32_t Motor::getFaults() {
  return pros::c::motor_get_faults(port);
}

uint32_t Motor::getFlags() {
  return pros::c::motor_get_flags(port);
}

std::int32_t Motor::getRawPosition(std::uint32_t *timestamp) {
  return pros::c::motor_get_raw_position(port, timestamp) * reversed;
}

double Motor::getPower() {
  return pros::c::motor_get_power(port);
}

double Motor::getTemperature() {
  return pros::c::motor_get_temperature(port);
}

double Motor::getTorque() {
  return pros::c::motor_get_torque(port);
}

std::int32_t Motor::getVoltage() {
  return pros::c::motor_get_voltage(port) * reversed;
}

std::int32_t Motor::setBrakeMode(const AbstractMotor::brakeMode imode) {
  switch (imode) {
  case AbstractMotor::brakeMode::brake:
    return pros::c::motor_set_brake_mode(port, pros::E_MOTOR_BRAKE_BRAKE);
  case AbstractMotor::brakeMode::coast:
    return pros::c::motor_set_brake_mode(port, pros::E_MOTOR_BRAKE_COAST);
  case AbstractMotor::brakeMode::hold:
    return pros::c::motor_set_brake_mode(port, pros::E_MOTOR_BRAKE_HOLD);
  case AbstractMotor::brakeMode::invalid:
  default:
    return pros::c::motor_set_brake_mode(port, pros::E_MOTOR_BRAKE_INVALID);
  }
}

AbstractMotor::brakeMode Motor::getBrakeMode() {
  switch (pros::c::motor_get_brake_mode(port)) {
  case pros::E_MOTOR_BRAKE_COAST:
    return AbstractMotor::brakeMode::coast;
  case pros::E_MOTOR_BRAKE_BRAKE:
    return AbstractMotor::brakeMode::brake;
  case pros::E_MOTOR_BRAKE_HOLD:
    return AbstractMotor::brakeMode::hold;
  case pros::E_MOTOR_BRAKE_INVALID:
  default:
    return AbstractMotor::brakeMode::invalid;
  }
}

std::int32_t Motor::setCurrentLimit(const std::int32_t ilimit) {
  return pros::c::motor_set_current_limit(port, ilimit);
}

std::int32_t Motor::getCurrentLimit() {
  return pros::c::motor_get_current_limit(port);
}

std::int32_t Motor::setEncoderUnits(const AbstractMotor::encoderUnits iunits) {
  switch (iunits) {
  case AbstractMotor::encoderUnits::counts:
    return pros::c::motor_set_encoder_units(port, pros::E_MOTOR_ENCODER_COUNTS);
  case AbstractMotor::encoderUnits::degrees:
    return pros::c::motor_set_encoder_units(port, pros::E_MOTOR_ENCODER_DEGREES);
  case AbstractMotor::encoderUnits::rotations:
    return pros::c::motor_set_encoder_units(port, pros::E_MOTOR_ENCODER_ROTATIONS);
  case AbstractMotor::encoderUnits::invalid:
  default:
    return pros::c::motor_set_encoder_units(port, pros::E_MOTOR_ENCODER_INVALID);
  }
}

AbstractMotor::encoderUnits Motor::getEncoderUnits() {
  switch (pros::c::motor_get_encoder_units(port)) {
  case pros::E_MOTOR_ENCODER_DEGREES:
    return AbstractMotor::encoderUnits::degrees;
  case pros::E_MOTOR_ENCODER_ROTATIONS:
    return AbstractMotor::encoderUnits::rotations;
  case pros::E_MOTOR_ENCODER_COUNTS:
    return AbstractMotor::encoderUnits::counts;
  case pros::E_MOTOR_ENCODER_INVALID:
  default:
    return AbstractMotor::encoderUnits::invalid;
  }
}

std::int32_t Motor::setGearing(const AbstractMotor::gearset igearset) {
  switch (igearset) {
  case AbstractMotor::gearset::blue:
    return pros::c::motor_set_gearing(port, pros::E_MOTOR_GEARSET_06);
  case AbstractMotor::gearset::green:
    return pros::c::motor_set_gearing(port, pros::E_MOTOR_GEARSET_18);
  case AbstractMotor::gearset::red:
    return pros::c::motor_set_gearing(port, pros::E_MOTOR_GEARSET_36);
  case AbstractMotor::gearset::invalid:
  default:
    return pros::c::motor_set_gearing(port, pros::E_MOTOR_GEARSET_INVALID);
  }
}

AbstractMotor::gearset Motor::getGearing() {
  switch (pros::c::motor_get_gearing(port)) {
  case pros::E_MOTOR_GEARSET_36:
    return AbstractMotor::gearset::red;
  case pros::E_MOTOR_GEARSET_18:
    return AbstractMotor::gearset::green;
  case pros::E_MOTOR_GEARSET_06:
    return AbstractMotor::gearset::blue;
  case pros::E_MOTOR_GEARSET_INVALID:
  default:
    return AbstractMotor::gearset::invalid;
  }
}

std::int32_t Motor::setReversed(const bool ireverse) {
  reversed = ireverse ? -1 : 1;
  return 0;
}

std::int32_t Motor::setVoltageLimit(const std::int32_t ilimit) {
  return pros::c::motor_set_voltage_limit(port, ilimit);
}

std::int32_t
Motor::setPosPID(const double ikF, const double ikP, const double ikI, const double ikD) {
  return pros::c::motor_set_pos_pid(port, pros::c::motor_convert_pid(ikF, ikP, ikI, ikD));
}

std::int32_t Motor::setPosPIDFull(const double ikF,
                                  const double ikP,
                                  const double ikI,
                                  const double ikD,
                                  const double ifilter,
                                  const double ilimit,
                                  const double ithreshold,
                                  const double iloopSpeed) {
  return pros::c::motor_set_pos_pid_full(
    port,
    pros::c::motor_convert_pid_full(ikF, ikP, ikI, ikD, ifilter, ilimit, ithreshold, iloopSpeed));
}

std::int32_t
Motor::setVelPID(const double ikF, const double ikP, const double ikI, const double ikD) {
  return pros::c::motor_set_vel_pid(port, pros::c::motor_convert_pid(ikF, ikP, ikI, ikD));
}

std::int32_t Motor::setVelPIDFull(const double ikF,
                                  const double ikP,
                                  const double ikI,
                                  const double ikD,
                                  const double ifilter,
                                  const double ilimit,
                                  const double ithreshold,
                                  const double iloopSpeed) {
  return pros::c::motor_set_vel_pid_full(
    port,
    pros::c::motor_convert_pid_full(ikF, ikP, ikI, ikD, ifilter, ilimit, ithreshold, iloopSpeed));
}

std::shared_ptr<ContinuousRotarySensor> Motor::getEncoder() {
  return std::make_shared<IntegratedEncoder>(port, isReversed());
}

void Motor::controllerSet(const double ivalue) {
  // Reversing is done by moveVelocity, no need to reverse here
  moveVelocity(ivalue * toUnderlyingType(getGearing()));
}

std::uint8_t Motor::getPort() const {
  return port;
}

bool Motor::isReversed() const {
  return reversed < 0;
}
} // namespace okapi
