/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/impl/device/motor/motor.hpp"
#include "okapi/api/util/mathUtil.hpp"
#include "okapi/impl/device/rotarysensor/integratedEncoder.hpp"
#include <cmath>

namespace okapi {
Motor::Motor(const std::int8_t port)
  : Motor(std::abs(port), port < 0, AbstractMotor::gearset::red) {
}

Motor::Motor(const std::uint8_t port,
             const bool reverse,
             const AbstractMotor::gearset igearset,
             const AbstractMotor::encoderUnits encoderUnits)
  : pros::Motor(port,
                gearset == AbstractMotor::gearset::red
                  ? pros::E_MOTOR_GEARSET_36
                  : gearset == AbstractMotor::gearset::green
                      ? pros::E_MOTOR_GEARSET_18
                      : gearset == AbstractMotor::gearset::blue ? pros::E_MOTOR_GEARSET_06
                                                                : pros::E_MOTOR_GEARSET_INVALID,
                reverse,
                encoderUnits == AbstractMotor::encoderUnits::counts
                  ? pros::E_MOTOR_ENCODER_COUNTS
                  : encoderUnits == AbstractMotor::encoderUnits::degrees
                      ? pros::E_MOTOR_ENCODER_DEGREES
                      : encoderUnits == AbstractMotor::encoderUnits::rotations
                          ? pros::E_MOTOR_ENCODER_ROTATIONS
                          : pros::E_MOTOR_ENCODER_INVALID),
    gearset(igearset) {
}

std::int32_t Motor::moveAbsolute(const double iposition, const std::int32_t ivelocity) const {
  return move_absolute(iposition, ivelocity);
}

std::int32_t Motor::moveRelative(const double iposition, const std::int32_t ivelocity) const {
  return move_relative(iposition, ivelocity);
}

std::int32_t Motor::moveVelocity(const std::int16_t ivelocity) const {
  return move_velocity(ivelocity);
}

std::int32_t Motor::moveVoltage(const std::int16_t ivoltage) const {
  return move_voltage(ivoltage);
}

std::int32_t Motor::modifyProfiledVelocity(std::int32_t ivelocity) const {
  return modify_profiled_velocity(ivelocity);
}

double Motor::getTargetPosition() const {
  return get_target_position();
}

double Motor::getPosition() const {
  return get_position();
}

std::int32_t Motor::tarePosition() const {
  return tare_position();
}

std::int32_t Motor::getTargetVelocity() const {
  return get_target_velocity();
}

double Motor::getActualVelocity() const {
  return get_actual_velocity();
}

std::int32_t Motor::getCurrentDraw() const {
  return get_current_draw();
}

std::int32_t Motor::getDirection() const {
  return get_direction();
}

double Motor::getEfficiency() const {
  return get_efficiency();
}

std::int32_t Motor::isOverCurrent() const {
  return is_over_current();
}

std::int32_t Motor::isOverTemp() const {
  return is_over_temp();
}

std::int32_t Motor::isStopped() const {
  return is_stopped();
}

std::int32_t Motor::getZeroPositionFlag() const {
  return get_zero_position_flag();
}

uint32_t Motor::getFaults() const {
  return get_faults();
}

uint32_t Motor::getFlags() const {
  return get_flags();
}

std::int32_t Motor::getRawPosition(std::uint32_t *timestamp) const {
  return get_raw_position(timestamp);
}

double Motor::getPower() const {
  return get_power();
}

double Motor::getTemperature() const {
  return get_temperature();
}

double Motor::getTorque() const {
  return get_torque();
}

std::int32_t Motor::getVoltage() const {
  return get_voltage();
}

std::int32_t Motor::setBrakeMode(const AbstractMotor::brakeMode imode) {
  switch (imode) {
  case AbstractMotor::brakeMode::brake:
    return set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  case AbstractMotor::brakeMode::coast:
    return set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  case AbstractMotor::brakeMode::hold:
    return set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  case AbstractMotor::brakeMode::invalid:
    return set_brake_mode(pros::E_MOTOR_BRAKE_INVALID);
  }
}

AbstractMotor::brakeMode Motor::getBrakeMode() const {
  switch (get_brake_mode()) {
  case pros::E_MOTOR_BRAKE_COAST:
    return AbstractMotor::brakeMode::coast;
  case pros::E_MOTOR_BRAKE_BRAKE:
    return AbstractMotor::brakeMode::brake;
  case pros::E_MOTOR_BRAKE_HOLD:
    return AbstractMotor::brakeMode::hold;
  case pros::E_MOTOR_BRAKE_INVALID:
    return AbstractMotor::brakeMode::invalid;
  }
}

std::int32_t Motor::setCurrentLimit(const std::int32_t ilimit) const {
  return set_current_limit(ilimit);
}

std::int32_t Motor::getCurrentLimit() const {
  return get_current_limit();
}

std::int32_t Motor::setEncoderUnits(const AbstractMotor::encoderUnits iunits) {
  switch (iunits) {
  case AbstractMotor::encoderUnits::counts:
    return set_encoder_units(pros::E_MOTOR_ENCODER_COUNTS);
  case AbstractMotor::encoderUnits::degrees:
    return set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
  case AbstractMotor::encoderUnits::rotations:
    return set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);
  case AbstractMotor::encoderUnits::invalid:
    return set_encoder_units(pros::E_MOTOR_ENCODER_INVALID);
  }
}

AbstractMotor::encoderUnits Motor::getEncoderUnits() const {
  switch (get_encoder_units()) {
  case pros::E_MOTOR_ENCODER_DEGREES:
    return AbstractMotor::encoderUnits::degrees;
  case pros::E_MOTOR_ENCODER_ROTATIONS:
    return AbstractMotor::encoderUnits::rotations;
  case pros::E_MOTOR_ENCODER_COUNTS:
    return AbstractMotor::encoderUnits::counts;
  case pros::E_MOTOR_ENCODER_INVALID:
    return AbstractMotor::encoderUnits::invalid;
  }
}

std::int32_t Motor::setGearing(const AbstractMotor::gearset igearset) {
  switch (igearset) {
  case AbstractMotor::gearset::blue:
    return set_gearing(pros::E_MOTOR_GEARSET_06);
  case AbstractMotor::gearset::green:
    return set_gearing(pros::E_MOTOR_GEARSET_18);
  case AbstractMotor::gearset::red:
    return set_gearing(pros::E_MOTOR_GEARSET_36);
  case AbstractMotor::gearset::invalid:
    return set_gearing(pros::E_MOTOR_GEARSET_INVALID);
  }
}

AbstractMotor::gearset Motor::getGearing() const {
  switch (get_gearing()) {
  case pros::E_MOTOR_GEARSET_36:
    return AbstractMotor::gearset::red;
  case pros::E_MOTOR_GEARSET_18:
    return AbstractMotor::gearset::green;
  case pros::E_MOTOR_GEARSET_06:
    return AbstractMotor::gearset::blue;
  case pros::E_MOTOR_GEARSET_INVALID:
    return AbstractMotor::gearset::invalid;
  }
}

std::int32_t Motor::setReversed(const bool ireverse) const {
  return set_reversed(ireverse);
}

std::int32_t Motor::setVoltageLimit(const std::int32_t ilimit) const {
  return set_voltage_limit(ilimit);
}

std::int32_t
Motor::setPosPID(const double ikF, const double ikP, const double ikI, const double ikD) const {
  return set_pos_pid(convert_pid(ikF, ikP, ikI, ikD));
}

std::int32_t Motor::setPosPIDFull(const double ikF,
                                  const double ikP,
                                  const double ikI,
                                  const double ikD,
                                  const double ifilter,
                                  const double ilimit,
                                  const double ithreshold,
                                  const double iloopSpeed) const {
  return set_pos_pid_full(
    convert_pid_full(ikF, ikP, ikI, ikD, ifilter, ilimit, ithreshold, iloopSpeed));
}

std::int32_t
Motor::setVelPID(const double ikF, const double ikP, const double ikI, const double ikD) const {
  return set_vel_pid(convert_pid(ikF, ikP, ikI, ikD));
}

std::int32_t Motor::setVelPIDFull(const double ikF,
                                  const double ikP,
                                  const double ikI,
                                  const double ikD,
                                  const double ifilter,
                                  const double ilimit,
                                  const double ithreshold,
                                  const double iloopSpeed) const {
  return set_vel_pid_full(
    convert_pid_full(ikF, ikP, ikI, ikD, ifilter, ilimit, ithreshold, iloopSpeed));
}

std::shared_ptr<ContinuousRotarySensor> Motor::getEncoder() const {
  return std::make_shared<IntegratedEncoder>(*this);
}

void Motor::controllerSet(const double ivalue) {
  move_velocity(ivalue * toUnderlyingType(gearset));
}

inline namespace literals {
okapi::Motor operator"" _mtr(const unsigned long long iport) {
  return okapi::Motor(static_cast<uint8_t>(iport), false, AbstractMotor::gearset::red);
}

okapi::Motor operator"" _rmtr(const unsigned long long iport) {
  return okapi::Motor(static_cast<uint8_t>(iport), true, AbstractMotor::gearset::red);
}
} // namespace literals
} // namespace okapi
