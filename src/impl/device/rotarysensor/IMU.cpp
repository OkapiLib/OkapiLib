/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/impl/device/rotarysensor/IMU.hpp"
#include "okapi/api/odometry/odomMath.hpp"

namespace okapi {
IMU::IMU(const std::uint8_t iport, const IMUAxes iaxis) : port(iport), axis(iaxis) {
}

double IMU::get() const {
  const double angle = readAngle();

  if (angle == PROS_ERR) {
    return PROS_ERR;
  }

  // Account for the offset after checking for PROS_ERR
  return OdomMath::constrainAngle180((angle - offset) * degree).convert(degree);
}

double IMU::getRemapped(const double iupperBound, const double ilowerBound) const {
  const double value = get();

  if (value == PROS_ERR) {
    return PROS_ERR;
  }

  return remapRange(get(), -180, 180, ilowerBound, iupperBound);
}

double IMU::getAcceleration() const {
  const pros::c::imu_accel_s_t accel = pros::c::imu_get_accel(port);

  switch (axis) {
  case IMUAxes::x:
    return accel.x;
  case IMUAxes::y:
    return accel.y;
  case IMUAxes::z:
    return accel.z;
  }

  // This should not run
  return PROS_ERR;
}

std::int32_t IMU::reset() {
  offset = readAngle();
  if (offset == PROS_ERR) {
    return PROS_ERR;
  } else {
    return 1;
  }
}

std::int32_t IMU::calibrate() {
  const std::int32_t result = pros::c::imu_reset(port);

  // Don't reset the offset or wait for calibration if the reset failed
  if (result == PROS_ERR) {
    return PROS_ERR;
  }

  offset = 0;

  // This operation should take approximately two seconds, so wait two seconds and then wait for it
  // to finish. We bound the maximum delay time to ensure that this function does not hang
  // indefinitely.
  const uint32_t maxDelayTime = 5000;
  const uint32_t before = pros::millis();
  pros::delay(2000);
  while (isCalibrating() && pros::millis() - before < maxDelayTime) {
    pros::delay(10);
  }

  if (pros::millis() - before < maxDelayTime) {
    // We did not timeout
    return 1;
  } else {
    // We timed out
    errno = EAGAIN;
    return PROS_ERR;
  }
}

double IMU::controllerGet() {
  return get();
}

double IMU::readAngle() const {
  const pros::c::euler_s_t eu = pros::c::imu_get_euler(port);
  double angle = 0;

  switch (axis) {
  case IMUAxes::x:
    angle = eu.roll;
    break;
  case IMUAxes::y:
    angle = eu.pitch;
    break;
  case IMUAxes::z:
    angle = eu.yaw;
    break;
  }

  return angle;
}

bool IMU::isCalibrating() const {
  return pros::c::imu_get_status(port) & pros::c::E_IMU_STATUS_CALIBRATING;
}
} // namespace okapi
