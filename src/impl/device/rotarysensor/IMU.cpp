/*
 * @author Alex Riensche, UNL
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/impl/device/rotarysensor/IMU.hpp"
#include "okapi/api/util/mathUtil.hpp"

namespace okapi {
IMU::IMU(std::uint8_t iport, IMUAxes iaxis):port(iport),axis(iaxis) {
}

IMU::~IMU() = default;

double IMU::get() const {
  pros::c::euler_s_t eu = pros::c::imu_get_euler(port);
  static double angle = 0;
  switch(axis) {
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
  angle += offset;
  if(angle > 180 || angle < -180) {
    angle += -360 * (offset/std::abs(offset));
  }
  return angle;
}

double IMU::getRemapped(const double iupperBound, const double ilowerBound) const {
  double value = get();

  return remapRange(value, -180, 180, ilowerBound, iupperBound);
}

double IMU::getAcc() {
  pros::c::imu_accel_s_t accel = pros::c::imu_get_accel(port);
  switch(axis) {
    case IMUAxes::x:
    return accel.x;
    break;
    case IMUAxes::y:
    return accel.y;
    break;
    case IMUAxes::z:
    return accel.z;
    break;
  }
  return -1;
}

std::int32_t IMU::reset() {
  const auto value = get() - offset;
  offset = -1 * value;

  return 1;
}

std::int32_t IMU::calibrate() {
  std::int32_t result = pros::c::imu_reset(port);
  pros::delay(2100);
  return result;
}

double IMU::controllerGet() {
  return get();
}

} // namespace okapi
