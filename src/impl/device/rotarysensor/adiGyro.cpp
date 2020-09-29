/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/impl/device/rotarysensor/adiGyro.hpp"
#include "okapi/api/util/mathUtil.hpp"

namespace okapi {
ADIGyro::ADIGyro(const std::uint8_t iport, const double imultiplier)
  : ADIGyro({INTERNAL_ADI_PORT, iport}, imultiplier) {
}

ADIGyro::ADIGyro(std::pair<std::uint8_t, std::uint8_t> iports, const double imultiplier)
  : gyro(pros::c::ext_adi_gyro_init(std::get<0>(iports), std::get<1>(iports), imultiplier)) {
}

double ADIGyro::get() const {
  return pros::c::ext_adi_gyro_get(gyro);
}

double ADIGyro::getRemapped(const double iupperBound, const double ilowerBound) const {
  const auto value = get();

  if (value == PROS_ERR) {
    return value;
  }

  return remapRange(value, -3600, 3600, ilowerBound, iupperBound);
}

std::int32_t ADIGyro::reset() {
  return pros::c::ext_adi_gyro_reset(gyro);
}

double ADIGyro::controllerGet() {
  return get();
}
} // namespace okapi
