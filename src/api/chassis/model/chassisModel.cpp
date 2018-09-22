/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/chassis/model/chassisModel.hpp"

namespace okapi {
ChassisModel::ChassisModel(const double imaxVelocity, const double imaxVoltage)
  : maxVelocity(imaxVelocity), maxVoltage(imaxVoltage) {
}

void ChassisModel::setMaxVelocity(const double imaxVelocity) {
  maxVelocity = imaxVelocity;
}

void ChassisModel::setMaxVoltage(const double imaxVoltage) {
  maxVoltage = imaxVoltage;
}
} // namespace okapi
