/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/chassis/controller/chassisScales.hpp"

namespace okapi {
ChassisScales::ChassisScales(const std::initializer_list<QLength> &iwheelbase) {
  validateInput(iwheelbase.size());

  std::vector<QLength> vec(iwheelbase);
  wheelDiameter = vec.at(0);
  wheelbaseWidth = vec.at(1);

  straight = static_cast<double>(360 / (wheelDiameter.convert(meter) * 1_pi));
  turn = wheelbaseWidth.convert(meter) / wheelDiameter.convert(meter);
}

void ChassisScales::validateInput(std::size_t inputSize) {
  if (inputSize < 2) {
    throw std::invalid_argument("At least two measurements must be given to ChassisScales.");
  }
}
} // namespace okapi
