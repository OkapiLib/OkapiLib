/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/chassis/controller/chassisScales.hpp"

namespace okapi {

ChassisScales::ChassisScales(const std::initializer_list<double> &iscales) {
  validateInput(iscales.size());

  std::vector<double> vec(iscales);
  straight = vec.at(0);
  turn = vec.at(1);

  if (iscales.size() >= 3) {
    middle = vec.at(2);
  }
}

ChassisScales::ChassisScales(const std::initializer_list<QLength> &iwheelbase) {
  validateInput(iwheelbase.size());

  std::vector<QLength> vec(iwheelbase);
  straight = static_cast<double>(360 / (vec.at(0).convert(meter) * 1_pi));
  turn = vec.at(1).convert(meter) / vec.at(0).convert(meter);
  // TODO: RGB: What would the middle scale be?
}

void ChassisScales::validateInput(std::size_t inputSize) {
  if (inputSize < 2) {
    throw std::invalid_argument("At least two measurements must be given to ChassisScales.");
  }
}
} // namespace okapi
