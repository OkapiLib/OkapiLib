/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/chassis/controller/chassisScales.hpp"

namespace okapi {
ChassisScales::ChassisScales(const std::initializer_list<QLength> &iwheelbase)
  : logger(Logger::instance()) {
  validateInput(iwheelbase.size());

  std::vector<QLength> vec(iwheelbase);
  wheelDiameter = vec.at(0);
  wheelbaseWidth = vec.at(1);
  straight = static_cast<double>(360 / (wheelDiameter.convert(meter) * 1_pi));
  turn = wheelbaseWidth.convert(meter) / wheelDiameter.convert(meter);
}

ChassisScales::ChassisScales(const std::initializer_list<double> &iscales)
  : logger(Logger::instance()) {
  validateInput(iscales.size());

  std::vector<double> vec(iscales);
  straight = vec.at(0);
  turn = vec.at(1);
  wheelDiameter = (360 / (straight * 1_pi)) * meter;
  wheelbaseWidth = turn * wheelDiameter;

  if (vec.size() >= 3) {
    middle = vec.at(2);
  }
}

void ChassisScales::validateInput(const std::size_t inputSize) {
  if (inputSize < 2) {
    logger->error("At least two measurements must be given to ChassisScales. Got " +
                  std::to_string(inputSize) + "measurements.");
    throw std::invalid_argument("At least two measurements must be given to ChassisScales.");
  }
}
} // namespace okapi
