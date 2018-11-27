/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/chassis/controller/chassisScales.hpp"

namespace okapi {
ChassisScales::ChassisScales(const std::initializer_list<QLength> &iwheelbase,
                             const std::int32_t itpr,
                             const std::shared_ptr<Logger> &ilogger)
  : tpr(itpr) {
  validateInput(iwheelbase.size(), ilogger);

  std::vector<QLength> vec(iwheelbase);
  wheelDiameter = vec.at(0);
  wheelbaseWidth = vec.at(1);

  if (vec.size() >= 3) {
    middleWheelDiameter = vec.at(2);
  } else {
    middleWheelDiameter = wheelDiameter;
  }

  straight = static_cast<double>(tpr / (wheelDiameter.convert(meter) * 1_pi));
  turn = wheelbaseWidth.convert(meter) / wheelDiameter.convert(meter);
  middle = static_cast<double>(tpr / (middleWheelDiameter.convert(meter) * 1_pi));
}

ChassisScales::ChassisScales(const std::initializer_list<double> &iscales,
                             const std::int32_t itpr,
                             const std::shared_ptr<Logger> &ilogger)
  : tpr(itpr) {
  validateInput(iscales.size(), ilogger);

  std::vector<double> vec(iscales);
  straight = vec.at(0);
  turn = vec.at(1);

  if (vec.size() >= 3) {
    middle = vec.at(2);
  } else {
    middle = straight;
  }

  wheelDiameter = (tpr / (straight * 1_pi)) * meter;
  wheelbaseWidth = turn * wheelDiameter;
  middleWheelDiameter = (tpr / (middle * 1_pi)) * meter;
}

void ChassisScales::validateInput(const std::size_t inputSize,
                                  const std::shared_ptr<Logger> &logger) {
  if (inputSize < 2) {
    logger->error("At least two measurements must be given to ChassisScales. Got " +
                  std::to_string(inputSize) + "measurements.");
    throw std::invalid_argument("At least two measurements must be given to ChassisScales.");
  }
}
} // namespace okapi
