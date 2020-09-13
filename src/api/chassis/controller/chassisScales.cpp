/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/chassis/controller/chassisScales.hpp"

namespace okapi {
ChassisScales::ChassisScales(const std::initializer_list<QLength> &idimensions,
                             const double itpr,
                             const std::shared_ptr<Logger> &logger)
  : tpr(itpr) {
  validateInputSize(idimensions.size(), logger);

  std::vector<QLength> vec(idimensions);
  wheelDiameter = vec.at(0);
  wheelTrack = vec.at(1);

  if (wheelDiameter > wheelTrack) {
    LOG_WARN("ChassisScales: Wheel diameter (" + std::to_string(wheelDiameter.convert(meter)) +
             " meters) is greater than wheel track (" + std::to_string(wheelTrack.convert(meter)) +
             " meters). This is probably an error.");
  }

  if (vec.size() >= 3) {
    middleWheelDistance = vec.at(2);
  } else {
    middleWheelDistance = 0_m;
  }

  if (vec.size() >= 4) {
    middleWheelDiameter = vec.at(3);
  } else {
    middleWheelDiameter = wheelDiameter;
  }

  straight = static_cast<double>(tpr / (wheelDiameter.convert(meter) * 1_pi));
  turn = wheelTrack.convert(meter) / wheelDiameter.convert(meter) * itpr / 360.0;
  middle = static_cast<double>(tpr / (middleWheelDiameter.convert(meter) * 1_pi));
}

ChassisScales::ChassisScales(const std::initializer_list<double> &iscales,
                             const double itpr,
                             const std::shared_ptr<Logger> &logger)
  : tpr(itpr) {
  validateInputSize(iscales.size(), logger);

  if (iscales.size() == 3) {
    std::string msg("Middle wheel distance and scale must both be supplied, not just one.");
    LOG_ERROR(msg);
    throw std::invalid_argument(msg);
  }

  std::vector<double> vec(iscales);
  straight = vec.at(0);
  turn = vec.at(1);

  if (vec.size() >= 4) {
    middle = vec.at(3);
  } else {
    middle = straight;
  }

  wheelDiameter = static_cast<double>(tpr / (straight * 1_pi)) * meter;
  wheelTrack = turn * wheelDiameter;
  middleWheelDiameter = static_cast<double>(tpr / (middle * 1_pi)) * meter;

  if (vec.size() >= 4) {
    middleWheelDistance = vec.at(2) * meter;
  } else {
    middleWheelDistance = 0_m;
  }
}

void ChassisScales::validateInputSize(size_t inputSize, const std::shared_ptr<Logger> &logger) {
  if (inputSize < 2) {
    std::string msg(
      "ChassisScales: At least two measurements must be given to ChassisScales. Got " +
      std::to_string(inputSize) + "measurements.");
    LOG_ERROR(msg);
    throw std::invalid_argument(msg);
  }
}
} // namespace okapi
