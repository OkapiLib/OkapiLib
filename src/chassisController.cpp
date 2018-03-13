/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#include "okapi/chassis/chassisController.hpp"
#include "okapi/util/timer.hpp"
#include <cmath>

namespace okapi {
  ChassisController::~ChassisController() = default;

  void ChassisController::driveForward(const int power) {
    model->driveForward(power);
  }

  void ChassisController::driveVector(const int distPower, const int anglePower) {
    model->driveVector(distPower, anglePower);
  }

  void ChassisController::turnClockwise(const int power) {
    model->turnClockwise(power);
  }

  void ChassisController::stop() {
    model->stop();
  }

  void ChassisController::tank(const int leftVal, const int rightVal, const int threshold) {
    model->tank(leftVal, rightVal, threshold);
  }

  void ChassisController::arcade(int verticalVal, int horizontalVal, const int threshold) {
    model->arcade(verticalVal, horizontalVal, threshold);
  }

  void ChassisController::left(const int val) {
    model->left(val);
  }

  void ChassisController::right(const int val) {
    model->right(val);
  }

  std::valarray<int> ChassisController::getSensorVals() {
    return model->getSensorVals();
  }
}
