/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/chassis/model/threeEncoderSkidSteerModel.hpp"

namespace okapi {
ThreeEncoderSkidSteerModel::ThreeEncoderSkidSteerModel(
  const std::shared_ptr<AbstractMotor> &ileftSideMotor,
  const std::shared_ptr<AbstractMotor> &irightSideMotor,
  const std::shared_ptr<ContinuousRotarySensor> &ileftEnc,
  const std::shared_ptr<ContinuousRotarySensor> &imiddleEnc,
  const std::shared_ptr<ContinuousRotarySensor> &irightEnc,
  const double imaxVelocity,
  const double imaxVoltage)
  : SkidSteerModel(ileftSideMotor, irightSideMotor, ileftEnc, irightEnc, imaxVelocity, imaxVoltage),
    middleSensor(imiddleEnc) {
}

std::valarray<std::int32_t> ThreeEncoderSkidSteerModel::getSensorVals() const {
  // Return the middle sensor last so this is compatible with SkidSteerModel::getSensorVals()
  return std::valarray<std::int32_t>{static_cast<std::int32_t>(leftSensor->get()),
                                     static_cast<std::int32_t>(rightSensor->get()),
                                     static_cast<std::int32_t>(middleSensor->get())};
}

void ThreeEncoderSkidSteerModel::resetSensors() const {
  SkidSteerModel::resetSensors();
  middleSensor->reset();
}
} // namespace okapi
