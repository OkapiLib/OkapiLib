/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/chassis/model/threeEncoderSkidSteerModel.hpp"

namespace okapi {
ThreeEncoderSkidSteerModelArgs::ThreeEncoderSkidSteerModelArgs(
  std::shared_ptr<AbstractMotor> ileftSideMotor,
  std::shared_ptr<AbstractMotor> irightSideMotor,
  std::shared_ptr<ContinuousRotarySensor> ileftEnc,
  std::shared_ptr<ContinuousRotarySensor> imiddleEnc,
  std::shared_ptr<ContinuousRotarySensor> irightEnc,
  const double imaxOutput)
  : SkidSteerModelArgs(ileftSideMotor, irightSideMotor, ileftEnc, irightEnc, imaxOutput),
    middleSensor(imiddleEnc) {
}

ThreeEncoderSkidSteerModel::ThreeEncoderSkidSteerModel(
  std::shared_ptr<AbstractMotor> ileftSideMotor,
  std::shared_ptr<AbstractMotor> irightSideMotor,
  std::shared_ptr<ContinuousRotarySensor> ileftEnc,
  std::shared_ptr<ContinuousRotarySensor> imiddleEnc,
  std::shared_ptr<ContinuousRotarySensor> irightEnc,
  const double imaxOutput)
  : SkidSteerModel(ileftSideMotor, irightSideMotor, ileftEnc, irightEnc, imaxOutput),
    middleSensor(imiddleEnc) {
}

ThreeEncoderSkidSteerModel::ThreeEncoderSkidSteerModel(
  const ThreeEncoderSkidSteerModelArgs &iparams)
  : SkidSteerModel(iparams.leftSideMotor,
                   iparams.rightSideMotor,
                   iparams.leftSensor,
                   iparams.rightSensor),
    middleSensor(iparams.middleSensor) {
}

std::valarray<std::int32_t> ThreeEncoderSkidSteerModel::getSensorVals() const {
  // Return the middle sensor last so this is compatible with SkidSteerModel::getSensorVals()
  return std::valarray<std::int32_t>{leftSensor->get(), rightSensor->get(), middleSensor->get()};
}
} // namespace okapi
