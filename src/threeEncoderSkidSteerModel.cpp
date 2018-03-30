/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#include "okapi/chassis/model/threeEncoderSkidSteerModel.hpp"

namespace okapi {
ThreeEncoderSkidSteerModelArgs::ThreeEncoderSkidSteerModelArgs(const AbstractMotor &ileftSideMotor,
                                                               const AbstractMotor &irightSideMotor,
                                                               const RotarySensor &ileftEnc,
                                                               const RotarySensor &imiddleEnc,
                                                               const RotarySensor &irightEnc)
  : SkidSteerModelArgs(ileftSideMotor, irightSideMotor, ileftEnc, irightEnc),
    middleSensor(imiddleEnc) {
}

ThreeEncoderSkidSteerModel::ThreeEncoderSkidSteerModel(const AbstractMotor &ileftSideMotor,
                                                       const AbstractMotor &irightSideMotor,
                                                       const RotarySensor &ileftEnc,
                                                       const RotarySensor &imiddleEnc,
                                                       const RotarySensor &irightEnc)
  : SkidSteerModel(ileftSideMotor, irightSideMotor, ileftEnc, irightEnc), middleSensor(imiddleEnc) {
}

ThreeEncoderSkidSteerModel::ThreeEncoderSkidSteerModel(
  const ThreeEncoderSkidSteerModelArgs &iparams)
  : SkidSteerModel(iparams.leftSideMotor, iparams.rightSideMotor, iparams.leftSensor,
                   iparams.rightSensor),
    middleSensor(iparams.middleSensor) {
}

std::valarray<int> ThreeEncoderSkidSteerModel::getSensorVals() const {
  // Return the middle sensor last so this is compatible with SkidSteerModel::getSensorVals()
  return std::valarray<int>{leftSensor.get(), rightSensor.get(), middleSensor.get()};
}
} // namespace okapi
