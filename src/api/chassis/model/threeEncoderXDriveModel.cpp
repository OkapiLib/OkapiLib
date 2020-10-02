/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/chassis/model/threeEncoderXDriveModel.hpp"

namespace okapi {
ThreeEncoderXDriveModel::ThreeEncoderXDriveModel(std::shared_ptr<AbstractMotor> itopLeftMotor,
                                                 std::shared_ptr<AbstractMotor> itopRightMotor,
                                                 std::shared_ptr<AbstractMotor> ibottomRightMotor,
                                                 std::shared_ptr<AbstractMotor> ibottomLeftMotor,
                                                 std::shared_ptr<ContinuousRotarySensor> ileftEnc,
                                                 std::shared_ptr<ContinuousRotarySensor> irightEnc,
                                                 std::shared_ptr<ContinuousRotarySensor> imiddleEnc,
                                                 const double imaxVelocity,
                                                 const double imaxVoltage)
  : XDriveModel(std::move(itopLeftMotor),
                std::move(itopRightMotor),
                std::move(ibottomRightMotor),
                std::move(ibottomLeftMotor),
                std::move(ileftEnc),
                std::move(irightEnc),
                imaxVelocity,
                imaxVoltage),
    middleSensor(std::move(imiddleEnc)) {
}

std::valarray<std::int32_t> ThreeEncoderXDriveModel::getSensorVals() const {
  // Return the middle sensor last so this is compatible with XDriveModel::getSensorVals()
  return std::valarray<std::int32_t>{static_cast<std::int32_t>(leftSensor->get()),
                                     static_cast<std::int32_t>(rightSensor->get()),
                                     static_cast<std::int32_t>(middleSensor->get())};
}

void ThreeEncoderXDriveModel::resetSensors() {
  XDriveModel::resetSensors();
  middleSensor->reset();
}
} // namespace okapi
