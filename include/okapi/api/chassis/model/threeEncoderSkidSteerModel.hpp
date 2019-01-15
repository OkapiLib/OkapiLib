/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#pragma once

#include "okapi/api/chassis/model/skidSteerModel.hpp"

namespace okapi {
class ThreeEncoderSkidSteerModel : public SkidSteerModel {
  public:
  /**
   * Model for a skid steer drive (wheels parallel with robot's direction of motion). When all
   * motors are powered +100%, the robot should move forward in a straight line.
   *
   * @param ileftSideMotor left side motor
   * @param irightSideMotor right side motor
   * @param ileftEnc left side encoder
   * @param imiddleEnc middle encoder (mounted perpendicular to the left and right side encoders)
   * @param irightEnc right side encoder
   */
  ThreeEncoderSkidSteerModel(const std::shared_ptr<AbstractMotor> &ileftSideMotor,
                             const std::shared_ptr<AbstractMotor> &irightSideMotor,
                             const std::shared_ptr<ContinuousRotarySensor> &ileftEnc,
                             const std::shared_ptr<ContinuousRotarySensor> &imiddleEnc,
                             const std::shared_ptr<ContinuousRotarySensor> &irightEnc,
                             double imaxVelocity,
                             double imaxVoltage = 12000);

  /**
   * Read the sensors.
   *
   * @return sensor readings in the format {left, right, middle}
   */
  std::valarray<std::int32_t> getSensorVals() const override;

  /**
   * Reset the sensors to their zero point.
   */
  void resetSensors() const override;

  protected:
  std::shared_ptr<ContinuousRotarySensor> middleSensor;
};
} // namespace okapi
