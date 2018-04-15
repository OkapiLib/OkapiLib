/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_THREEENCODERSKIDSTEERMODEL_HPP_
#define _OKAPI_THREEENCODERSKIDSTEERMODEL_HPP_

#include "okapi/chassis/model/skidSteerModel.hpp"

namespace okapi {
class ThreeEncoderSkidSteerModelArgs : public SkidSteerModelArgs {
  public:
  ThreeEncoderSkidSteerModelArgs(std::shared_ptr<AbstractMotor> ileftSideMotor,
                                 std::shared_ptr<AbstractMotor> irightSideMotor,
                                 std::shared_ptr<RotarySensor> ileftEnc,
                                 std::shared_ptr<RotarySensor> imiddleEnc,
                                 std::shared_ptr<RotarySensor> irightEnc,
                                 const double imaxOutput = 127);

  std::shared_ptr<RotarySensor> middleSensor;
};

class ThreeEncoderSkidSteerModel : public SkidSteerModel {
  public:
  /**
   * Model for a skid steer drive (wheels parallel with robot's direction of motion). When all
   * motors are powered +127, the robot should move forward in a straight line.
   *
   * @param ileftSideMotor left side motor
   * @param irightSideMotor right side motor
   * @param ileftEnc left side encoder
   * @param imiddleEnc middle encoder (mounted perpendicular to the left and right side encoders)
   * @param irightEnc right side encoder
   */
  ThreeEncoderSkidSteerModel(std::shared_ptr<AbstractMotor> ileftSideMotor,
                             std::shared_ptr<AbstractMotor> irightSideMotor,
                             std::shared_ptr<RotarySensor> ileftEnc,
                             std::shared_ptr<RotarySensor> imiddleEnc,
                             std::shared_ptr<RotarySensor> irightEnc,
                             const double imaxOutput = 127);

  ThreeEncoderSkidSteerModel(const ThreeEncoderSkidSteerModelArgs &iparams);

  /**
   * Read the sensors.
   *
   * @return sensor readings in the format {left, right, middle}
   */
  virtual std::valarray<int> getSensorVals() const override;

  protected:
  std::shared_ptr<RotarySensor> middleSensor;
};
} // namespace okapi

#endif
