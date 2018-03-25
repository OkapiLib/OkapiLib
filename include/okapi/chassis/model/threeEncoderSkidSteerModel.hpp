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
class ThreeEncoderSkidSteerModelParams : public SkidSteerModelParams {
  public:
  ThreeEncoderSkidSteerModelParams(const AbstractMotor &ileftSideMotor,
                                   const AbstractMotor &irightSideMotor,
                                   const RotarySensor &ileftEnc, const RotarySensor &imiddleEnc,
                                   const RotarySensor &irightEnc);

  virtual ~ThreeEncoderSkidSteerModelParams();

  /**
   * Constructs a new SkidSteerModel.
   *
   * @return const reference to the ChassisModel
   */
  virtual std::shared_ptr<const ChassisModel> make() const override;

  const RotarySensor &middleSensor;
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
  ThreeEncoderSkidSteerModel(const AbstractMotor &ileftSideMotor,
                             const AbstractMotor &irightSideMotor, const RotarySensor &ileftEnc,
                             const RotarySensor &imiddleEnc, const RotarySensor &irightEnc);

  ThreeEncoderSkidSteerModel(const ThreeEncoderSkidSteerModelParams &iparams);

  virtual ~ThreeEncoderSkidSteerModel();

  /**
   * Read the sensors.
   *
   * @return sensor readings in the format {left, right, middle}
   */
  virtual std::valarray<int> getSensorVals() const override;

  protected:
  const RotarySensor &middleSensor;
};
} // namespace okapi

#endif
