/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_CHASSISMODELFACTORY_HPP_
#define _OKAPI_CHASSISMODELFACTORY_HPP_

#include "okapi/api/chassis/model/skidSteerModel.hpp"
#include "okapi/api/chassis/model/threeEncoderSkidSteerModel.hpp"
#include "okapi/api/chassis/model/xDriveModel.hpp"
#include "okapi/impl/device/motor/motor.hpp"
#include "okapi/impl/device/motor/motorGroup.hpp"
#include "okapi/impl/device/rotarysensor/adiEncoder.hpp"

namespace okapi {
class ChassisModelFactory {
  public:
  /**
   * Model for a skid steer drive (wheels parallel with robot's direction of motion). When all
   * motors are powered +127, the robot should move forward in a straight line.
   *
   * This constructor infers the two sensors from the left and right motors (using the integrated
   * encoders).
   *
   * @param ileftSideMotor left side motor
   * @param irightSideMotor right side motor
   */
  static SkidSteerModel create(Motor ileftSideMotor,
                               Motor irightSideMotor,
                               double imaxVelocity = 127,
                               double imaxVoltage = 12000);

  /**
   * Model for a skid steer drive (wheels parallel with robot's direction of motion). When all
   * motors are powered +127, the robot should move forward in a straight line.
   *
   * This constructor infers the two sensors from the left and right motors (using the integrated
   * encoders).
   *
   * @param ileftSideMotor left side motor
   * @param irightSideMotor right side motor
   */
  static SkidSteerModel create(MotorGroup ileftSideMotor,
                               MotorGroup irightSideMotor,
                               double imaxVelocity = 127,
                               double imaxVoltage = 12000);

  /**
   * Model for a skid steer drive (wheels parallel with robot's direction of motion). When all
   * motors are powered +127, the robot should move forward in a straight line.
   *
   * This constructor infers the two sensors from the left and right motors (using the integrated
   * encoders).
   *
   * @param ileftSideMotor left side motor
   * @param irightSideMotor right side motor
   */
  static SkidSteerModel create(MotorGroup ileftSideMotor,
                               MotorGroup irightSideMotor,
                               ADIEncoder ileftEnc,
                               ADIEncoder irightEnc,
                               double imaxVelocity = 127,
                               double imaxVoltage = 12000);

  /**
   * Model for an x drive (wheels at 45 deg from a skid steer drive). When all motors are powered
   * +127, the robot should move forward in a straight line.
   *
   * This constructor infers the two sensors from the top left and top right motors (using the
   * integrated encoders).
   *
   * @param itopLeftMotor top left motor
   * @param itopRightMotor top right motor
   * @param ibottomRightMotor bottom right motor
   * @param ibottomLeftMotor bottom left motor
   */
  static XDriveModel create(Motor itopLeftMotor,
                            Motor itopRightMotor,
                            Motor ibottomRightMotor,
                            Motor ibottomLeftMotor,
                            double imaxVelocity = 127,
                            double imaxVoltage = 12000);

  /**
   * Model for an x drive (wheels at 45 deg from a skid steer drive). When all motors are powered
   * +127, the robot should move forward in a straight line.
   *
   * This constructor infers the two sensors from the top left and top right motors (using the
   * integrated encoders).
   *
   * @param itopLeftMotor top left motor
   * @param itopRightMotor top right motor
   * @param ibottomRightMotor bottom right motor
   * @param ibottomLeftMotor bottom left motor
   */
  static XDriveModel create(Motor itopLeftMotor,
                            Motor itopRightMotor,
                            Motor ibottomRightMotor,
                            Motor ibottomLeftMotor,
                            ADIEncoder ileftEnc,
                            ADIEncoder irightEnc,
                            double imaxVelocity = 127,
                            double imaxVoltage = 12000);

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
  static ThreeEncoderSkidSteerModel create(Motor ileftSideMotor,
                                           Motor irightSideMotor,
                                           ADIEncoder ileftEnc,
                                           ADIEncoder imiddleEnc,
                                           ADIEncoder irightEnc,
                                           double imaxVelocity = 127,
                                           double imaxVoltage = 12000);

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
  static ThreeEncoderSkidSteerModel create(MotorGroup ileftSideMotor,
                                           MotorGroup irightSideMotor,
                                           ADIEncoder ileftEnc,
                                           ADIEncoder imiddleEnc,
                                           ADIEncoder irightEnc,
                                           double imaxVelocity = 127,
                                           double imaxVoltage = 12000);
};
} // namespace okapi

#endif