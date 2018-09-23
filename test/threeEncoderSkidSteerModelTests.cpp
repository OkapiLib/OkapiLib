/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/chassis/model/threeEncoderSkidSteerModel.hpp"
#include "okapi/api/chassis/model/xDriveModel.hpp"
#include "test/tests/api/implMocks.hpp"
#include <gtest/gtest.h>

using namespace okapi;

class ThreeEncoderSkidSteerModelTest : public ::testing::Test {
  protected:
  void SetUp() override {
    leftMotor = std::make_shared<MockMotor>();
    rightMotor = std::make_shared<MockMotor>();
    leftSensor = std::make_shared<MockContinuousRotarySensor>();
    rightSensor = std::make_shared<MockContinuousRotarySensor>();
    middleSensor = std::make_shared<MockContinuousRotarySensor>();
    model = new ThreeEncoderSkidSteerModel(
      leftMotor, rightMotor, leftSensor, middleSensor, rightSensor, 100);
  }

  void TearDown() override {
    delete model;
  }

  std::shared_ptr<MockMotor> leftMotor;
  std::shared_ptr<MockMotor> rightMotor;
  std::shared_ptr<MockContinuousRotarySensor> leftSensor;
  std::shared_ptr<MockContinuousRotarySensor> rightSensor;
  std::shared_ptr<MockContinuousRotarySensor> middleSensor;
  ThreeEncoderSkidSteerModel *model;
};

TEST_F(ThreeEncoderSkidSteerModelTest, GetSensorValsIsCompatibleWithSkidSteerModel) {
  leftSensor->value = 1;
  rightSensor->value = 2;
  middleSensor->value = 3;

  auto skidSteerModelVals =
    SkidSteerModel(leftMotor, rightMotor, leftSensor, rightSensor, 100).getSensorVals();
  auto threeEncoderSkidSteerModelVals = model->getSensorVals();

  EXPECT_EQ(skidSteerModelVals[0], threeEncoderSkidSteerModelVals[0]);
  EXPECT_EQ(skidSteerModelVals[1], threeEncoderSkidSteerModelVals[1]);
}

TEST_F(ThreeEncoderSkidSteerModelTest, GetSensorValsIsCompatibleWithXDriveModel) {
  leftSensor->value = 1;
  rightSensor->value = 2;
  middleSensor->value = 3;

  // nonsense motors because it doesn't matter
  auto xDriveModelVals =
    XDriveModel(leftMotor, rightMotor, leftMotor, rightMotor, leftSensor, rightSensor, 100)
      .getSensorVals();
  auto threeEncoderSkidSteerModelVals = model->getSensorVals();

  EXPECT_EQ(xDriveModelVals[0], threeEncoderSkidSteerModelVals[0]);
  EXPECT_EQ(xDriveModelVals[1], threeEncoderSkidSteerModelVals[1]);
}
