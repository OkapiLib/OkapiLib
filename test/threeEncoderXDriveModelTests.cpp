/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/chassis/model/threeEncoderXDriveModel.hpp"
#include "okapi/api/chassis/model/xDriveModel.hpp"
#include "test/tests/api/implMocks.hpp"
#include <gtest/gtest.h>

using namespace okapi;

class ThreeEncoderXDriveModelTest : public ::testing::Test {
  protected:
  void SetUp() override {
    topLeftMotor = std::make_shared<MockMotor>();
    topRightMotor = std::make_shared<MockMotor>();
    bottomRightMotor = std::make_shared<MockMotor>();
    bottomLeftMotor = std::make_shared<MockMotor>();
    leftSensor = std::make_shared<MockContinuousRotarySensor>();
    rightSensor = std::make_shared<MockContinuousRotarySensor>();
    middleSensor = std::make_shared<MockContinuousRotarySensor>();
    model = new ThreeEncoderXDriveModel(topLeftMotor,
                                        topRightMotor,
                                        bottomRightMotor,
                                        bottomLeftMotor,
                                        leftSensor,
                                        rightSensor,
                                        middleSensor,
                                        100,
                                        v5MotorMaxVoltage);
  }

  void TearDown() override {
    delete model;
  }

  std::shared_ptr<MockMotor> topLeftMotor;
  std::shared_ptr<MockMotor> topRightMotor;
  std::shared_ptr<MockMotor> bottomRightMotor;
  std::shared_ptr<MockMotor> bottomLeftMotor;
  std::shared_ptr<MockContinuousRotarySensor> leftSensor;
  std::shared_ptr<MockContinuousRotarySensor> rightSensor;
  std::shared_ptr<MockContinuousRotarySensor> middleSensor;
  ThreeEncoderXDriveModel *model;
};

TEST_F(ThreeEncoderXDriveModelTest, GetSensorValsIsCompatibleWithXDriveModel) {
  leftSensor->value = 1;
  rightSensor->value = 2;
  middleSensor->value = 3;

  auto xDriveModelVals = XDriveModel(topLeftMotor,
                                     topRightMotor,
                                     bottomRightMotor,
                                     bottomLeftMotor,
                                     leftSensor,
                                     rightSensor,
                                     100,
                                     v5MotorMaxVoltage)
                           .getSensorVals();
  auto threeEncoderXDriveModelVals = model->getSensorVals();

  EXPECT_EQ(xDriveModelVals[0], threeEncoderXDriveModelVals[0]);
  EXPECT_EQ(xDriveModelVals[1], threeEncoderXDriveModelVals[1]);
}

TEST_F(ThreeEncoderXDriveModelTest, Reset) {
  leftSensor->value = 1;
  rightSensor->value = 1;
  middleSensor->value = 1;

  model->resetSensors();

  EXPECT_EQ(leftSensor->get(), 0);
  EXPECT_EQ(rightSensor->get(), 0);
  EXPECT_EQ(middleSensor->get(), 0);
}
