/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/chassis/controller/chassisControllerIntegrated.hpp"
#include "okapi/api/chassis/controller/chassisControllerPid.hpp"
#include "okapi/api/chassis/controller/chassisScales.hpp"
#include "okapi/api/chassis/model/skidSteerModel.hpp"
#include "test/crossPlatformTestRunner.hpp"
#include "test/tests/api/implMocks.hpp"
#include <gtest/gtest.h>

using namespace okapi;
using namespace snowhouse;

TEST(ChassisScalesTest, RawScales) {
  ChassisScales scales({0.5, 0.3});
  EXPECT_DOUBLE_EQ(scales.straight, 0.5);
  EXPECT_DOUBLE_EQ(scales.turn, 0.3);
}

TEST(ChassisScalesTest, ScalesFromWheelbase) {
  ChassisScales scales({4_in, 11.5_in});
  EXPECT_FLOAT_EQ(scales.straight, 1127.8696);
  EXPECT_FLOAT_EQ(scales.turn, 2.875);
}

class ChassisControllerIntegratedTest : public ::testing::Test {
  protected:
  void SetUp() override {
    scales = new ChassisScales({2, 2});
    leftMotor = new MockMotor();
    rightMotor = new MockMotor();

    leftController = new MockAsyncController();
    rightController = new MockAsyncController();

    model = new SkidSteerModel(std::unique_ptr<AbstractMotor>(leftMotor),
                               std::unique_ptr<AbstractMotor>(rightMotor));

    controller = new ChassisControllerIntegrated(
      createTimeUtil(), std::unique_ptr<ChassisModel>(model),
      std::unique_ptr<AsyncPosIntegratedController>(leftController),
      std::unique_ptr<AsyncPosIntegratedController>(rightController), AbstractMotor::gearset::red,
      *scales);
  }

  void TearDown() override {
    delete scales;
    delete controller;
  }

  ChassisScales *scales;
  ChassisController *controller;
  MockMotor *leftMotor;
  MockMotor *rightMotor;
  MockAsyncController *leftController;
  MockAsyncController *rightController;
  SkidSteerModel *model;
};

TEST_F(ChassisControllerIntegratedTest, MoveDistanceRawUnitsTest) {
  controller->moveDistance(100);

  EXPECT_DOUBLE_EQ(leftController->target, 100);
  EXPECT_DOUBLE_EQ(rightController->target, 100);

  EXPECT_TRUE(leftController->disabled);
  EXPECT_TRUE(rightController->disabled);

  assertMotorsHaveBeenStopped(leftMotor, rightMotor);
}

TEST_F(ChassisControllerIntegratedTest, MoveDistanceUnitsTest) {
  controller->moveDistance(1_m);

  EXPECT_DOUBLE_EQ(leftController->target, 2);
  EXPECT_DOUBLE_EQ(rightController->target, 2);

  EXPECT_TRUE(leftController->disabled);
  EXPECT_TRUE(rightController->disabled);

  assertMotorsHaveBeenStopped(leftMotor, rightMotor);
}

TEST_F(ChassisControllerIntegratedTest, MoveDistanceAsyncRawUnitsTest) {
  controller->moveDistanceAsync(100);

  EXPECT_DOUBLE_EQ(leftController->target, 100);
  EXPECT_DOUBLE_EQ(rightController->target, 100);

  EXPECT_FALSE(leftController->disabled);
  EXPECT_FALSE(rightController->disabled);

  controller->waitUntilSettled();

  EXPECT_TRUE(leftController->disabled);
  EXPECT_TRUE(rightController->disabled);

  assertMotorsHaveBeenStopped(leftMotor, rightMotor);
}

TEST_F(ChassisControllerIntegratedTest, MoveDistanceAsyncUnitsTest) {
  controller->moveDistanceAsync(1_m);

  EXPECT_DOUBLE_EQ(leftController->target, 2);
  EXPECT_DOUBLE_EQ(rightController->target, 2);

  EXPECT_FALSE(leftController->disabled);
  EXPECT_FALSE(rightController->disabled);

  controller->waitUntilSettled();

  EXPECT_DOUBLE_EQ(leftController->target, 2);
  EXPECT_DOUBLE_EQ(rightController->target, 2);

  EXPECT_TRUE(leftController->disabled);
  EXPECT_TRUE(rightController->disabled);

  assertMotorsHaveBeenStopped(leftMotor, rightMotor);
}

TEST_F(ChassisControllerIntegratedTest, TurnAngleRawUnitsTest) {
  controller->turnAngle(100);

  EXPECT_DOUBLE_EQ(leftController->target, 100);
  EXPECT_DOUBLE_EQ(rightController->target, -100);

  EXPECT_TRUE(leftController->disabled);
  EXPECT_TRUE(rightController->disabled);

  assertMotorsHaveBeenStopped(leftMotor, rightMotor);
}

TEST_F(ChassisControllerIntegratedTest, TurnAngleUnitsTest) {
  controller->turnAngle(45_deg);

  EXPECT_DOUBLE_EQ(leftController->target, 90);
  EXPECT_DOUBLE_EQ(rightController->target, -90);

  EXPECT_TRUE(leftController->disabled);
  EXPECT_TRUE(rightController->disabled);

  assertMotorsHaveBeenStopped(leftMotor, rightMotor);
}

TEST_F(ChassisControllerIntegratedTest, TurnAngleAsyncRawUnitsTest) {
  controller->turnAngleAsync(100);

  EXPECT_DOUBLE_EQ(leftController->target, 100);
  EXPECT_DOUBLE_EQ(rightController->target, -100);

  EXPECT_FALSE(leftController->disabled);
  EXPECT_FALSE(rightController->disabled);

  controller->waitUntilSettled();

  EXPECT_TRUE(leftController->disabled);
  EXPECT_TRUE(rightController->disabled);

  assertMotorsHaveBeenStopped(leftMotor, rightMotor);
}

TEST_F(ChassisControllerIntegratedTest, TurnAngleAsyncUnitsTest) {
  controller->turnAngleAsync(45_deg);

  EXPECT_DOUBLE_EQ(leftController->target, 90);
  EXPECT_DOUBLE_EQ(rightController->target, -90);

  EXPECT_FALSE(leftController->disabled);
  EXPECT_FALSE(rightController->disabled);

  controller->waitUntilSettled();

  EXPECT_TRUE(leftController->disabled);
  EXPECT_TRUE(rightController->disabled);

  assertMotorsHaveBeenStopped(leftMotor, rightMotor);
}

TEST_F(ChassisControllerIntegratedTest, MoveDistanceThenTurnAngleAsyncTest) {
  controller->moveDistanceAsync(100);

  EXPECT_DOUBLE_EQ(leftController->target, 100);
  EXPECT_DOUBLE_EQ(rightController->target, 100);

  EXPECT_FALSE(leftController->disabled);
  EXPECT_FALSE(rightController->disabled);

  controller->turnAngleAsync(200);

  EXPECT_DOUBLE_EQ(leftController->target, 200);
  EXPECT_DOUBLE_EQ(rightController->target, -200);

  EXPECT_FALSE(leftController->disabled);
  EXPECT_FALSE(rightController->disabled);

  controller->waitUntilSettled();

  EXPECT_TRUE(leftController->disabled);
  EXPECT_TRUE(rightController->disabled);

  assertMotorsHaveBeenStopped(leftMotor, rightMotor);
}

TEST_F(ChassisControllerIntegratedTest, TurnAngleThenMoveDistanceAsyncTest) {
  controller->turnAngleAsync(200);

  EXPECT_DOUBLE_EQ(leftController->target, 200);
  EXPECT_DOUBLE_EQ(rightController->target, -200);

  EXPECT_FALSE(leftController->disabled);
  EXPECT_FALSE(rightController->disabled);

  controller->moveDistanceAsync(100);

  EXPECT_DOUBLE_EQ(leftController->target, 100);
  EXPECT_DOUBLE_EQ(rightController->target, 100);

  EXPECT_FALSE(leftController->disabled);
  EXPECT_FALSE(rightController->disabled);

  controller->waitUntilSettled();

  EXPECT_TRUE(leftController->disabled);
  EXPECT_TRUE(rightController->disabled);

  assertMotorsHaveBeenStopped(leftMotor, rightMotor);
}

TEST_F(ChassisControllerIntegratedTest, MoveDistanceAndWaitTest) {
  controller->moveDistance(100);
  controller->waitUntilSettled();

  assertMotorsHaveBeenStopped(leftMotor, rightMotor);
}

TEST_F(ChassisControllerIntegratedTest, MoveDistanceGetPushedAndWaitTest) {
  controller->moveDistance(100);

  // First bump
  leftMotor->encoder->value = 500;
  rightMotor->encoder->value = 500;
  controller->waitUntilSettled();

  // Second bump
  leftMotor->encoder->value = 1000;
  rightMotor->encoder->value = 1000;
  controller->waitUntilSettled();

  assertMotorsHaveBeenStopped(leftMotor, rightMotor);
}

TEST_F(ChassisControllerIntegratedTest, TurnAngleAndWaitTest) {
  controller->turnAngle(100);
  controller->waitUntilSettled();

  assertMotorsHaveBeenStopped(leftMotor, rightMotor);
}

TEST_F(ChassisControllerIntegratedTest, TurnAngleGetPushedAndWaitTest) {
  controller->turnAngle(100);

  // First bump
  leftMotor->encoder->value = 500;
  rightMotor->encoder->value = 500;
  controller->waitUntilSettled();

  // Second bump
  leftMotor->encoder->value = 1000;
  rightMotor->encoder->value = 1000;
  controller->waitUntilSettled();

  assertMotorsHaveBeenStopped(leftMotor, rightMotor);
}

TEST_F(ChassisControllerIntegratedTest, MoveDistanceAndStopTest) {
  controller->moveDistanceAsync(100);
  controller->stop();

  assertMotorsHaveBeenStopped(leftMotor, rightMotor);
  EXPECT_TRUE(leftController->isDisabled());
  EXPECT_TRUE(rightController->isDisabled());
}

TEST_F(ChassisControllerIntegratedTest, TurnAngleAndStopTest) {
  controller->turnAngleAsync(100);
  controller->stop();

  assertMotorsHaveBeenStopped(leftMotor, rightMotor);
  EXPECT_TRUE(leftController->isDisabled());
  EXPECT_TRUE(rightController->isDisabled());
}

class ChassisControllerPIDTest : public ::testing::Test {
  protected:
  void SetUp() override {
    scales = new ChassisScales({2, 2});
    leftMotor = new MockMotor();
    rightMotor = new MockMotor();

    distanceController = new MockIterativeController();
    angleController = new MockIterativeController();

    model = new SkidSteerModel(std::unique_ptr<AbstractMotor>(leftMotor),
                               std::unique_ptr<AbstractMotor>(rightMotor));

    controller =
      new ChassisControllerPID(createTimeUtil(), std::unique_ptr<ChassisModel>(model),
                               std::unique_ptr<IterativePosPIDController>(distanceController),
                               std::unique_ptr<IterativePosPIDController>(angleController),
                               AbstractMotor::gearset::red, *scales);
  }

  void TearDown() override {
    delete scales;
    delete controller;
  }

  ChassisScales *scales;
  ChassisController *controller;
  MockMotor *leftMotor;
  MockMotor *rightMotor;
  MockIterativeController *distanceController;
  MockIterativeController *angleController;
  SkidSteerModel *model;
};

TEST_F(ChassisControllerPIDTest, MoveDistanceRawUnitsTest) {
  controller->moveDistance(100);

  EXPECT_DOUBLE_EQ(distanceController->target, 100);
  EXPECT_DOUBLE_EQ(angleController->target, 0);

  EXPECT_TRUE(distanceController->disabled);
  EXPECT_TRUE(angleController->disabled);

  assertMotorsHaveBeenStopped(leftMotor, rightMotor);
}

TEST_F(ChassisControllerPIDTest, MoveDistanceUnitsTest) {
  controller->moveDistance(1_m);

  EXPECT_DOUBLE_EQ(distanceController->target, 2);
  EXPECT_DOUBLE_EQ(angleController->target, 0);

  EXPECT_TRUE(distanceController->disabled);
  EXPECT_TRUE(angleController->disabled);

  assertMotorsHaveBeenStopped(leftMotor, rightMotor);
}

TEST_F(ChassisControllerPIDTest, MoveDistanceAsyncRawUnitsTest) {
  controller->moveDistanceAsync(100);

  EXPECT_DOUBLE_EQ(distanceController->target, 100);
  EXPECT_DOUBLE_EQ(angleController->target, 0);

  EXPECT_FALSE(distanceController->disabled);
  EXPECT_FALSE(angleController->disabled);

  controller->waitUntilSettled();

  EXPECT_TRUE(distanceController->disabled);
  EXPECT_TRUE(angleController->disabled);

  assertMotorsHaveBeenStopped(leftMotor, rightMotor);
}

TEST_F(ChassisControllerPIDTest, MoveDistanceAsyncUnitsTest) {
  controller->moveDistanceAsync(1_m);

  EXPECT_DOUBLE_EQ(distanceController->target, 2);
  EXPECT_DOUBLE_EQ(angleController->target, 0);

  EXPECT_FALSE(distanceController->disabled);
  EXPECT_FALSE(angleController->disabled);

  controller->waitUntilSettled();

  EXPECT_TRUE(distanceController->disabled);
  EXPECT_TRUE(angleController->disabled);

  assertMotorsHaveBeenStopped(leftMotor, rightMotor);
}

TEST_F(ChassisControllerPIDTest, TurnAngleRawUnitsTest) {
  controller->turnAngle(100);

  EXPECT_DOUBLE_EQ(angleController->target, 100);

  EXPECT_TRUE(angleController->disabled);

  assertMotorsHaveBeenStopped(leftMotor, rightMotor);
}

TEST_F(ChassisControllerPIDTest, TurnAngleUnitsTest) {
  controller->turnAngle(45_deg);

  EXPECT_DOUBLE_EQ(angleController->target, 90);

  EXPECT_TRUE(angleController->disabled);

  assertMotorsHaveBeenStopped(leftMotor, rightMotor);
}

TEST_F(ChassisControllerPIDTest, TurnAngleAsyncRawUnitsTest) {
  controller->turnAngleAsync(100);

  EXPECT_DOUBLE_EQ(angleController->target, 100);

  EXPECT_FALSE(angleController->disabled);

  controller->waitUntilSettled();

  EXPECT_TRUE(angleController->disabled);

  assertMotorsHaveBeenStopped(leftMotor, rightMotor);
}

TEST_F(ChassisControllerPIDTest, TurnAngleAsyncUnitsTest) {
  controller->turnAngleAsync(45_deg);

  EXPECT_DOUBLE_EQ(angleController->target, 90);

  EXPECT_FALSE(angleController->disabled);

  controller->waitUntilSettled();

  EXPECT_TRUE(angleController->disabled);

  assertMotorsHaveBeenStopped(leftMotor, rightMotor);
}

TEST_F(ChassisControllerPIDTest, MoveDistanceThenTurnAngleAsyncTest) {
  controller->moveDistanceAsync(100);

  EXPECT_DOUBLE_EQ(distanceController->target, 100);
  EXPECT_DOUBLE_EQ(angleController->target, 0);

  EXPECT_FALSE(distanceController->disabled);
  EXPECT_FALSE(angleController->disabled);

  controller->turnAngleAsync(200);

  EXPECT_DOUBLE_EQ(angleController->target, 200);

  EXPECT_TRUE(distanceController->disabled);
  EXPECT_FALSE(angleController->disabled);

  controller->waitUntilSettled();

  EXPECT_TRUE(distanceController->disabled);
  EXPECT_TRUE(angleController->disabled);

  assertMotorsHaveBeenStopped(leftMotor, rightMotor);
}

TEST_F(ChassisControllerPIDTest, TurnAngleThenMoveDistanceAsyncTest) {
  controller->turnAngleAsync(200);

  EXPECT_DOUBLE_EQ(angleController->target, 200);

  EXPECT_TRUE(distanceController->disabled);
  EXPECT_FALSE(angleController->disabled);

  controller->moveDistanceAsync(100);

  EXPECT_DOUBLE_EQ(distanceController->target, 100);
  EXPECT_DOUBLE_EQ(angleController->target, 0);

  EXPECT_FALSE(distanceController->disabled);
  EXPECT_FALSE(angleController->disabled);

  controller->waitUntilSettled();

  EXPECT_TRUE(distanceController->disabled);
  EXPECT_TRUE(angleController->disabled);

  assertMotorsHaveBeenStopped(leftMotor, rightMotor);
}

TEST_F(ChassisControllerPIDTest, MoveDistanceAndWaitTest) {
  controller->moveDistance(100);
  controller->waitUntilSettled();

  assertMotorsHaveBeenStopped(leftMotor, rightMotor);
}

TEST_F(ChassisControllerPIDTest, MoveDistanceGetBumpedAndWaitTest) {
  controller->moveDistance(100);

  // First bump
  leftMotor->encoder->value = 500;
  rightMotor->encoder->value = 500;
  controller->waitUntilSettled();

  // Second bump
  leftMotor->encoder->value = 1000;
  rightMotor->encoder->value = 1000;
  controller->waitUntilSettled();

  assertMotorsHaveBeenStopped(leftMotor, rightMotor);
}

TEST_F(ChassisControllerPIDTest, TurnAngleAndWaitTest) {
  controller->turnAngle(100);
  controller->waitUntilSettled();

  assertMotorsHaveBeenStopped(leftMotor, rightMotor);
}

TEST_F(ChassisControllerPIDTest, TurnAngleGetBumpedAndWaitTest) {
  controller->turnAngle(100);

  // First bump
  leftMotor->encoder->value = 500;
  rightMotor->encoder->value = 500;
  controller->waitUntilSettled();

  // Second bump
  leftMotor->encoder->value = 1000;
  rightMotor->encoder->value = 1000;
  controller->waitUntilSettled();

  assertMotorsHaveBeenStopped(leftMotor, rightMotor);
}

TEST_F(ChassisControllerPIDTest, MoveDistanceAndStopTest) {
  controller->moveDistanceAsync(100);
  controller->stop();

  assertMotorsHaveBeenStopped(leftMotor, rightMotor);
  EXPECT_TRUE(distanceController->isDisabled());
  EXPECT_TRUE(angleController->isDisabled());
}

TEST_F(ChassisControllerPIDTest, TurnAngleAndStopTest) {
  controller->turnAngleAsync(100);
  controller->stop();

  assertMotorsHaveBeenStopped(leftMotor, rightMotor);
  EXPECT_TRUE(angleController->isDisabled());
}
