/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/chassis/controller/chassisControllerPid.hpp"
#include "okapi/api/chassis/model/skidSteerModel.hpp"
#include "test/tests/api/implMocks.hpp"
#include <gtest/gtest.h>

using namespace okapi;

class ChassisControllerPIDTest : public ::testing::Test {
  protected:
  void SetUp() override {
    scales = new ChassisScales({wheelDiam, wheelTrack}, gearsetToTPR(gearset));

    distanceController = new MockIterativeController(0.1);
    turnController = new MockIterativeController(0.1);
    angleController = new MockIterativeController(0.1);

    model = new MockSkidSteerModel();
    leftMotor = model->leftMtr.get();
    rightMotor = model->rightMtr.get();

    controller =
      new ChassisControllerPID(createTimeUtil(),
                               std::shared_ptr<ChassisModel>(model),
                               std::unique_ptr<IterativePosPIDController>(distanceController),
                               std::unique_ptr<IterativePosPIDController>(turnController),
                               std::unique_ptr<IterativePosPIDController>(angleController),
                               gearset, // must match the tpr given to ChassisScales
                               *scales);
    controller->startThread();
  }

  void TearDown() override {
    delete scales;
    delete controller;
  }

  QLength wheelDiam = 4_in;
  QLength wheelTrack = 8_in;
  AbstractMotor::gearset gearset = AbstractMotor::gearset::green;
  ChassisScales *scales;
  ChassisControllerPID *controller;
  MockMotor *leftMotor;
  MockMotor *rightMotor;
  MockIterativeController *distanceController;
  MockIterativeController *turnController;
  MockIterativeController *angleController;
  MockSkidSteerModel *model;
};

TEST_F(ChassisControllerPIDTest, GearsetIsCorrect) {
  EXPECT_EQ(AbstractMotor::gearset::green, controller->getGearsetRatioPair().internalGearset);
  EXPECT_EQ(imev5GreenTPR, gearsetToTPR(controller->getGearsetRatioPair().internalGearset));
  EXPECT_EQ(1, controller->getGearsetRatioPair().ratio);
}

TEST_F(ChassisControllerPIDTest, MoveDistanceRawUnitsTest) {
  controller->moveDistance(100);

  EXPECT_DOUBLE_EQ(distanceController->getTarget(), 100);
  EXPECT_DOUBLE_EQ(angleController->getTarget(), 0);

  EXPECT_TRUE(turnController->isDisabled());
  EXPECT_TRUE(distanceController->isDisabled());
  EXPECT_TRUE(angleController->isDisabled());

  assertMotorsHaveBeenStopped(leftMotor, rightMotor);
}

TEST_F(ChassisControllerPIDTest, MoveDistanceUnitsTest) {
  controller->moveDistance(wheelDiam * 1_pi);

  EXPECT_DOUBLE_EQ(distanceController->getTarget(),
                   gearsetToTPR(controller->getGearsetRatioPair().internalGearset));
  EXPECT_DOUBLE_EQ(angleController->getTarget(), 0);

  EXPECT_TRUE(turnController->isDisabled());
  EXPECT_TRUE(distanceController->isDisabled());
  EXPECT_TRUE(angleController->isDisabled());

  assertMotorsHaveBeenStopped(leftMotor, rightMotor);
}

TEST_F(ChassisControllerPIDTest, MoveDistanceAsyncRawUnitsTest) {
  controller->moveDistanceAsync(100);

  EXPECT_DOUBLE_EQ(distanceController->getTarget(), 100);
  EXPECT_DOUBLE_EQ(angleController->getTarget(), 0);

  EXPECT_TRUE(turnController->isDisabled());
  EXPECT_FALSE(distanceController->isDisabled());
  EXPECT_FALSE(angleController->isDisabled());

  controller->waitUntilSettled();

  EXPECT_TRUE(turnController->isDisabled());
  EXPECT_TRUE(distanceController->isDisabled());
  EXPECT_TRUE(angleController->isDisabled());

  assertMotorsHaveBeenStopped(leftMotor, rightMotor);
}

TEST_F(ChassisControllerPIDTest, MoveDistanceAsyncUnitsTest) {
  controller->moveDistanceAsync(wheelDiam * 1_pi);

  EXPECT_DOUBLE_EQ(distanceController->getTarget(),
                   gearsetToTPR(controller->getGearsetRatioPair().internalGearset));
  EXPECT_DOUBLE_EQ(angleController->getTarget(), 0);

  EXPECT_TRUE(turnController->isDisabled());
  EXPECT_FALSE(distanceController->isDisabled());
  EXPECT_FALSE(angleController->isDisabled());

  controller->waitUntilSettled();

  EXPECT_TRUE(turnController->isDisabled());
  EXPECT_TRUE(distanceController->isDisabled());
  EXPECT_TRUE(angleController->isDisabled());

  assertMotorsHaveBeenStopped(leftMotor, rightMotor);
}

TEST_F(ChassisControllerPIDTest, TurnAngleRawUnitsTest) {
  controller->turnAngle(100);

  EXPECT_DOUBLE_EQ(angleController->getTarget(), 0);
  EXPECT_DOUBLE_EQ(turnController->getTarget(), 100);

  EXPECT_TRUE(distanceController->isDisabled());
  EXPECT_TRUE(angleController->isDisabled());
  EXPECT_TRUE(turnController->isDisabled());

  assertMotorsHaveBeenStopped(leftMotor, rightMotor);
}

TEST_F(ChassisControllerPIDTest, TurnAngleUnitsTest) {
  controller->turnAngle(wheelDiam / wheelTrack * 360_deg);

  EXPECT_DOUBLE_EQ(angleController->getTarget(), 0);
  EXPECT_DOUBLE_EQ(turnController->getTarget(),
                   gearsetToTPR(controller->getGearsetRatioPair().internalGearset));

  EXPECT_TRUE(distanceController->isDisabled());
  EXPECT_TRUE(angleController->isDisabled());
  EXPECT_TRUE(turnController->isDisabled());

  assertMotorsHaveBeenStopped(leftMotor, rightMotor);
}

TEST_F(ChassisControllerPIDTest, TurnAngleAsyncRawUnitsTest) {
  controller->turnAngleAsync(100);

  EXPECT_DOUBLE_EQ(angleController->getTarget(), 0);
  EXPECT_DOUBLE_EQ(turnController->getTarget(), 100);

  EXPECT_TRUE(distanceController->isDisabled());
  EXPECT_TRUE(angleController->isDisabled());
  EXPECT_FALSE(turnController->isDisabled());

  controller->waitUntilSettled();

  EXPECT_TRUE(distanceController->isDisabled());
  EXPECT_TRUE(angleController->isDisabled());
  EXPECT_TRUE(turnController->isDisabled());

  assertMotorsHaveBeenStopped(leftMotor, rightMotor);
}

TEST_F(ChassisControllerPIDTest, TurnAngleAsyncUnitsTest) {
  controller->turnAngleAsync(wheelDiam / wheelTrack * 360_deg);

  EXPECT_DOUBLE_EQ(angleController->getTarget(), 0);
  EXPECT_DOUBLE_EQ(turnController->getTarget(),
                   gearsetToTPR(controller->getGearsetRatioPair().internalGearset));

  EXPECT_TRUE(distanceController->isDisabled());
  EXPECT_TRUE(angleController->isDisabled());
  EXPECT_FALSE(turnController->isDisabled());

  controller->waitUntilSettled();

  EXPECT_TRUE(distanceController->isDisabled());
  EXPECT_TRUE(angleController->isDisabled());
  EXPECT_TRUE(turnController->isDisabled());

  assertMotorsHaveBeenStopped(leftMotor, rightMotor);
}

TEST_F(ChassisControllerPIDTest, MirrorTurnTest) {
  controller->setTurnsMirrored(true);
  controller->turnAngle(wheelDiam / wheelTrack * 360_deg);

  EXPECT_DOUBLE_EQ(distanceController->getTarget(), 0);
  EXPECT_DOUBLE_EQ(angleController->getTarget(), 0);
  EXPECT_DOUBLE_EQ(turnController->getTarget(),
                   -1 * gearsetToTPR(controller->getGearsetRatioPair().internalGearset));
}

TEST_F(ChassisControllerPIDTest, MoveDistanceThenTurnAngleAsyncTest) {
  controller->moveDistanceAsync(100);

  EXPECT_DOUBLE_EQ(distanceController->getTarget(), 100);
  EXPECT_DOUBLE_EQ(angleController->getTarget(), 0);

  EXPECT_FALSE(distanceController->isDisabled());
  EXPECT_FALSE(angleController->isDisabled());
  EXPECT_TRUE(turnController->isDisabled());

  controller->turnAngleAsync(200);

  EXPECT_DOUBLE_EQ(turnController->getTarget(), 200);

  EXPECT_TRUE(distanceController->isDisabled());
  EXPECT_TRUE(angleController->isDisabled());
  EXPECT_FALSE(turnController->isDisabled());

  controller->waitUntilSettled();

  EXPECT_TRUE(distanceController->isDisabled());
  EXPECT_TRUE(angleController->isDisabled());
  EXPECT_TRUE(turnController->isDisabled());

  assertMotorsHaveBeenStopped(leftMotor, rightMotor);
}

TEST_F(ChassisControllerPIDTest, TurnAngleThenMoveDistanceAsyncTest) {
  controller->turnAngleAsync(200);

  EXPECT_DOUBLE_EQ(turnController->getTarget(), 200);

  EXPECT_TRUE(distanceController->isDisabled());
  EXPECT_TRUE(angleController->isDisabled());
  EXPECT_FALSE(turnController->isDisabled());

  controller->moveDistanceAsync(100);

  EXPECT_DOUBLE_EQ(distanceController->getTarget(), 100);
  EXPECT_DOUBLE_EQ(angleController->getTarget(), 0);

  EXPECT_FALSE(distanceController->isDisabled());
  EXPECT_FALSE(angleController->isDisabled());
  EXPECT_TRUE(turnController->isDisabled());

  controller->waitUntilSettled();

  EXPECT_TRUE(distanceController->isDisabled());
  EXPECT_TRUE(angleController->isDisabled());
  EXPECT_TRUE(turnController->isDisabled());

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

TEST_F(ChassisControllerPIDTest, WaitUntilSettledInModeNone) {
  controller->waitUntilSettled();

  EXPECT_TRUE(turnController->isDisabled());
  EXPECT_TRUE(distanceController->isDisabled());
  EXPECT_TRUE(angleController->isDisabled());
}

TEST_F(ChassisControllerPIDTest, GetGainsReturnsTheSetGains) {
  controller->setGains({0.1, 0.2, 0.3, 0.4}, {0.5, 0.6, 0.7, 0.8}, {0.9, 1.0, 1.1, 1.2});
  auto [distanceGains, turnGains, angleGains] = controller->getGains();

  EXPECT_FLOAT_EQ(distanceGains.kP, 0.1);
  EXPECT_FLOAT_EQ(distanceGains.kI, 0.2);
  EXPECT_FLOAT_EQ(distanceGains.kD, 0.3);
  EXPECT_FLOAT_EQ(distanceGains.kBias, 0.4);

  EXPECT_FLOAT_EQ(turnGains.kP, 0.5);
  EXPECT_FLOAT_EQ(turnGains.kI, 0.6);
  EXPECT_FLOAT_EQ(turnGains.kD, 0.7);
  EXPECT_FLOAT_EQ(turnGains.kBias, 0.8);

  EXPECT_FLOAT_EQ(angleGains.kP, 0.9);
  EXPECT_FLOAT_EQ(angleGains.kI, 1.0);
  EXPECT_FLOAT_EQ(angleGains.kD, 1.1);
  EXPECT_FLOAT_EQ(angleGains.kBias, 1.2);
}
