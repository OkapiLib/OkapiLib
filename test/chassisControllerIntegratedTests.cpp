/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/chassis/controller/chassisControllerIntegrated.hpp"
#include "okapi/api/chassis/model/skidSteerModel.hpp"
#include "test/tests/api/implMocks.hpp"
#include <gtest/gtest.h>

using namespace okapi;

class ChassisControllerIntegratedTest : public ::testing::Test { // NOLINT(hicpp-member-init)
  protected:
  void SetUp() override {
    scales = new ChassisScales({wheelDiam, wheelTrack}, gearsetToTPR(gearset));

    leftController = new MockAsyncPosIntegratedController();
    rightController = new MockAsyncPosIntegratedController();

    leftController->isSettledOverride = IsSettledOverride::alwaysSettled;
    rightController->isSettledOverride = IsSettledOverride::alwaysSettled;

    model = new MockSkidSteerModel();
    model->setMaxVelocity(101);
    leftMotor = model->leftMtr.get();
    rightMotor = model->rightMtr.get();

    controller = new ChassisControllerIntegrated(
      createTimeUtil(),
      std::shared_ptr<ChassisModel>(model),
      std::unique_ptr<AsyncPosIntegratedController>(leftController),
      std::unique_ptr<AsyncPosIntegratedController>(rightController),
      gearset,
      *scales);
  }

  void TearDown() override {
    delete scales;
    delete controller;
  }

  QLength wheelDiam = 4_in;
  QLength wheelTrack = 8_in;
  AbstractMotor::gearset gearset = AbstractMotor::gearset::green;
  ChassisScales *scales;
  ChassisControllerIntegrated *controller;
  MockMotor *leftMotor;
  MockMotor *rightMotor;
  MockAsyncPosIntegratedController *leftController;
  MockAsyncPosIntegratedController *rightController;
  MockSkidSteerModel *model;
};

TEST_F(ChassisControllerIntegratedTest, InitialMaxVelocityIsPropagatedToControllers) {
  EXPECT_EQ(leftController->maxVelocity, 101);
  EXPECT_EQ(rightController->maxVelocity, 101);
}

TEST_F(ChassisControllerIntegratedTest, GearsetIsCorrect) {
  EXPECT_EQ(AbstractMotor::gearset::green, controller->getGearsetRatioPair().internalGearset);
  EXPECT_EQ(imev5GreenTPR, gearsetToTPR(controller->getGearsetRatioPair().internalGearset));
  EXPECT_EQ(1, controller->getGearsetRatioPair().ratio);
}

TEST_F(ChassisControllerIntegratedTest, MoveDistanceRawUnitsTest) {
  controller->moveRaw(100);

  EXPECT_DOUBLE_EQ(leftController->getTarget(), 100);
  EXPECT_DOUBLE_EQ(rightController->getTarget(), 100);

  EXPECT_TRUE(leftController->isDisabled());
  EXPECT_TRUE(rightController->isDisabled());

  assertMotorsHaveBeenStopped(leftMotor, rightMotor);
}

TEST_F(ChassisControllerIntegratedTest, MoveDistanceUnitsTest) {
  controller->moveDistance(wheelDiam * 1_pi);

  EXPECT_DOUBLE_EQ(leftController->getTarget(),
                   gearsetToTPR(controller->getGearsetRatioPair().internalGearset));
  EXPECT_DOUBLE_EQ(rightController->getTarget(),
                   gearsetToTPR(controller->getGearsetRatioPair().internalGearset));

  EXPECT_TRUE(leftController->isDisabled());
  EXPECT_TRUE(rightController->isDisabled());

  assertMotorsHaveBeenStopped(leftMotor, rightMotor);
}

TEST_F(ChassisControllerIntegratedTest, MoveDistanceAsyncRawUnitsTest) {
  controller->moveRawAsync(100);

  EXPECT_DOUBLE_EQ(leftController->getTarget(), 100);
  EXPECT_DOUBLE_EQ(rightController->getTarget(), 100);

  EXPECT_FALSE(leftController->isDisabled());
  EXPECT_FALSE(rightController->isDisabled());

  controller->waitUntilSettled();

  EXPECT_TRUE(leftController->isDisabled());
  EXPECT_TRUE(rightController->isDisabled());

  assertMotorsHaveBeenStopped(leftMotor, rightMotor);
}

TEST_F(ChassisControllerIntegratedTest, MoveDistanceAsyncUnitsTest) {
  controller->moveDistanceAsync(wheelDiam * 1_pi);

  EXPECT_DOUBLE_EQ(leftController->getTarget(),
                   gearsetToTPR(controller->getGearsetRatioPair().internalGearset));
  EXPECT_DOUBLE_EQ(rightController->getTarget(),
                   gearsetToTPR(controller->getGearsetRatioPair().internalGearset));

  EXPECT_FALSE(leftController->isDisabled());
  EXPECT_FALSE(rightController->isDisabled());

  controller->waitUntilSettled();

  EXPECT_DOUBLE_EQ(leftController->getTarget(),
                   gearsetToTPR(controller->getGearsetRatioPair().internalGearset));
  EXPECT_DOUBLE_EQ(rightController->getTarget(),
                   gearsetToTPR(controller->getGearsetRatioPair().internalGearset));

  EXPECT_TRUE(leftController->isDisabled());
  EXPECT_TRUE(rightController->isDisabled());

  assertMotorsHaveBeenStopped(leftMotor, rightMotor);
}

TEST_F(ChassisControllerIntegratedTest, TurnAngleRawUnitsTest) {
  controller->turnRaw(100);

  EXPECT_DOUBLE_EQ(leftController->getTarget(), 100);
  EXPECT_DOUBLE_EQ(rightController->getTarget(), -100);

  EXPECT_TRUE(leftController->isDisabled());
  EXPECT_TRUE(rightController->isDisabled());

  assertMotorsHaveBeenStopped(leftMotor, rightMotor);
}

TEST_F(ChassisControllerIntegratedTest, TurnAngleUnitsTest) {
  controller->turnAngle(wheelDiam / wheelTrack * 360_deg);

  EXPECT_DOUBLE_EQ(leftController->getTarget(),
                   gearsetToTPR(controller->getGearsetRatioPair().internalGearset));
  EXPECT_DOUBLE_EQ(rightController->getTarget(),
                   -1 * gearsetToTPR(controller->getGearsetRatioPair().internalGearset));

  EXPECT_TRUE(leftController->isDisabled());
  EXPECT_TRUE(rightController->isDisabled());

  assertMotorsHaveBeenStopped(leftMotor, rightMotor);
}

TEST_F(ChassisControllerIntegratedTest, TurnAngleAsyncRawUnitsTest) {
  controller->turnRawAsync(100);

  EXPECT_DOUBLE_EQ(leftController->getTarget(), 100);
  EXPECT_DOUBLE_EQ(rightController->getTarget(), -100);

  EXPECT_FALSE(leftController->isDisabled());
  EXPECT_FALSE(rightController->isDisabled());

  controller->waitUntilSettled();

  EXPECT_TRUE(leftController->isDisabled());
  EXPECT_TRUE(rightController->isDisabled());

  assertMotorsHaveBeenStopped(leftMotor, rightMotor);
}

TEST_F(ChassisControllerIntegratedTest, TurnAngleAsyncUnitsTest) {
  controller->turnAngleAsync(wheelDiam / wheelTrack * 360_deg);

  EXPECT_DOUBLE_EQ(leftController->getTarget(),
                   gearsetToTPR(controller->getGearsetRatioPair().internalGearset));
  EXPECT_DOUBLE_EQ(rightController->getTarget(),
                   -1 * gearsetToTPR(controller->getGearsetRatioPair().internalGearset));

  EXPECT_FALSE(leftController->isDisabled());
  EXPECT_FALSE(rightController->isDisabled());

  controller->waitUntilSettled();

  EXPECT_TRUE(leftController->isDisabled());
  EXPECT_TRUE(rightController->isDisabled());

  assertMotorsHaveBeenStopped(leftMotor, rightMotor);
}

TEST_F(ChassisControllerIntegratedTest, MirrorTurnTest) {
  controller->setTurnsMirrored(true);
  controller->turnAngle(wheelDiam / wheelTrack * 360_deg);

  EXPECT_DOUBLE_EQ(leftController->getTarget(),
                   -1 * gearsetToTPR(controller->getGearsetRatioPair().internalGearset));
  EXPECT_DOUBLE_EQ(rightController->getTarget(),
                   gearsetToTPR(controller->getGearsetRatioPair().internalGearset));
}

TEST_F(ChassisControllerIntegratedTest, MoveDistanceThenTurnAngleAsyncTest) {
  controller->moveRawAsync(100);

  EXPECT_DOUBLE_EQ(leftController->getTarget(), 100);
  EXPECT_DOUBLE_EQ(rightController->getTarget(), 100);

  EXPECT_FALSE(leftController->isDisabled());
  EXPECT_FALSE(rightController->isDisabled());

  controller->turnRawAsync(200);

  EXPECT_DOUBLE_EQ(leftController->getTarget(), 200);
  EXPECT_DOUBLE_EQ(rightController->getTarget(), -200);

  EXPECT_FALSE(leftController->isDisabled());
  EXPECT_FALSE(rightController->isDisabled());

  controller->waitUntilSettled();

  EXPECT_TRUE(leftController->isDisabled());
  EXPECT_TRUE(rightController->isDisabled());

  assertMotorsHaveBeenStopped(leftMotor, rightMotor);
}

TEST_F(ChassisControllerIntegratedTest, TurnAngleThenMoveDistanceAsyncTest) {
  controller->turnRawAsync(200);

  EXPECT_DOUBLE_EQ(leftController->getTarget(), 200);
  EXPECT_DOUBLE_EQ(rightController->getTarget(), -200);

  EXPECT_FALSE(leftController->isDisabled());
  EXPECT_FALSE(rightController->isDisabled());

  controller->moveRawAsync(100);

  EXPECT_DOUBLE_EQ(leftController->getTarget(), 100);
  EXPECT_DOUBLE_EQ(rightController->getTarget(), 100);

  EXPECT_FALSE(leftController->isDisabled());
  EXPECT_FALSE(rightController->isDisabled());

  controller->waitUntilSettled();

  EXPECT_TRUE(leftController->isDisabled());
  EXPECT_TRUE(rightController->isDisabled());

  assertMotorsHaveBeenStopped(leftMotor, rightMotor);
}

TEST_F(ChassisControllerIntegratedTest, MoveDistanceAndWaitTest) {
  controller->moveRaw(100);
  controller->waitUntilSettled();

  assertMotorsHaveBeenStopped(leftMotor, rightMotor);
}

TEST_F(ChassisControllerIntegratedTest, MoveDistanceGetPushedAndWaitTest) {
  controller->moveRaw(100);

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
  controller->turnRaw(100);
  controller->waitUntilSettled();

  assertMotorsHaveBeenStopped(leftMotor, rightMotor);
}

TEST_F(ChassisControllerIntegratedTest, TurnAngleGetPushedAndWaitTest) {
  controller->turnRaw(100);

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
  controller->moveRawAsync(100);
  controller->stop();

  assertMotorsHaveBeenStopped(leftMotor, rightMotor);
  EXPECT_TRUE(leftController->isDisabled());
  EXPECT_TRUE(rightController->isDisabled());
}

TEST_F(ChassisControllerIntegratedTest, TurnAngleAndStopTest) {
  controller->turnRawAsync(100);
  controller->stop();

  assertMotorsHaveBeenStopped(leftMotor, rightMotor);
  EXPECT_TRUE(leftController->isDisabled());
  EXPECT_TRUE(rightController->isDisabled());
}

TEST_F(ChassisControllerIntegratedTest, SetMaxVelocityTest) {
  controller->setMaxVelocity(42);
  EXPECT_EQ(leftController->maxVelocity, 42);
  EXPECT_EQ(rightController->maxVelocity, 42);
  EXPECT_EQ(model->getMaxVelocity(), 42);
}

TEST_F(ChassisControllerIntegratedTest, isNotSettledWhenLeftControllerIsNotSettled) {
  leftController->isSettledOverride = IsSettledOverride::neverSettled;
  rightController->isSettledOverride = IsSettledOverride::alwaysSettled;
  EXPECT_FALSE(controller->isSettled());
}

TEST_F(ChassisControllerIntegratedTest, isNotSettledWhenRightControllerIsNotSettled) {
  leftController->isSettledOverride = IsSettledOverride::alwaysSettled;
  rightController->isSettledOverride = IsSettledOverride::neverSettled;
  EXPECT_FALSE(controller->isSettled());
}

TEST_F(ChassisControllerIntegratedTest, isSettledWhenLeftAndRightControllersAreSettled) {
  leftController->isSettledOverride = IsSettledOverride::alwaysSettled;
  rightController->isSettledOverride = IsSettledOverride::alwaysSettled;
  EXPECT_TRUE(controller->isSettled());
}

TEST_F(ChassisControllerIntegratedTest, SetsTheCorrectTargetWhenCustomSensorsAreOnTheModelForMove) {
  auto model = new MockSkidSteerModel();
  // Set the values on the model (which we are pretending has ADI encoders) to be nonzero
  // to check that CCI is not reading from them. The motors' integrated encoders should still be
  // at zero.
  model->leftEnc->value = 100;
  model->rightEnc->value = -100;

  auto leftController = new MockAsyncPosIntegratedController();
  auto rightController = new MockAsyncPosIntegratedController();
  leftController->isSettledOverride = IsSettledOverride::alwaysSettled;
  rightController->isSettledOverride = IsSettledOverride::alwaysSettled;

  ChassisControllerIntegrated chassis(
    createTimeUtil(),
    std::shared_ptr<ChassisModel>(model),
    std::unique_ptr<AsyncPosIntegratedController>(leftController),
    std::unique_ptr<AsyncPosIntegratedController>(rightController),
    gearset,
    ChassisScales({wheelDiam, wheelTrack}, gearsetToTPR(gearset)));

  chassis.moveRaw(200);

  EXPECT_DOUBLE_EQ(leftController->getTarget(), 200);
  EXPECT_DOUBLE_EQ(rightController->getTarget(), 200);
}

TEST_F(ChassisControllerIntegratedTest, SetsTheCorrectTargetWhenCustomSensorsAreOnTheModelForTurn) {
  auto model = new MockSkidSteerModel();
  // Set the values on the model (which we are pretending has ADI encoders) to be nonzero
  // to check that CCI is not reading from them. The motors' integrated encoders should still be
  // at zero.
  model->leftEnc->value = 100;
  model->rightEnc->value = -100;

  auto leftController = new MockAsyncPosIntegratedController();
  auto rightController = new MockAsyncPosIntegratedController();
  leftController->isSettledOverride = IsSettledOverride::alwaysSettled;
  rightController->isSettledOverride = IsSettledOverride::alwaysSettled;

  ChassisControllerIntegrated chassis(
    createTimeUtil(),
    std::shared_ptr<ChassisModel>(model),
    std::unique_ptr<AsyncPosIntegratedController>(leftController),
    std::unique_ptr<AsyncPosIntegratedController>(rightController),
    gearset,
    ChassisScales({wheelDiam, wheelTrack}, gearsetToTPR(gearset)));

  chassis.turnRaw(200);

  EXPECT_DOUBLE_EQ(leftController->getTarget(), 200);
  EXPECT_DOUBLE_EQ(rightController->getTarget(), -200);
}
