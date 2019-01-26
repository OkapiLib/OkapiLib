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

class MockSkidSteerModel : public SkidSteerModel {
  public:
  using SkidSteerModel::maxVelocity;
  using SkidSteerModel::SkidSteerModel;
};

class ChassisControllerPIDIntegrationTest : public ::testing::Test {
  protected:
  void SetUp() override {
    scales = new ChassisScales({4_in, 11.5_in});
    leftMotor = new ThreadedMockMotor();
    rightMotor = new ThreadedMockMotor();

    distanceController = new IterativePosPIDController(0.1, 0, 0, 0, createTimeUtil());
    angleController = new IterativePosPIDController(0.1, 0, 0, 0, createTimeUtil());
    turnController = new IterativePosPIDController(0.1, 0, 0, 0, createTimeUtil());

    model = new SkidSteerModel(
      std::unique_ptr<AbstractMotor>(leftMotor), std::unique_ptr<AbstractMotor>(rightMotor), 200);

    controller =
      new ChassisControllerPID(createTimeUtil(),
                               std::shared_ptr<ChassisModel>(model),
                               std::unique_ptr<IterativePosPIDController>(distanceController),
                               std::unique_ptr<IterativePosPIDController>(angleController),
                               std::unique_ptr<IterativePosPIDController>(turnController),
                               AbstractMotor::gearset::red,
                               *scales);

    leftMotor->startThread();
    rightMotor->startThread();
    controller->startThread();
  }

  void TearDown() override {
    leftMotor->stopThread();
    rightMotor->stopThread();

    leftMotor->thread.join();
    rightMotor->thread.join();

    delete scales;
    delete controller;
  }

  ChassisScales *scales;
  ChassisControllerPID *controller;
  ThreadedMockMotor *leftMotor;
  ThreadedMockMotor *rightMotor;
  IterativePosPIDController *distanceController;
  IterativePosPIDController *angleController;
  IterativePosPIDController *turnController;
  SkidSteerModel *model;
};

TEST_F(ChassisControllerPIDIntegrationTest, MoveDistanceRawUnitsTest) {
  controller->moveDistance(100);

  EXPECT_DOUBLE_EQ(distanceController->getTarget(), 100);
  EXPECT_DOUBLE_EQ(angleController->getTarget(), 0);

  EXPECT_TRUE(turnController->isDisabled());
  EXPECT_TRUE(distanceController->isDisabled());
  EXPECT_TRUE(angleController->isDisabled());

  // Wait a bit extra in case the thread is still writing to the motors
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  EXPECT_EQ(controller->getSensorVals()[0], 100);
  EXPECT_EQ(controller->getSensorVals()[1], 100);
  EXPECT_EQ(leftMotor->getTargetVelocity(), 0);
  EXPECT_EQ(rightMotor->getTargetVelocity(), 0);
}

// TEST_F(ChassisControllerPIDIntegrationTest, MoveDistanceUnitsTest) {
//  controller->moveDistance(1_m);
//
//  EXPECT_DOUBLE_EQ(distanceController->getTarget(), 2);
//  EXPECT_DOUBLE_EQ(angleController->getTarget(), 0);
//
//  EXPECT_TRUE(turnController->isDisabled());
//  EXPECT_TRUE(distanceController->isDisabled());
//  EXPECT_TRUE(angleController->isDisabled());
//
//  //  assertMotorsHaveBeenStopped(leftMotor, rightMotor);
//}
//
// TEST_F(ChassisControllerPIDIntegrationTest, MoveDistanceAsyncRawUnitsTest) {
//  controller->moveDistanceAsync(100);
//
//  EXPECT_DOUBLE_EQ(distanceController->getTarget(), 100);
//  EXPECT_DOUBLE_EQ(angleController->getTarget(), 0);
//
//  EXPECT_TRUE(turnController->isDisabled());
//  EXPECT_FALSE(distanceController->isDisabled());
//  EXPECT_FALSE(angleController->isDisabled());
//
//  controller->waitUntilSettled();
//
//  EXPECT_TRUE(turnController->isDisabled());
//  EXPECT_TRUE(distanceController->isDisabled());
//  EXPECT_TRUE(angleController->isDisabled());
//
//  //  assertMotorsHaveBeenStopped(leftMotor, rightMotor);
//}
//
// TEST_F(ChassisControllerPIDIntegrationTest, MoveDistanceAsyncUnitsTest) {
//  controller->moveDistanceAsync(1_m);
//
//  EXPECT_DOUBLE_EQ(distanceController->getTarget(), 2);
//  EXPECT_DOUBLE_EQ(angleController->getTarget(), 0);
//
//  EXPECT_TRUE(turnController->isDisabled());
//  EXPECT_FALSE(distanceController->isDisabled());
//  EXPECT_FALSE(angleController->isDisabled());
//
//  controller->waitUntilSettled();
//
//  EXPECT_TRUE(turnController->isDisabled());
//  EXPECT_TRUE(distanceController->isDisabled());
//  EXPECT_TRUE(angleController->isDisabled());
//
//  //  assertMotorsHaveBeenStopped(leftMotor, rightMotor);
//}
//
// TEST_F(ChassisControllerPIDIntegrationTest, TurnAngleRawUnitsTest) {
//  controller->turnAngle(100);
//
//  EXPECT_DOUBLE_EQ(angleController->getTarget(), 0);
//  EXPECT_DOUBLE_EQ(turnController->getTarget(), 100);
//
//  EXPECT_TRUE(distanceController->isDisabled());
//  EXPECT_TRUE(angleController->isDisabled());
//  EXPECT_TRUE(turnController->isDisabled());
//
//  //  assertMotorsHaveBeenStopped(leftMotor, rightMotor);
//}
//
// TEST_F(ChassisControllerPIDIntegrationTest, TurnAngleUnitsTest) {
//  controller->turnAngle(45_deg);
//
//  EXPECT_DOUBLE_EQ(angleController->getTarget(), 0);
//  EXPECT_DOUBLE_EQ(turnController->getTarget(), 90);
//
//  EXPECT_TRUE(distanceController->isDisabled());
//  EXPECT_TRUE(angleController->isDisabled());
//  EXPECT_TRUE(turnController->isDisabled());
//
//  //  assertMotorsHaveBeenStopped(leftMotor, rightMotor);
//}
//
// TEST_F(ChassisControllerPIDIntegrationTest, TurnAngleAsyncRawUnitsTest) {
//  controller->turnAngleAsync(100);
//
//  EXPECT_DOUBLE_EQ(angleController->getTarget(), 0);
//  EXPECT_DOUBLE_EQ(turnController->getTarget(), 100);
//
//  EXPECT_TRUE(distanceController->isDisabled());
//  EXPECT_TRUE(angleController->isDisabled());
//  EXPECT_FALSE(turnController->isDisabled());
//
//  controller->waitUntilSettled();
//
//  EXPECT_TRUE(distanceController->isDisabled());
//  EXPECT_TRUE(angleController->isDisabled());
//  EXPECT_TRUE(turnController->isDisabled());
//
//  //  assertMotorsHaveBeenStopped(leftMotor, rightMotor);
//}
//
// TEST_F(ChassisControllerPIDIntegrationTest, TurnAngleAsyncUnitsTest) {
//  controller->turnAngleAsync(45_deg);
//
//  EXPECT_DOUBLE_EQ(angleController->getTarget(), 0);
//  EXPECT_DOUBLE_EQ(turnController->getTarget(), 90);
//
//  EXPECT_TRUE(distanceController->isDisabled());
//  EXPECT_TRUE(angleController->isDisabled());
//  EXPECT_FALSE(turnController->isDisabled());
//
//  controller->waitUntilSettled();
//
//  EXPECT_TRUE(distanceController->isDisabled());
//  EXPECT_TRUE(angleController->isDisabled());
//  EXPECT_TRUE(turnController->isDisabled());
//
//  //  assertMotorsHaveBeenStopped(leftMotor, rightMotor);
//}
//
// TEST_F(ChassisControllerPIDIntegrationTest, MirrorTurnTest) {
//  controller->setTurnsMirrored(true);
//  controller->turnAngle(45_deg);
//
//  EXPECT_DOUBLE_EQ(distanceController->getTarget(), 0);
//  EXPECT_DOUBLE_EQ(angleController->getTarget(), 0);
//  EXPECT_DOUBLE_EQ(turnController->getTarget(), -90);
//}
//
// TEST_F(ChassisControllerPIDIntegrationTest, MoveDistanceThenTurnAngleAsyncTest) {
//  controller->moveDistanceAsync(100);
//
//  EXPECT_DOUBLE_EQ(distanceController->getTarget(), 100);
//  EXPECT_DOUBLE_EQ(angleController->getTarget(), 0);
//
//  EXPECT_FALSE(distanceController->isDisabled());
//  EXPECT_FALSE(angleController->isDisabled());
//  EXPECT_TRUE(turnController->isDisabled());
//
//  controller->turnAngleAsync(200);
//
//  EXPECT_DOUBLE_EQ(turnController->getTarget(), 200);
//
//  EXPECT_TRUE(distanceController->isDisabled());
//  EXPECT_TRUE(angleController->isDisabled());
//  EXPECT_FALSE(turnController->isDisabled());
//
//  controller->waitUntilSettled();
//
//  EXPECT_TRUE(distanceController->isDisabled());
//  EXPECT_TRUE(angleController->isDisabled());
//  EXPECT_TRUE(turnController->isDisabled());
//
//  //  assertMotorsHaveBeenStopped(leftMotor, rightMotor);
//}
//
// TEST_F(ChassisControllerPIDIntegrationTest, TurnAngleThenMoveDistanceAsyncTest) {
//  controller->turnAngleAsync(200);
//
//  EXPECT_DOUBLE_EQ(turnController->getTarget(), 200);
//
//  EXPECT_TRUE(distanceController->isDisabled());
//  EXPECT_TRUE(angleController->isDisabled());
//  EXPECT_FALSE(turnController->isDisabled());
//
//  controller->moveDistanceAsync(100);
//
//  EXPECT_DOUBLE_EQ(distanceController->getTarget(), 100);
//  EXPECT_DOUBLE_EQ(angleController->getTarget(), 0);
//
//  EXPECT_FALSE(distanceController->isDisabled());
//  EXPECT_FALSE(angleController->isDisabled());
//  EXPECT_TRUE(turnController->isDisabled());
//
//  controller->waitUntilSettled();
//
//  EXPECT_TRUE(distanceController->isDisabled());
//  EXPECT_TRUE(angleController->isDisabled());
//  EXPECT_TRUE(turnController->isDisabled());
//
//  //  assertMotorsHaveBeenStopped(leftMotor, rightMotor);
//}
//
// TEST_F(ChassisControllerPIDIntegrationTest, MoveDistanceAndWaitTest) {
//  controller->moveDistance(100);
//  controller->waitUntilSettled();
//
//  //  assertMotorsHaveBeenStopped(leftMotor, rightMotor);
//}
//
// TEST_F(ChassisControllerPIDIntegrationTest, MoveDistanceGetBumpedAndWaitTest) {
//  controller->moveDistance(100);
//
//  // First bump
//  leftMotor->encoder->value = 500;
//  rightMotor->encoder->value = 500;
//  controller->waitUntilSettled();
//
//  // Second bump
//  leftMotor->encoder->value = 1000;
//  rightMotor->encoder->value = 1000;
//  controller->waitUntilSettled();
//
//  //  assertMotorsHaveBeenStopped(leftMotor, rightMotor);
//}
//
// TEST_F(ChassisControllerPIDIntegrationTest, TurnAngleAndWaitTest) {
//  controller->turnAngle(100);
//  controller->waitUntilSettled();
//
//  //  assertMotorsHaveBeenStopped(leftMotor, rightMotor);
//}
//
// TEST_F(ChassisControllerPIDIntegrationTest, TurnAngleGetBumpedAndWaitTest) {
//  controller->turnAngle(100);
//
//  // First bump
//  leftMotor->encoder->value = 500;
//  rightMotor->encoder->value = 500;
//  controller->waitUntilSettled();
//
//  // Second bump
//  leftMotor->encoder->value = 1000;
//  rightMotor->encoder->value = 1000;
//  controller->waitUntilSettled();
//
//  //  assertMotorsHaveBeenStopped(leftMotor, rightMotor);
//}
//
// TEST_F(ChassisControllerPIDIntegrationTest, MoveDistanceAndStopTest) {
//  controller->moveDistanceAsync(100);
//  controller->stop();
//
//  //  assertMotorsHaveBeenStopped(leftMotor, rightMotor);
//  EXPECT_TRUE(distanceController->isDisabled());
//  EXPECT_TRUE(angleController->isDisabled());
//}
//
// TEST_F(ChassisControllerPIDIntegrationTest, TurnAngleAndStopTest) {
//  controller->turnAngleAsync(100);
//  controller->stop();
//
//  //  assertMotorsHaveBeenStopped(leftMotor, rightMotor);
//  EXPECT_TRUE(angleController->isDisabled());
//}
//
// TEST_F(ChassisControllerPIDIntegrationTest, WaitUntilSettledInModeNone) {
//  controller->waitUntilSettled();
//
//  EXPECT_TRUE(turnController->isDisabled());
//  EXPECT_TRUE(distanceController->isDisabled());
//  EXPECT_TRUE(angleController->isDisabled());
//}
//
// TEST_F(ChassisControllerPIDIntegrationTest, SetMaxVelocityTest) {
//  controller->setMaxVelocity(42);
//  //  EXPECT_EQ(model->maxVelocity, 42);
//}
