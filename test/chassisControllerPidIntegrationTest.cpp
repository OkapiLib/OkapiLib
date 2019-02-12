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

    auto controllerTimeUtil = createTimeUtil(
      Supplier<std::unique_ptr<SettledUtil>>([]() { return createSettledUtilPtr(250, 5, 1_s); }));

    distanceController = new IterativePosPIDController(0.1, 0, 0, 0, controllerTimeUtil);
    angleController = new IterativePosPIDController(0.1, 0, 0, 0, controllerTimeUtil);
    turnController = new IterativePosPIDController(0.1, 0, 0, 0, controllerTimeUtil);

    model = new SkidSteerModel(
      std::unique_ptr<AbstractMotor>(leftMotor), std::unique_ptr<AbstractMotor>(rightMotor), 200);

    controller =
      new ChassisControllerPID(createTimeUtil(),
                               std::shared_ptr<ChassisModel>(model),
                               std::unique_ptr<IterativePosPIDController>(distanceController),
                               std::unique_ptr<IterativePosPIDController>(turnController),
                               std::unique_ptr<IterativePosPIDController>(angleController),
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

  EXPECT_NEAR(controller->getSensorVals()[0], 100, 10);
  EXPECT_NEAR(controller->getSensorVals()[1], 100, 10);
  EXPECT_EQ(leftMotor->getTargetVelocity(), 0);
  EXPECT_EQ(rightMotor->getTargetVelocity(), 0);
}

TEST_F(ChassisControllerPIDIntegrationTest, MoveDistanceUnitsTest) {
  controller->moveDistance(1_m);

  EXPECT_NEAR(distanceController->getTarget(), 1128, 1);
  EXPECT_DOUBLE_EQ(angleController->getTarget(), 0);

  EXPECT_TRUE(turnController->isDisabled());
  EXPECT_TRUE(distanceController->isDisabled());
  EXPECT_TRUE(angleController->isDisabled());

  // Wait a bit extra in case the thread is still writing to the motors
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  EXPECT_NEAR(controller->getSensorVals()[0], 1128, 10);
  EXPECT_NEAR(controller->getSensorVals()[1], 1128, 10);
  EXPECT_EQ(leftMotor->getTargetVelocity(), 0);
  EXPECT_EQ(rightMotor->getTargetVelocity(), 0);
}

TEST_F(ChassisControllerPIDIntegrationTest, MoveDistanceAsyncRawUnitsTest) {
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

  // Wait a bit extra in case the thread is still writing to the motors
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  EXPECT_NEAR(controller->getSensorVals()[0], 100, 10);
  EXPECT_NEAR(controller->getSensorVals()[1], 100, 10);
  EXPECT_EQ(leftMotor->getTargetVelocity(), 0);
  EXPECT_EQ(rightMotor->getTargetVelocity(), 0);
}

TEST_F(ChassisControllerPIDIntegrationTest, MoveDistanceAsyncUnitsTest) {
  controller->moveDistanceAsync(1_m);

  EXPECT_NEAR(distanceController->getTarget(), 1128, 1);
  EXPECT_DOUBLE_EQ(angleController->getTarget(), 0);

  EXPECT_TRUE(turnController->isDisabled());
  EXPECT_FALSE(distanceController->isDisabled());
  EXPECT_FALSE(angleController->isDisabled());

  controller->waitUntilSettled();

  EXPECT_TRUE(turnController->isDisabled());
  EXPECT_TRUE(distanceController->isDisabled());
  EXPECT_TRUE(angleController->isDisabled());

  // Wait a bit extra in case the thread is still writing to the motors
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  EXPECT_NEAR(controller->getSensorVals()[0], 1128, 10);
  EXPECT_NEAR(controller->getSensorVals()[1], 1128, 10);
  EXPECT_EQ(leftMotor->getTargetVelocity(), 0);
  EXPECT_EQ(rightMotor->getTargetVelocity(), 0);
}

TEST_F(ChassisControllerPIDIntegrationTest, TurnAngleRawUnitsTest) {
  controller->turnAngle(100);

  EXPECT_DOUBLE_EQ(turnController->getTarget(), 100);

  EXPECT_TRUE(distanceController->isDisabled());
  EXPECT_TRUE(angleController->isDisabled());
  EXPECT_TRUE(turnController->isDisabled());

  // Wait a bit extra in case the thread is still writing to the motors
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  EXPECT_NEAR(controller->getSensorVals()[0], 100, 10);
  EXPECT_NEAR(controller->getSensorVals()[1], -100, 10);
  EXPECT_EQ(leftMotor->getTargetVelocity(), 0);
  EXPECT_EQ(rightMotor->getTargetVelocity(), 0);
}

TEST_F(ChassisControllerPIDIntegrationTest, TurnAngleUnitsTest) {
  controller->turnAngle(45_deg);

  EXPECT_NEAR(turnController->getTarget(), 129, 1);

  EXPECT_TRUE(distanceController->isDisabled());
  EXPECT_TRUE(angleController->isDisabled());
  EXPECT_TRUE(turnController->isDisabled());

  // Wait a bit extra in case the thread is still writing to the motors
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  EXPECT_NEAR(controller->getSensorVals()[0], 129, 10);
  EXPECT_NEAR(controller->getSensorVals()[1], -129, 10);
  EXPECT_EQ(leftMotor->getTargetVelocity(), 0);
  EXPECT_EQ(rightMotor->getTargetVelocity(), 0);
}

TEST_F(ChassisControllerPIDIntegrationTest, TurnAngleAsyncRawUnitsTest) {
  controller->turnAngleAsync(100);

  EXPECT_DOUBLE_EQ(turnController->getTarget(), 100);

  EXPECT_TRUE(distanceController->isDisabled());
  EXPECT_TRUE(angleController->isDisabled());
  EXPECT_FALSE(turnController->isDisabled());

  controller->waitUntilSettled();

  EXPECT_TRUE(distanceController->isDisabled());
  EXPECT_TRUE(angleController->isDisabled());
  EXPECT_TRUE(turnController->isDisabled());

  // Wait a bit extra in case the thread is still writing to the motors
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  EXPECT_NEAR(controller->getSensorVals()[0], 100, 10);
  EXPECT_NEAR(controller->getSensorVals()[1], -100, 10);
  EXPECT_EQ(leftMotor->getTargetVelocity(), 0);
  EXPECT_EQ(rightMotor->getTargetVelocity(), 0);
}

TEST_F(ChassisControllerPIDIntegrationTest, TurnAngleAsyncUnitsTest) {
  controller->turnAngleAsync(45_deg);

  EXPECT_NEAR(turnController->getTarget(), 129, 1);

  EXPECT_TRUE(distanceController->isDisabled());
  EXPECT_TRUE(angleController->isDisabled());
  EXPECT_FALSE(turnController->isDisabled());

  controller->waitUntilSettled();

  EXPECT_TRUE(distanceController->isDisabled());
  EXPECT_TRUE(angleController->isDisabled());
  EXPECT_TRUE(turnController->isDisabled());

  // Wait a bit extra in case the thread is still writing to the motors
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  EXPECT_NEAR(controller->getSensorVals()[0], 129, 10);
  EXPECT_NEAR(controller->getSensorVals()[1], -129, 10);
  EXPECT_EQ(leftMotor->getTargetVelocity(), 0);
  EXPECT_EQ(rightMotor->getTargetVelocity(), 0);
}

TEST_F(ChassisControllerPIDIntegrationTest, MirrorTurnTest) {
  controller->setTurnsMirrored(true);
  controller->turnAngle(45_deg);

  EXPECT_NEAR(turnController->getTarget(), -129, 1);

  // Wait a bit extra in case the thread is still writing to the motors
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  EXPECT_NEAR(controller->getSensorVals()[0], -129, 10);
  EXPECT_NEAR(controller->getSensorVals()[1], 129, 10);
  EXPECT_EQ(leftMotor->getTargetVelocity(), 0);
  EXPECT_EQ(rightMotor->getTargetVelocity(), 0);
}

TEST_F(ChassisControllerPIDIntegrationTest, MoveDistanceThenTurnAngleAsyncTest) {
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

  // Wait a bit extra in case the thread is still writing to the motors
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  EXPECT_NEAR(controller->getSensorVals()[0], 200, 10);
  EXPECT_NEAR(controller->getSensorVals()[1], -200, 10);
  EXPECT_EQ(leftMotor->getTargetVelocity(), 0);
  EXPECT_EQ(rightMotor->getTargetVelocity(), 0);
}

TEST_F(ChassisControllerPIDIntegrationTest, TurnAngleThenMoveDistanceAsyncTest) {
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

  // Wait a bit extra in case the thread is still writing to the motors
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  EXPECT_NEAR(controller->getSensorVals()[0], 100, 10);
  EXPECT_NEAR(controller->getSensorVals()[1], 100, 10);
  EXPECT_EQ(leftMotor->getTargetVelocity(), 0);
  EXPECT_EQ(rightMotor->getTargetVelocity(), 0);
}

TEST_F(ChassisControllerPIDIntegrationTest, MoveDistanceAndWaitTest) {
  controller->moveDistance(100);
  controller->waitUntilSettled();

  EXPECT_TRUE(distanceController->isDisabled());
  EXPECT_TRUE(angleController->isDisabled());
  EXPECT_TRUE(turnController->isDisabled());

  // Wait a bit extra in case the thread is still writing to the motors
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  EXPECT_NEAR(controller->getSensorVals()[0], 100, 10);
  EXPECT_NEAR(controller->getSensorVals()[1], 100, 10);
  EXPECT_EQ(leftMotor->getTargetVelocity(), 0);
  EXPECT_EQ(rightMotor->getTargetVelocity(), 0);
}

TEST_F(ChassisControllerPIDIntegrationTest, TurnAngleAndWaitTest) {
  controller->turnAngle(100);
  controller->waitUntilSettled();

  EXPECT_TRUE(distanceController->isDisabled());
  EXPECT_TRUE(angleController->isDisabled());
  EXPECT_TRUE(turnController->isDisabled());

  // Wait a bit extra in case the thread is still writing to the motors
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  EXPECT_NEAR(controller->getSensorVals()[0], 100, 10);
  EXPECT_NEAR(controller->getSensorVals()[1], -100, 10);
  EXPECT_EQ(leftMotor->getTargetVelocity(), 0);
  EXPECT_EQ(rightMotor->getTargetVelocity(), 0);
}

TEST_F(ChassisControllerPIDIntegrationTest, MoveDistanceAsyncAndStopTest) {
  controller->moveDistanceAsync(100);
  controller->stop();

  EXPECT_TRUE(distanceController->isDisabled());
  EXPECT_TRUE(angleController->isDisabled());
  EXPECT_TRUE(turnController->isDisabled());

  // Wait a bit extra in case the thread is still writing to the motors
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  EXPECT_NEAR(controller->getSensorVals()[0], 0, 10);
  EXPECT_NEAR(controller->getSensorVals()[1], 0, 10);
  EXPECT_EQ(leftMotor->getTargetVelocity(), 0);
  EXPECT_EQ(rightMotor->getTargetVelocity(), 0);
}

TEST_F(ChassisControllerPIDIntegrationTest, TurnAngleAsyncAndStopTest) {
  controller->turnAngleAsync(100);
  controller->stop();

  EXPECT_TRUE(distanceController->isDisabled());
  EXPECT_TRUE(angleController->isDisabled());
  EXPECT_TRUE(turnController->isDisabled());

  // Wait a bit extra in case the thread is still writing to the motors
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  EXPECT_NEAR(controller->getSensorVals()[0], 0, 10);
  EXPECT_NEAR(controller->getSensorVals()[1], 0, 10);
  EXPECT_EQ(leftMotor->getTargetVelocity(), 0);
  EXPECT_EQ(rightMotor->getTargetVelocity(), 0);
}

TEST_F(ChassisControllerPIDIntegrationTest, WaitUntilSettledInModeNone) {
  controller->waitUntilSettled();

  EXPECT_TRUE(turnController->isDisabled());
  EXPECT_TRUE(distanceController->isDisabled());
  EXPECT_TRUE(angleController->isDisabled());
}
