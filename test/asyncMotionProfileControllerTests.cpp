/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/control/async/asyncMotionProfileController.hpp"
#include "test/tests/api/implMocks.hpp"
#include <gtest/gtest.h>

using namespace okapi;

class MockAsyncMotionProfileController : public AsyncMotionProfileController {
  public:
  using AsyncMotionProfileController::AsyncMotionProfileController;

  void executeSinglePath(const TrajectoryPair &path, std::unique_ptr<AbstractRate> rate) override {
    executeSinglePathCalled = true;
    AsyncMotionProfileController::executeSinglePath(path, std::move(rate));
  }

  bool executeSinglePathCalled{false};
};

class AsyncMotionProfileControllerTest : public ::testing::Test {
  protected:
  void SetUp() override {
    leftMotor = std::make_shared<MockMotor>();
    rightMotor = std::make_shared<MockMotor>();

    model = new SkidSteerModel(leftMotor, rightMotor, 100);

    controller = new MockAsyncMotionProfileController(createTimeUtil(),
                                                      1.0,
                                                      2.0,
                                                      10.0,
                                                      std::shared_ptr<SkidSteerModel>(model),
                                                      {4_in, 10.5_in},
                                                      AbstractMotor::gearset::green);
    controller->startThread();
  }

  void TearDown() override {
    delete controller;
  }

  std::shared_ptr<MockMotor> leftMotor;
  std::shared_ptr<MockMotor> rightMotor;
  SkidSteerModel *model;
  MockAsyncMotionProfileController *controller;
};

TEST_F(AsyncMotionProfileControllerTest, SettledWhenDisabled) {
  assertControllerIsSettledWhenDisabled(*controller, std::string("A"));
}

TEST_F(AsyncMotionProfileControllerTest, WaitUntilSettledWorksWhenDisabled) {
  assertWaitUntilSettledWorksWhenDisabled(*controller);
}

TEST_F(AsyncMotionProfileControllerTest, MotorsAreStoppedAfterSettling) {
  controller->generatePath({Point{0_m, 0_m, 0_deg}, Point{3_ft, 0_m, 45_deg}}, "A");

  EXPECT_EQ(controller->getPaths().front(), "A");
  EXPECT_EQ(controller->getPaths().size(), 1);

  controller->setTarget("A");

  EXPECT_EQ(controller->getTarget(), "A");

  controller->waitUntilSettled();

  assertMotorsHaveBeenStopped(leftMotor.get(), rightMotor.get());
  EXPECT_GT(leftMotor->maxVelocity, 0);
  EXPECT_GT(rightMotor->maxVelocity, 0);
}
TEST_F(AsyncMotionProfileControllerTest, WrongPathNameDoesNotMoveAnything) {
  controller->setTarget("A");
  controller->waitUntilSettled();

  EXPECT_EQ(leftMotor->maxVelocity, 0);
  EXPECT_EQ(rightMotor->maxVelocity, 0);
}

TEST_F(AsyncMotionProfileControllerTest, TwoPathsOverwriteEachOther) {
  controller->generatePath({Point{0_m, 0_m, 0_deg}, Point{3_ft, 0_m, 45_deg}}, "A");
  controller->generatePath({Point{0_m, 0_m, 0_deg}, Point{3_ft, 2_ft, 45_deg}}, "A");

  EXPECT_EQ(controller->getPaths().front(), "A");
  EXPECT_EQ(controller->getPaths().size(), 1);

  controller->setTarget("A");
  controller->waitUntilSettled();
  assertMotorsHaveBeenStopped(leftMotor.get(), rightMotor.get());
  EXPECT_GT(leftMotor->maxVelocity, 0);
  EXPECT_GT(rightMotor->maxVelocity, 0);
}

TEST_F(AsyncMotionProfileControllerTest, ImpossiblePathThrowsException) {
  EXPECT_THROW(controller->generatePath({Point{0_m, 0_m, 0_deg},
                                         Point{3_ft, 0_m, 0_deg},
                                         Point{3_ft, 1_ft, 0_deg},
                                         Point{2_ft, 1_ft, 0_deg},
                                         Point{1_ft, 1_m, 0_deg},
                                         Point{1_ft, 0_m, 0_deg}},
                                        "A"),
               std::runtime_error);
  EXPECT_EQ(controller->getPaths().size(), 0);
}

TEST_F(AsyncMotionProfileControllerTest, ZeroWaypointsDoesNothing) {
  controller->generatePath({}, "A");
  EXPECT_EQ(controller->getPaths().size(), 0);
}

TEST_F(AsyncMotionProfileControllerTest, RemoveAPath) {
  controller->generatePath({Point{0_m, 0_m, 0_deg}, Point{3_ft, 0_m, 45_deg}}, "A");

  EXPECT_EQ(controller->getPaths().front(), "A");
  EXPECT_EQ(controller->getPaths().size(), 1);

  controller->removePath("A");

  EXPECT_EQ(controller->getPaths().size(), 0);
}

TEST_F(AsyncMotionProfileControllerTest, RemoveAPathWhichDoesNotExist) {
  EXPECT_EQ(controller->getPaths().size(), 0);

  controller->removePath("A");

  EXPECT_EQ(controller->getPaths().size(), 0);
}

TEST_F(AsyncMotionProfileControllerTest, ControllerSetChangesTarget) {
  controller->controllerSet("A");
  EXPECT_EQ(controller->getTarget(), "A");
}

TEST_F(AsyncMotionProfileControllerTest, ResetStopsMotors) {
  controller->generatePath({Point{0_m, 0_m, 0_deg}, Point{3_ft, 0_m, 45_deg}}, "A");
  controller->setTarget("A");

  auto rate = createTimeUtil().getRate();
  while (!controller->executeSinglePathCalled) {
    rate->delayUntil(1_ms);
  }

  // Wait a little longer so we get into the path
  rate->delayUntil(200_ms);
  EXPECT_GT(leftMotor->maxVelocity, 0);
  EXPECT_GT(rightMotor->maxVelocity, 0);

  controller->reset();
  EXPECT_FALSE(controller->isDisabled());
  EXPECT_TRUE(controller->isSettled());
  EXPECT_EQ(leftMotor->lastVelocity, 0);
  EXPECT_EQ(rightMotor->lastVelocity, 0);
}

TEST_F(AsyncMotionProfileControllerTest, DisabledStopsMotors) {
  controller->generatePath({Point{0_m, 0_m, 0_deg}, Point{3_ft, 0_m, 45_deg}}, "A");
  controller->setTarget("A");

  auto rate = createTimeUtil().getRate();
  while (!controller->executeSinglePathCalled) {
    rate->delayUntil(1_ms);
  }

  // Wait a little longer so we get into the path
  rate->delayUntil(200_ms);
  EXPECT_GT(leftMotor->maxVelocity, 0);
  EXPECT_GT(rightMotor->maxVelocity, 0);

  controller->flipDisable(true);

  // Wait a bit because the loop() thread is what cleans up
  rate->delayUntil(10_ms);

  EXPECT_TRUE(controller->isDisabled());
  EXPECT_TRUE(controller->isSettled());
  EXPECT_EQ(leftMotor->lastVelocity, 0);
  EXPECT_EQ(rightMotor->lastVelocity, 0);
}
