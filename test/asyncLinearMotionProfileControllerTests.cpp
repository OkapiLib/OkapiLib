/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/control/async/asyncLinearMotionProfileController.hpp"
#include "test/tests/api/implMocks.hpp"
#include <gtest/gtest.h>

using namespace okapi;

class MockAsyncLinearMotionProfileController : public AsyncLinearMotionProfileController {
  public:
  using AsyncLinearMotionProfileController::AsyncLinearMotionProfileController;

  void executeSinglePath(const TrajectoryPair &path, std::unique_ptr<AbstractRate> rate) override {
    executeSinglePathCalled = true;
    AsyncLinearMotionProfileController::executeSinglePath(path, std::move(rate));
  }

  bool executeSinglePathCalled{false};
};

class AsyncLinearMotionProfileControllerTest : public ::testing::Test {
  protected:
  void SetUp() override {
    output = new MockAsyncVelIntegratedController();

    controller = new MockAsyncLinearMotionProfileController(
      createTimeUtil(), 1.0, 2.0, 10.0, std::shared_ptr<MockAsyncVelIntegratedController>(output));
    controller->startThread();
  }

  void TearDown() override {
    delete controller;
  }

  MockAsyncVelIntegratedController *output;
  MockAsyncLinearMotionProfileController *controller;
};

TEST_F(AsyncLinearMotionProfileControllerTest, SettledWhenDisabled) {
  assertControllerIsSettledWhenDisabled(*controller, std::string("A"));
}

TEST_F(AsyncLinearMotionProfileControllerTest, WaitUntilSettledWorksWhenDisabled) {
  assertWaitUntilSettledWorksWhenDisabled(*controller);
}

TEST_F(AsyncLinearMotionProfileControllerTest, MoveToTest) {
  controller->moveTo(0, 3);
  EXPECT_EQ(output->lastControllerOutputSet, 0);
  EXPECT_GT(output->maxControllerOutputSet, 0);
}

TEST_F(AsyncLinearMotionProfileControllerTest, MotorsAreStoppedAfterSettling) {
  controller->generatePath({0, 3}, "A");

  EXPECT_EQ(controller->getPaths().front(), "A");
  EXPECT_EQ(controller->getPaths().size(), 1);

  controller->setTarget("A");

  EXPECT_EQ(controller->getTarget(), "A");

  controller->waitUntilSettled();

  EXPECT_EQ(output->lastControllerOutputSet, 0);
  EXPECT_GT(output->maxControllerOutputSet, 0);
}

TEST_F(AsyncLinearMotionProfileControllerTest, WrongPathNameDoesNotMoveAnything) {
  controller->setTarget("A");
  controller->waitUntilSettled();

  EXPECT_EQ(output->lastControllerOutputSet, 0);
  EXPECT_EQ(output->maxControllerOutputSet, 0);
}

TEST_F(AsyncLinearMotionProfileControllerTest, TwoPathsOverwriteEachOther) {
  controller->generatePath({0, 3}, "A");
  controller->generatePath({0, 4}, "A");

  EXPECT_EQ(controller->getPaths().front(), "A");
  EXPECT_EQ(controller->getPaths().size(), 1);

  controller->setTarget("A");
  controller->waitUntilSettled();
  EXPECT_EQ(output->lastControllerOutputSet, 0);
  EXPECT_GT(output->maxControllerOutputSet, 0);
}

TEST_F(AsyncLinearMotionProfileControllerTest, ZeroWaypointsDoesNothing) {
  controller->generatePath({}, "A");
  EXPECT_EQ(controller->getPaths().size(), 0);
}

TEST_F(AsyncLinearMotionProfileControllerTest, RemoveAPath) {
  controller->generatePath({0, 3}, "A");

  EXPECT_EQ(controller->getPaths().front(), "A");
  EXPECT_EQ(controller->getPaths().size(), 1);

  controller->removePath("A");

  EXPECT_EQ(controller->getPaths().size(), 0);
}

TEST_F(AsyncLinearMotionProfileControllerTest, RemoveAPathWhichDoesNotExist) {
  EXPECT_EQ(controller->getPaths().size(), 0);

  controller->removePath("A");

  EXPECT_EQ(controller->getPaths().size(), 0);
}

TEST_F(AsyncLinearMotionProfileControllerTest, ControllerSetChangesTarget) {
  controller->controllerSet("A");
  EXPECT_EQ(controller->getTarget(), "A");
}

TEST_F(AsyncLinearMotionProfileControllerTest, GetErrorWithNoTarget) {
  EXPECT_EQ(controller->getError(), 0);
}

TEST_F(AsyncLinearMotionProfileControllerTest, GetErrorWithNonexistentTarget) {
  controller->setTarget("A");
  EXPECT_EQ(controller->getError(), 0);
}

TEST_F(AsyncLinearMotionProfileControllerTest, GetErrorWithCorrectTarget) {
  controller->generatePath({0, 3}, "A");
  controller->setTarget("A");

  // Pathfinder generates an approximate path so this could be slightly off
  EXPECT_NEAR(controller->getError(), 3, 0.1);
}

TEST_F(AsyncLinearMotionProfileControllerTest, ResetStopsMotors) {
  controller->generatePath({0, 3}, "A");
  controller->setTarget("A");

  auto rate = createTimeUtil().getRate();
  while (!controller->executeSinglePathCalled) {
    rate->delayUntil(1_ms);
  }

  // Wait a little longer so we get into the path
  rate->delayUntil(200_ms);
  EXPECT_GT(output->lastControllerOutputSet, 0);

  controller->reset();
  EXPECT_FALSE(controller->isDisabled());
  EXPECT_TRUE(controller->isSettled());
  EXPECT_EQ(output->lastControllerOutputSet, 0);
}

TEST_F(AsyncLinearMotionProfileControllerTest, DisabledStopsMotors) {
  controller->generatePath({0, 3}, "A");
  controller->setTarget("A");

  auto rate = createTimeUtil().getRate();
  while (!controller->executeSinglePathCalled) {
    rate->delayUntil(1_ms);
  }

  // Wait a little longer so we get into the path
  rate->delayUntil(200_ms);
  EXPECT_GT(output->lastControllerOutputSet, 0);

  controller->flipDisable(true);

  // Wait a bit because the loop() thread is what cleans up
  rate->delayUntil(10_ms);

  EXPECT_TRUE(controller->isDisabled());
  EXPECT_TRUE(controller->isSettled());
  EXPECT_EQ(output->lastControllerOutputSet, 0);
}
