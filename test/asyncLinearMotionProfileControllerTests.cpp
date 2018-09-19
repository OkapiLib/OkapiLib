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

class AsyncLinearMotionProfileControllerTest : public ::testing::Test {
  protected:
  void SetUp() override {
    output = new MockAsyncVelIntegratedController();

    controller = new AsyncLinearMotionProfileController(
      createTimeUtil(), 1.0, 2.0, 10.0, std::shared_ptr<MockAsyncVelIntegratedController>(output));
    controller->startThread();
  }

  void TearDown() override {
    delete controller;
  }

  MockAsyncVelIntegratedController *output;
  AsyncLinearMotionProfileController *controller;
};

TEST_F(AsyncLinearMotionProfileControllerTest, SettledWhenDisabled) {
  assertControllerIsSettledWhenDisabled(*controller, std::string("A"));
}

TEST_F(AsyncLinearMotionProfileControllerTest, WaitUntilSettledWorksWhenDisabled) {
  assertWaitUntilSettledWorksWhenDisabled(*controller);
}

TEST_F(AsyncLinearMotionProfileControllerTest, MoveToTest) {
  controller->moveTo(0_ft, 3_ft);
  EXPECT_EQ(output->lastTarget, 0);
  EXPECT_GT(output->maxTarget, 0);
}

TEST_F(AsyncLinearMotionProfileControllerTest, MotorsAreStoppedAfterSettling) {
  controller->generatePath({0_m, 3_ft}, "A");

  EXPECT_EQ(controller->getPaths().front(), "A");
  EXPECT_EQ(controller->getPaths().size(), 1);

  controller->setTarget("A");

  EXPECT_EQ(controller->getTarget(), "A");

  controller->waitUntilSettled();

  EXPECT_EQ(output->lastTarget, 0);
  EXPECT_GT(output->maxTarget, 0);
}

TEST_F(AsyncLinearMotionProfileControllerTest, WrongPathNameDoesNotMoveAnything) {
  controller->setTarget("A");
  controller->waitUntilSettled();

  EXPECT_EQ(output->lastTarget, 0);
  EXPECT_EQ(output->maxTarget, 0);
}

TEST_F(AsyncLinearMotionProfileControllerTest, TwoPathsOverwriteEachOther) {
  controller->generatePath({0_m, 3_ft}, "A");
  controller->generatePath({0_m, 4_ft}, "A");

  EXPECT_EQ(controller->getPaths().front(), "A");
  EXPECT_EQ(controller->getPaths().size(), 1);

  controller->setTarget("A");
  controller->waitUntilSettled();
  EXPECT_EQ(output->lastTarget, 0);
  EXPECT_GT(output->maxTarget, 0);
}

TEST_F(AsyncLinearMotionProfileControllerTest, ZeroWaypointsDoesNothing) {
  controller->generatePath({}, "A");
  EXPECT_EQ(controller->getPaths().size(), 0);
}

TEST_F(AsyncLinearMotionProfileControllerTest, RemoveAPath) {
  controller->generatePath({0_m, 3_ft}, "A");

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
