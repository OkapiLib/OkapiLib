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

class AsyncMotionProfileControllerTest : public ::testing::Test {
  protected:
  void SetUp() override {
    leftMotor = new MockMotor();
    rightMotor = new MockMotor();

    model = new SkidSteerModel(std::unique_ptr<AbstractMotor>(leftMotor),
                               std::unique_ptr<AbstractMotor>(rightMotor));

    controller = new AsyncMotionProfileController(
      createTimeUtil(), 1.0, 2.0, 10.0, std::shared_ptr<SkidSteerModel>(model), 10.5_in);
    controller->startThread();
  }

  void TearDown() override {
    delete controller;
  }

  MockMotor *leftMotor;
  MockMotor *rightMotor;
  SkidSteerModel *model;
  AsyncMotionProfileController *controller;
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

  assertMotorsHaveBeenStopped(leftMotor, rightMotor);
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
  assertMotorsHaveBeenStopped(leftMotor, rightMotor);
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
