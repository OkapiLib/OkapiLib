/*
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
  using AsyncMotionProfileController::convertLinearToRotational;
  using AsyncMotionProfileController::internalLoadPath;
  using AsyncMotionProfileController::internalStorePath;
  using AsyncMotionProfileController::makeFilePath;

  void executeSinglePath(const TrajectoryPair &path, std::unique_ptr<AbstractRate> rate) override {
    executeSinglePathCalled = true;
    AsyncMotionProfileController::executeSinglePath(path, std::move(rate));
  }

  TrajectoryPair &getPathData(std::string ipathId) {
    return paths.at(ipathId);
  }

  bool executeSinglePathCalled{false};
};

class AsyncMotionProfileControllerTest : public ::testing::Test {
  protected:
  void SetUp() override {
    leftPathFile = open_memstream(&leftFileBuf, &leftFileSize);
    rightPathFile = open_memstream(&rightFileBuf, &rightFileSize);

    leftMotor = std::make_shared<MockMotor>();
    rightMotor = std::make_shared<MockMotor>();

    model = new SkidSteerModel(leftMotor,
                               rightMotor,
                               leftMotor->getEncoder(),
                               rightMotor->getEncoder(),
                               100,
                               v5MotorMaxVoltage);

    controller = new MockAsyncMotionProfileController(createTimeUtil(),
                                                      {1.0, 2.0, 10.0},
                                                      std::shared_ptr<SkidSteerModel>(model),
                                                      {{4_in, 10.5_in}, quadEncoderTPR},
                                                      AbstractMotor::gearset::green * (1.0 / 2));
    controller->startThread();
  }

  void TearDown() override {
    fclose(leftPathFile);
    fclose(rightPathFile);
    free(leftFileBuf);
    free(rightFileBuf);
    delete controller;
  }

  std::shared_ptr<MockMotor> leftMotor;
  std::shared_ptr<MockMotor> rightMotor;
  SkidSteerModel *model;
  MockAsyncMotionProfileController *controller;

  FILE *leftPathFile;
  FILE *rightPathFile;
  char *leftFileBuf;
  char *rightFileBuf;
  size_t leftFileSize;
  size_t rightFileSize;
};

TEST_F(AsyncMotionProfileControllerTest, ConstructWithGearRatioOf0) {
  EXPECT_THROW(
    AsyncMotionProfileController(
      createTimeUtil(), {}, nullptr, {{2_in, 8_in}, 360}, AbstractMotor::gearset::green * 0),
    std::invalid_argument);
}

TEST_F(AsyncMotionProfileControllerTest, SettledWhenDisabled) {
  assertControllerIsSettledWhenDisabled(*controller, std::string("A"));
}

TEST_F(AsyncMotionProfileControllerTest, WaitUntilSettledWorksWhenDisabled) {
  assertWaitUntilSettledWorksWhenDisabled(*controller);
}

TEST_F(AsyncMotionProfileControllerTest, MotorsAreStoppedAfterSettling) {
  controller->generatePath({PathfinderPoint{0_m, 0_m, 0_deg}, PathfinderPoint{3_ft, 0_m, 45_deg}},
                           "A");

  EXPECT_EQ(controller->getPaths().front(), "A");
  EXPECT_EQ(controller->getPaths().size(), 1);

  controller->setTarget("A");

  EXPECT_EQ(controller->getTarget(), "A");

  controller->waitUntilSettled();

  assertMotorsHaveBeenStopped(leftMotor.get(), rightMotor.get());
  EXPECT_GT(leftMotor->maxVelocity, 0);
  EXPECT_GT(rightMotor->maxVelocity, 0);
}

TEST_F(AsyncMotionProfileControllerTest, FollowPathWithMoveTo) {
  controller->moveTo({PathfinderPoint{0_m, 0_m, 0_deg}, PathfinderPoint{3_ft, 0_m, 0_deg}});

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
  controller->generatePath({PathfinderPoint{0_m, 0_m, 0_deg}, PathfinderPoint{3_ft, 0_m, 45_deg}},
                           "A");
  controller->generatePath({PathfinderPoint{0_m, 0_m, 0_deg}, PathfinderPoint{3_ft, 2_ft, 45_deg}},
                           "A");

  EXPECT_EQ(controller->getPaths().front(), "A");
  EXPECT_EQ(controller->getPaths().size(), 1);

  controller->setTarget("A");
  controller->waitUntilSettled();
  assertMotorsHaveBeenStopped(leftMotor.get(), rightMotor.get());
  EXPECT_GT(leftMotor->maxVelocity, 0);
  EXPECT_GT(rightMotor->maxVelocity, 0);
}

TEST_F(AsyncMotionProfileControllerTest, ImpossiblePathThrowsException) {
  EXPECT_THROW(controller->generatePath({PathfinderPoint{0_m, 0_m, 0_deg},
                                         PathfinderPoint{3_ft, 0_m, 0_deg},
                                         PathfinderPoint{3_ft, 1_ft, 0_deg},
                                         PathfinderPoint{2_ft, 1_ft, 0_deg},
                                         PathfinderPoint{1_ft, 1_m, 0_deg},
                                         PathfinderPoint{1_ft, 0_m, 0_deg}},
                                        "A"),
               std::runtime_error);
  EXPECT_EQ(controller->getPaths().size(), 0);
}

TEST_F(AsyncMotionProfileControllerTest, ZeroWaypointsDoesNothing) {
  controller->generatePath({}, "A");
  EXPECT_EQ(controller->getPaths().size(), 0);
}

TEST_F(AsyncMotionProfileControllerTest, RemoveAPath) {
  controller->generatePath({PathfinderPoint{0_m, 0_m, 0_deg}, PathfinderPoint{3_ft, 0_m, 45_deg}},
                           "A");

  EXPECT_EQ(controller->getPaths().front(), "A");
  EXPECT_EQ(controller->getPaths().size(), 1);

  EXPECT_TRUE(controller->removePath("A"));

  EXPECT_EQ(controller->getPaths().size(), 0);
}

TEST_F(AsyncMotionProfileControllerTest, RemoveRunningPath) {
  controller->generatePath({PathfinderPoint{0_m, 0_m, 0_deg}, PathfinderPoint{3_ft, 0_m, 45_deg}},
                           "A");

  EXPECT_EQ(controller->getPaths().front(), "A");
  EXPECT_EQ(controller->getPaths().size(), 1);

  controller->setTarget("A");

  EXPECT_FALSE(controller->removePath("A"));

  EXPECT_EQ(controller->getPaths().size(), 1);
}

TEST_F(AsyncMotionProfileControllerTest, ReplaceRunningPath) {
  controller->generatePath({PathfinderPoint{0_m, 0_m, 0_deg}, PathfinderPoint{3_ft, 0_m, 45_deg}},
                           "A");

  controller->setTarget("A");
  controller->flipDisable(false);

  controller->generatePath({PathfinderPoint{0_m, 0_m, 0_deg}, PathfinderPoint{3_ft, 3_ft, 45_deg}},
                           "A");
  EXPECT_EQ(controller->isDisabled(), true);

  EXPECT_EQ(controller->getPaths().size(), 1);
}

TEST_F(AsyncMotionProfileControllerTest, RemoveAPathWhichDoesNotExist) {
  EXPECT_EQ(controller->getPaths().size(), 0);

  EXPECT_TRUE(controller->removePath("A"));

  EXPECT_EQ(controller->getPaths().size(), 0);
}

TEST_F(AsyncMotionProfileControllerTest, ControllerSetChangesTarget) {
  controller->controllerSet("A");
  EXPECT_EQ(controller->getTarget(), "A");
}

TEST_F(AsyncMotionProfileControllerTest, ResetStopsMotors) {
  controller->generatePath({PathfinderPoint{0_m, 0_m, 0_deg}, PathfinderPoint{3_ft, 0_m, 45_deg}},
                           "A");
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
  controller->generatePath({PathfinderPoint{0_m, 0_m, 0_deg}, PathfinderPoint{3_ft, 0_m, 45_deg}},
                           "A");
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

TEST_F(AsyncMotionProfileControllerTest, SpeedConversionTest) {
  // 4 inch wheels, 2 wheel rotations per 1 motor rotation
  EXPECT_NEAR(controller->convertLinearToRotational(1_mps).convert(rpm), 93.989, 0.001);
}

TEST_F(AsyncMotionProfileControllerTest, FollowPathBackwards) {
  controller->generatePath({PathfinderPoint{0_m, 0_m, 0_deg}, PathfinderPoint{3_ft, 0_m, 0_deg}},
                           "A");
  controller->setTarget("A", true);

  auto rate = createTimeUtil().getRate();
  while (!controller->executeSinglePathCalled) {
    rate->delayUntil(1_ms);
  }

  // Wait a little longer so we get into the path
  rate->delayUntil(200_ms);

  EXPECT_LT(leftMotor->lastVelocity, 0);
  EXPECT_LT(rightMotor->lastVelocity, 0);

  // Disable the controller so gtest doesn't clean up the test fixture while the internal thread is
  // still running
  controller->flipDisable(true);
}

TEST_F(AsyncMotionProfileControllerTest, FollowPathNotMirrored) {
  controller->generatePath({PathfinderPoint{0_m, 0_m, 0_deg}, PathfinderPoint{1_ft, 1_ft, 0_deg}},
                           "A");
  controller->setTarget("A");

  auto rate = createTimeUtil().getRate();
  while (!controller->executeSinglePathCalled) {
    rate->delayUntil(1_ms);
  }

  // Wait a little longer so we get into the path
  rate->delayUntil(200_ms);

  EXPECT_GT(leftMotor->lastVelocity, 0);
  EXPECT_GT(rightMotor->lastVelocity, 0);
  EXPECT_GT(rightMotor->maxVelocity, leftMotor->maxVelocity);

  // Disable the controller so gtest doesn't clean up the test fixture while the internal thread is
  // still running
  controller->flipDisable(true);
}

TEST_F(AsyncMotionProfileControllerTest, FollowPathMirrored) {
  controller->generatePath({PathfinderPoint{0_m, 0_m, 0_deg}, PathfinderPoint{1_ft, 1_ft, 0_deg}},
                           "A");
  controller->setTarget("A", false, true);

  auto rate = createTimeUtil().getRate();
  while (!controller->executeSinglePathCalled) {
    rate->delayUntil(1_ms);
  }

  // Wait a little longer so we get into the path
  rate->delayUntil(200_ms);

  EXPECT_GT(leftMotor->lastVelocity, 0);
  EXPECT_GT(rightMotor->lastVelocity, 0);
  EXPECT_GT(leftMotor->maxVelocity, rightMotor->maxVelocity);

  // Disable the controller so gtest doesn't clean up the test fixture while the internal thread is
  // still running
  controller->flipDisable(true);
}

TEST_F(AsyncMotionProfileControllerTest, FilePathJoin) {
  EXPECT_STREQ(MockAsyncMotionProfileController::makeFilePath("/usd/", "test").c_str(),
               "/usd/test");
  EXPECT_STREQ(MockAsyncMotionProfileController::makeFilePath("usd/", "test").c_str(), "/usd/test");
  EXPECT_STREQ(MockAsyncMotionProfileController::makeFilePath("/usd", "test").c_str(), "/usd/test");
  EXPECT_STREQ(MockAsyncMotionProfileController::makeFilePath("usd", "test").c_str(), "/usd/test");
  EXPECT_STREQ(MockAsyncMotionProfileController::makeFilePath("", "test").c_str(), "/usd/test");
  EXPECT_STREQ(MockAsyncMotionProfileController::makeFilePath("/", "test").c_str(), "/usd/test");

  EXPECT_STREQ(MockAsyncMotionProfileController::makeFilePath("/usd/subdir", "test").c_str(),
               "/usd/subdir/test");
  EXPECT_STREQ(MockAsyncMotionProfileController::makeFilePath("usd/subdir", "test").c_str(),
               "/usd/subdir/test");
  EXPECT_STREQ(MockAsyncMotionProfileController::makeFilePath("/usd/subdir/", "test").c_str(),
               "/usd/subdir/test");
  EXPECT_STREQ(MockAsyncMotionProfileController::makeFilePath("usd/subdir/", "test").c_str(),
               "/usd/subdir/test");
  EXPECT_STREQ(MockAsyncMotionProfileController::makeFilePath("subdir", "test").c_str(),
               "/usd/subdir/test");
  EXPECT_STREQ(MockAsyncMotionProfileController::makeFilePath("subdir/", "test").c_str(),
               "/usd/subdir/test");
  EXPECT_STREQ(MockAsyncMotionProfileController::makeFilePath("/subdir/", "test").c_str(),
               "/usd/subdir/test");
}

TEST_F(AsyncMotionProfileControllerTest, FilePathRestrict) {
  EXPECT_STREQ(MockAsyncMotionProfileController::makeFilePath("", "t>e<s\"t\\F:i*l|e/").c_str(),
               "/usd/testFile");
}

TEST_F(AsyncMotionProfileControllerTest, SaveLoadPath) {
  controller->generatePath(
    {PathfinderPoint{0_in, 0_in, 0_deg}, PathfinderPoint{3_ft, 0_in, 45_deg}}, "A");
  controller->internalStorePath(leftPathFile, rightPathFile, "A");

  int genPathLen = controller->getPathData("A").length;

  controller->removePath("A");
  controller->internalLoadPath(leftPathFile, rightPathFile, "A");
  EXPECT_EQ(controller->getPaths().front(), "A");
  EXPECT_EQ(controller->getPaths().size(), 1);
  EXPECT_EQ(controller->getPathData("A").length, genPathLen);

  controller->setTarget("A");
  EXPECT_EQ(controller->getTarget(), "A");
}
