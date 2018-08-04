/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/control/async/asyncMotionProfileController.hpp"
#include "okapi/api/control/async/asyncPosIntegratedController.hpp"
#include "okapi/api/control/async/asyncPosPidController.hpp"
#include "okapi/api/control/async/asyncVelIntegratedController.hpp"
#include "okapi/api/control/async/asyncVelPidController.hpp"
#include "okapi/api/control/iterative/iterativeMotorVelocityController.hpp"
#include "okapi/api/control/iterative/iterativePosPidController.hpp"
#include "okapi/api/control/iterative/iterativeVelPidController.hpp"
#include "okapi/api/control/util/flywheelSimulator.hpp"
#include "okapi/api/control/util/pidTuner.hpp"
#include "okapi/api/filter/averageFilter.hpp"
#include "okapi/api/filter/filteredControllerInput.hpp"
#include "okapi/api/filter/passthroughFilter.hpp"
#include "okapi/api/filter/velMath.hpp"
#include "okapi/api/util/mathUtil.hpp"
#include "test/testRunner.hpp"
#include "test/tests/api/implMocks.hpp"
#include <gtest/gtest.h>
#include <limits>

using namespace okapi;
using namespace snowhouse;

class IterativeControllerTest : public ::testing::Test {
  protected:
  virtual void SetUp() {
    sim.setExternalTorqueFunction([](double, double, double) { return 0; });
  }

  void runSimulation(IterativeController<double, double> &controller, const double target) {
    controller.setTarget(target);
    for (size_t i = 0; i < 2000; i++) {
      controller.step(sim.getAngle() * radianToDegree);
      sim.step(controller.getOutput() * sim.getMaxTorque());
    }
  }

  FlywheelSimulator sim;
};

TEST_F(IterativeControllerTest, IterativePosPIDControllerTest) {
  IterativePosPIDController controller(
    0.004, 0, 0, 0, createTimeUtil(Supplier<std::unique_ptr<AbstractTimer>>([]() {
      return std::make_unique<ConstantMockTimer>(10_ms);
    })));

  runSimulation(controller, 45);

  EXPECT_NE(sim.getAngle(), 0);
  EXPECT_NE(controller.getError(), 0);
}

TEST_F(IterativeControllerTest, IterativeVelPIDController) {
  IterativeVelPIDController controller(
    0.000015, 0, 0,
    std::make_unique<VelMath>(1800, std::make_shared<PassthroughFilter>(),
                              std::make_unique<ConstantMockTimer>(10_ms)),
    createTimeUtil(Supplier<std::unique_ptr<AbstractTimer>>(
      []() { return std::make_unique<ConstantMockTimer>(10_ms); })));

  runSimulation(controller, 10);

  EXPECT_NE(sim.getAngle(), 0);
  EXPECT_NE(controller.getError(), 0);
}

TEST_F(IterativeControllerTest, IterativeVelPIDControllerFeedForwardOnly) {
  IterativeVelPIDController controller(
    0, 0, 0.1,
    std::make_unique<VelMath>(1800, std::make_shared<PassthroughFilter>(),
                              std::make_unique<ConstantMockTimer>(10_ms)),
    createTimeUtil(Supplier<std::unique_ptr<AbstractTimer>>(
      []() { return std::make_unique<ConstantMockTimer>(10_ms); })));

  controller.setTarget(5);

  for (int i = 0; i < 5; i++) {
    EXPECT_NEAR(controller.step(0), 0.5, 0.01);
  }
}

TEST_F(IterativeControllerTest, IterativeMotorVelocityController) {
  class MockIterativeVelPIDController : public IterativeVelPIDController {
    public:
    MockIterativeVelPIDController()
      : IterativeVelPIDController(
          0, 0, 0,
          std::make_unique<VelMath>(imev5TPR, std::make_shared<AverageFilter<2>>(),
                                    std::make_unique<ConstantMockTimer>(10_ms)),
          createTimeUtil(Supplier<std::unique_ptr<AbstractTimer>>(
            []() { return std::make_unique<ConstantMockTimer>(10_ms); }))) {
    }

    double step(const double inewReading) override {
      return inewReading;
    }
  };

  auto motor = std::make_shared<MockMotor>();

  IterativeMotorVelocityController controller(motor,
                                              std::make_shared<MockIterativeVelPIDController>());

  controller.step(0);
  EXPECT_NEAR(motor->lastVelocity, 0, 0.01);

  controller.step(0.5);
  EXPECT_NEAR(motor->lastVelocity, 63, 0.01);

  controller.step(1);
  EXPECT_NEAR(motor->lastVelocity, 127, 0.01);

  controller.step(-0.5);
  EXPECT_NEAR(motor->lastVelocity, -63, 0.01);
}

class AsyncControllerTest : public ::testing::Test {
  public:
  void assertControllerFollowsDisableLifecycle(AsyncController<double, double> &&controller,
                                               std::int16_t &domainValue,
                                               std::int16_t &voltageValue) {
    EXPECT_FALSE(controller.isDisabled()) << "Should not be disabled at the start.";

    controller.setTarget(100);
    EXPECT_EQ(domainValue, 100) << "Should be on by default.";

    controller.flipDisable();
    EXPECT_TRUE(controller.isDisabled()) << "Should be disabled after flipDisable";
    EXPECT_EQ(voltageValue, 0) << "Disabling the controller should turn the motor off";

    controller.flipDisable();
    EXPECT_FALSE(controller.isDisabled()) << "Should not be disabled after flipDisable";
    EXPECT_EQ(domainValue, 100)
      << "Re-enabling the controller should move the motor to the previous target";

    controller.flipDisable();
    EXPECT_TRUE(controller.isDisabled()) << "Should be disabled after flipDisable";
    controller.reset();
    EXPECT_TRUE(controller.isDisabled()) << "Should be disabled after reset";
    EXPECT_EQ(voltageValue, 0) << "Resetting the controller should not change the current target";

    controller.flipDisable();
    EXPECT_FALSE(controller.isDisabled()) << "Should not be disabled after flipDisable";
    domainValue = 1337;            // Sample value to check it doesn't change
    MockRate().delayUntil(100_ms); // Wait for it to possibly change
    EXPECT_EQ(domainValue, 1337)
      << "Re-enabling the controller after a reset should not move the motor";
  }

  void assertControllerFollowsTargetLifecycle(AsyncController<double, double> &&controller) {
    EXPECT_DOUBLE_EQ(0, controller.getError()) << "Should start with 0 error";
    controller.setTarget(100);
    EXPECT_DOUBLE_EQ(100, controller.getError());
    controller.setTarget(0);
    EXPECT_DOUBLE_EQ(0, controller.getError());
  }
};

TEST_F(AsyncControllerTest, AsyncPosIntegratedController) {
  auto motor = std::make_shared<MockMotor>();
  assertControllerFollowsDisableLifecycle(AsyncPosIntegratedController(motor, createTimeUtil()),
                                          motor->lastPosition, motor->lastVoltage);
  assertControllerFollowsTargetLifecycle(AsyncPosIntegratedController(motor, createTimeUtil()));
}

TEST_F(AsyncControllerTest, AsyncVelIntegratedController) {
  auto motor = std::make_shared<MockMotor>();
  assertControllerFollowsDisableLifecycle(AsyncVelIntegratedController(motor, createTimeUtil()),
                                          motor->lastVelocity, motor->lastVoltage);
  assertControllerFollowsTargetLifecycle(AsyncVelIntegratedController(motor, createTimeUtil()));
}

void assertControllerIsSettledWhenDisabled(ClosedLoopController<double, double> &controller) {
  controller.setTarget(100);
  EXPECT_FALSE(controller.isSettled());

  controller.flipDisable();
  EXPECT_TRUE(controller.isSettled());
}

TEST(IterativePosPIDControllerTest, SettledWhenDisabled) {
  IterativePosPIDController controller(
    0.004, 0, 0, 0, createTimeUtil(Supplier<std::unique_ptr<AbstractTimer>>([]() {
      return std::make_unique<ConstantMockTimer>(10_ms);
    })));

  assertControllerIsSettledWhenDisabled(controller);
}

TEST(IterativeVelPIDControllerTest, SettledWhenDisabled) {
  IterativeVelPIDController controller(
    0, 0, 0.1,
    std::make_unique<VelMath>(1800, std::make_shared<PassthroughFilter>(),
                              std::make_unique<ConstantMockTimer>(10_ms)),
    createTimeUtil(Supplier<std::unique_ptr<AbstractTimer>>(
      []() { return std::make_unique<ConstantMockTimer>(10_ms); })));

  assertControllerIsSettledWhenDisabled(controller);
}

TEST(AsyncPosIntegratedControllerTest, SettledWhenDisabled) {
  AsyncPosIntegratedController controller(std::make_shared<MockMotor>(), createTimeUtil());

  assertControllerIsSettledWhenDisabled(controller);
}

TEST(AsyncVelIntegratedControllerTest, SettledWhenDisabled) {
  AsyncVelIntegratedController controller(std::make_shared<MockMotor>(), createTimeUtil());

  assertControllerIsSettledWhenDisabled(controller);
}

TEST(FilteredControllerInputTest, InputShouldBePassedThrough) {
  class MockControllerInput : public ControllerInput<double> {
    public:
    double controllerGet() override {
      return 1;
    }
  };

  std::unique_ptr<ControllerInput<double>> mockInput = std::make_unique<MockControllerInput>();
  PassthroughFilter filter;
  FilteredControllerInput<double, PassthroughFilter> input(std::move(mockInput), filter);

  for (int i = 0; i < 3; i++) {
    EXPECT_FLOAT_EQ(input.controllerGet(), 1);
  }
}

TEST(PIDTunerTest, ConstructorShouldNotSegfault) {
  auto output = std::make_shared<MockMotor>();
  auto input = output->getEncoder();
  PIDTuner pidTuner(input, output, createTimeUtil(), 1_s, 100, 0, 10, 0, 10, 0, 10);
}

TEST(PIDTunerTest, AutotuneShouldNotSegfault) {
  FlywheelSimulator simulator;
  simulator.setExternalTorqueFunction([](double, double, double) { return 0; });

  auto system = std::make_shared<SimulatedSystem>(simulator);

  PIDTuner pidTuner(system, system, createTimeUtil(), 100_ms, 100, 0, 10, 0, 10, 0, 10);
  auto result = pidTuner.autotune();

  system->join(); // gtest will cause a SIGABRT if we don't join manually first
}

TEST(SettledUtilTest, MaxDoubleError) {
  MockRate rate;
  SettledUtil settledUtil(std::make_unique<MockTimer>(), std::numeric_limits<double>::max(), 5,
                          250_ms);
  EXPECT_FALSE(settledUtil.isSettled(1000));
  EXPECT_FALSE(settledUtil.isSettled(1000));
  rate.delayUntil(300_ms);
  EXPECT_TRUE(settledUtil.isSettled(1000));
}

TEST(SettledUtilTest, MaxDoubleDerivative) {
  MockRate rate;
  SettledUtil settledUtil(std::make_unique<MockTimer>(), 50, std::numeric_limits<double>::max(),
                          250_ms);
  EXPECT_FALSE(settledUtil.isSettled(1000));
  EXPECT_FALSE(settledUtil.isSettled(0));
  rate.delayUntil(300_ms);
  EXPECT_TRUE(settledUtil.isSettled(0));
}

TEST(SettledUtilTest, ZeroTime) {
  MockRate rate;
  SettledUtil settledUtil(std::make_unique<MockTimer>(), 50, 5, 0_ms);
  EXPECT_FALSE(settledUtil.isSettled(60));
  EXPECT_FALSE(settledUtil.isSettled(55));
  EXPECT_TRUE(settledUtil.isSettled(50));
}

class AsyncMotionProfileControllerTest : public ::testing::Test {
  protected:
  virtual void SetUp() {
    Logger::initialize(std::make_unique<MockTimer>(), "Log_AsyncMotionProfileControllerTest.txt",
                       Logger::LogLevel::debug);

    leftMotor = new MockMotor();
    rightMotor = new MockMotor();

    model = new SkidSteerModel(std::unique_ptr<AbstractMotor>(leftMotor),
                               std::unique_ptr<AbstractMotor>(rightMotor));

    controller = new AsyncMotionProfileController(createTimeUtil(), 1.0, 2.0, 10.0,
                                                  std::shared_ptr<SkidSteerModel>(model), 10.5_in);
  }

  virtual void TearDown() {
    free(controller);
  }

  MockMotor *leftMotor;
  MockMotor *rightMotor;
  SkidSteerModel *model;
  AsyncMotionProfileController *controller;
};

TEST_F(AsyncMotionProfileControllerTest, BasicTest) {
  controller->generatePath({AsyncMotionProfileController::Point{0_m, 0_m, 0_deg},
                            AsyncMotionProfileController::Point{3_ft, 0_m, 45_deg}},
                           "A");

  controller->executePath("A");
}
