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
#include "test/tests/api/implMocks.hpp"
#include <gtest/gtest.h>
#include <limits>

using namespace okapi;

class IterativeControllerWithSimulatorTest : public ::testing::Test {
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

TEST_F(IterativeControllerWithSimulatorTest, IterativePosPIDControllerTest) {
  IterativePosPIDController controller(
    0.004, 0, 0, 0, createTimeUtil(Supplier<std::unique_ptr<AbstractTimer>>([]() {
      return std::make_unique<ConstantMockTimer>(10_ms);
    })));

  runSimulation(controller, 45);

  EXPECT_NE(sim.getAngle(), 0);
  EXPECT_NE(controller.getError(), 0);
}

TEST_F(IterativeControllerWithSimulatorTest, IterativeVelPIDController) {
  IterativeVelPIDController controller(
    0.000015,
    0,
    0,
    0,
    std::make_unique<VelMath>(1800,
                              std::make_shared<PassthroughFilter>(),
                              0_ms,
                              std::make_unique<ConstantMockTimer>(10_ms)),
    createTimeUtil(Supplier<std::unique_ptr<AbstractTimer>>(
      []() { return std::make_unique<ConstantMockTimer>(10_ms); })));

  runSimulation(controller, 10);

  EXPECT_NE(sim.getAngle(), 0);
  EXPECT_NE(controller.getError(), 0);
}

TEST_F(IterativeControllerWithSimulatorTest, IterativeVelPIDControllerFeedForwardOnly) {
  IterativeVelPIDController controller(
    0,
    0,
    0.1,
    0,
    std::make_unique<VelMath>(1800,
                              std::make_shared<PassthroughFilter>(),
                              0_ms,
                              std::make_unique<ConstantMockTimer>(10_ms)),
    createTimeUtil(Supplier<std::unique_ptr<AbstractTimer>>(
      []() { return std::make_unique<ConstantMockTimer>(10_ms); })));

  controller.setTarget(5);

  for (int i = 0; i < 5; i++) {
    EXPECT_NEAR(controller.step(0), 0.5, 0.01);
  }
}

TEST(FilteredControllerInputTest, InputShouldBePassedThrough) {
  class MockControllerInput : public ControllerInput<double> {
    public:
    double controllerGet() override {
      return 1;
    }
  };

  FilteredControllerInput<double, PassthroughFilter> input(std::make_unique<MockControllerInput>(),
                                                           std::make_unique<PassthroughFilter>());

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
  system->startThread();

  PIDTuner pidTuner(system, system, createTimeUtil(), 100_ms, 100, 0, 10, 0, 10, 0, 10);
  pidTuner.autotune();

  system->join(); // gtest will cause a SIGABRT if we don't join manually first
}

TEST(SettledUtilTest, MaxDoubleError) {
  MockRate rate;
  SettledUtil settledUtil(
    std::make_unique<MockTimer>(), std::numeric_limits<double>::max(), 5, 250_ms);
  EXPECT_FALSE(settledUtil.isSettled(1000));
  EXPECT_FALSE(settledUtil.isSettled(1000));
  rate.delayUntil(300_ms);
  EXPECT_TRUE(settledUtil.isSettled(1000));
}

TEST(SettledUtilTest, MaxDoubleDerivative) {
  MockRate rate;
  SettledUtil settledUtil(
    std::make_unique<MockTimer>(), 50, std::numeric_limits<double>::max(), 250_ms);
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
