#include "test/tests/api/controllerTests.hpp"
#include "okapi/api/control/async/asyncPosIntegratedController.hpp"
#include "okapi/api/control/async/asyncVelIntegratedController.hpp"
#include "okapi/api/control/iterative/iterativeMotorVelocityController.hpp"
#include "okapi/api/control/iterative/iterativePosPidController.hpp"
#include "okapi/api/control/iterative/iterativeVelPidController.hpp"
#include "okapi/api/control/util/flywheelSimulator.hpp"
#include "okapi/api/filter/averageFilter.hpp"
#include "okapi/api/filter/filteredControllerInput.hpp"
#include "okapi/api/filter/passthroughFilter.hpp"
#include "okapi/api/filter/velMath.hpp"
#include "okapi/api/util/mathUtil.hpp"
#include "test/testRunner.hpp"
#include "test/tests/api/implMocks.hpp"
#include <gtest/gtest.h>

using namespace okapi;
using namespace snowhouse;

class IterativeControllerTest : public ::testing::Test {
  protected:
  virtual void SetUp() {
    sim.setExternalTorqueFunction([](double, double, double) { return 0; });
  }

  void runSimulation(IterativeController &controller, const double target) {
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
  void assertControllerFollowsDisableLifecycle(AsyncController &controller,
                                               std::int16_t &domainValue,
                                               std::int16_t &voltageValue) {
    controller.setTarget(100);
    EXPECT_EQ(domainValue, 100) << "Should be on by default.";

    controller.flipDisable();
    EXPECT_EQ(voltageValue, 0) << "Disabling the controller should turn the motor off";

    controller.flipDisable();
    EXPECT_EQ(domainValue, 100)
      << "Re-enabling the controller should move the motor to the previous target";

    controller.flipDisable();
    controller.reset();
    EXPECT_EQ(voltageValue, 0) << "Resetting the controller should not change the current target";

    controller.flipDisable();
    domainValue = 1337;            // Sample value to check it doesn't change
    MockRate().delayUntil(100_ms); // Wait for it to possibly change
    EXPECT_EQ(domainValue, 1337)
      << "Re-enabling the controller after a reset should not move the motor";
  }
};

TEST_F(AsyncControllerTest, AsyncPosIntegratedController) {
  auto motor = std::make_shared<MockMotor>();
  AsyncPosIntegratedController controller(motor, createTimeUtil());
  assertControllerFollowsDisableLifecycle(controller, motor->lastPosition, motor->lastVoltage);
}

TEST_F(AsyncControllerTest, AsyncVelIntegratedController) {
  auto motor = std::make_shared<MockMotor>();
  AsyncVelIntegratedController controller(motor, createTimeUtil());
  assertControllerFollowsDisableLifecycle(controller, motor->lastVelocity, motor->lastVoltage);
}

TEST(FilteredControllerInputTest, InputShouldBePassedThrough) {
  class MockControllerInput : public ControllerInput {
    public:
    double controllerGet() override {
      return 1;
    }
  };

  MockControllerInput mockInput;
  PassthroughFilter filter;
  FilteredControllerInput<MockControllerInput, PassthroughFilter> input(mockInput, filter);

  for (int i = 0; i < 3; i++) {
    EXPECT_FLOAT_EQ(input.controllerGet(), 1);
  }
}
