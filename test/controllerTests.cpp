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

void testIterativeControllers() {
  using namespace okapi;
  using namespace snowhouse;

  {
    test_printf("Testing IterativePosPIDController");

    FlywheelSimulator sim;
    sim.setExternalTorqueFunction([](double, double, double) { return 0; });

    IterativePosPIDController controller(0.004, 0, 0, 0, std::make_unique<ConstantMockTimer>(10_ms),
                                         createSettledUtilPtr());

    const double target = 45;
    controller.setTarget(target);
    for (size_t i = 0; i < 2000; i++) {
      controller.step(sim.getAngle() * radianToDegree);
      sim.step(controller.getOutput() * sim.getMaxTorque());
    }

    test("IterativePosPIDController should have moved", [&]() {
      AssertThat(sim.getAngle(), Is().Not().EqualTo(0));
      AssertThat(controller.getError(), Is().Not().EqualTo(0));
    });
  }

  {
    test_printf("Testing IterativeVelPIDController");

    FlywheelSimulator sim;
    sim.setExternalTorqueFunction([](double, double, double) { return 0; });

    IterativeVelPIDController controller(
      0.000015, 0, 0,
      std::make_unique<VelMath>(1800, std::make_shared<PassthroughFilter>(),
                                std::make_unique<ConstantMockTimer>(10_ms)),
      std::make_unique<ConstantMockTimer>(10_ms), createSettledUtilPtr());

    const double target = 10;
    controller.setTarget(target);
    for (size_t i = 0; i < 2000; i++) {
      controller.step(sim.getAngle() * radianToDegree);
      sim.step(controller.getOutput() * sim.getMaxTorque());
    }

    test("IterativeVelPIDController should have moved", [&]() {
      AssertThat(sim.getOmega(), Is().Not().EqualTo(0));
      AssertThat(controller.getError(), Is().Not().EqualTo(0));
    });

    IterativeVelPIDController controller2(
      0, 0, 0.1,
      std::make_unique<VelMath>(1800, std::make_shared<PassthroughFilter>(),
                                std::make_unique<ConstantMockTimer>(10_ms)),
      std::make_unique<ConstantMockTimer>(10_ms), createSettledUtilPtr());

    controller2.setTarget(5);
    for (size_t i = 0; i < 5; i++) {
      test(
        "IterativeVelPIDController with feed-forward should output proportional to error and not "
        "integrate " +
          std::to_string(i),
        TEST_BODY(AssertThat, controller2.step(0), EqualsWithDelta(0.5, 0.01)));
    }
  }

  {
    test_printf("Testing IterativeMotorVelocityController");

    class MockIterativeVelPIDController : public IterativeVelPIDController {
      public:
      MockIterativeVelPIDController()
        : IterativeVelPIDController(
            0, 0, 0,
            std::make_unique<VelMath>(imev5TPR, std::make_shared<AverageFilter<2>>(),
                                      std::make_unique<ConstantMockTimer>(10_ms)),
            std::make_unique<ConstantMockTimer>(10_ms), createSettledUtilPtr()) {
      }

      double step(const double inewReading) override {
        return inewReading;
      }
    };

    auto motor = std::make_shared<MockMotor>();

    IterativeMotorVelocityController controller(motor,
                                                std::make_shared<MockIterativeVelPIDController>());

    controller.step(0);
    test("IterativeMotorVelocityController should set the motor velocity to the controller output "
         "* 127 1",
         TEST_BODY(AssertThat, motor->lastVelocity, EqualsWithDelta(0 * 127, 0.01)));

    controller.step(0.5);
    test("IterativeMotorVelocityController should set the motor velocity to the controller output "
         "* 127 2",
         TEST_BODY(AssertThat, motor->lastVelocity, EqualsWithDelta(63, 0.01)));

    controller.step(1);
    test("IterativeMotorVelocityController should set the motor velocity to the controller output "
         "* 127 3",
         TEST_BODY(AssertThat, motor->lastVelocity, EqualsWithDelta(1 * 127, 0.01)));

    controller.step(-0.5);
    test("IterativeMotorVelocityController should set the motor velocity to the controller output "
         "* 127 4",
         TEST_BODY(AssertThat, motor->lastVelocity, EqualsWithDelta(-63, 0.01)));
  }
}

void testAsyncControllers() {
  using namespace okapi;
  using namespace snowhouse;

  {
    test_printf("Testing AsyncPosIntegratedController");

    auto motor = std::make_shared<MockMotor>();

    AsyncPosIntegratedController controller(motor, createSettledUtilPtr(),
                                            std::make_unique<MockRate>());

    controller.setTarget(100);
    test("Should be on by default", TEST_BODY(AssertThat, motor->lastPosition, Equals(100)));

    controller.flipDisable();
    test("Disabling the controller should turn the motor off",
         TEST_BODY(AssertThat, motor->lastVoltage, Equals(0)));

    controller.flipDisable();
    test("Re-enabling the controller should move the motor to the previous target",
         TEST_BODY(AssertThat, motor->lastPosition, Equals(100)));

    controller.flipDisable();
    controller.reset();
    test("Resetting the controller should not change the current target",
         TEST_BODY(AssertThat, motor->lastVoltage, Equals(0)));
    controller.flipDisable();
    motor->lastPosition = 1337; // Sample value to check it doesn't change
    test("Re-enabling the controller after a reset should not move the motor",
         TEST_BODY(AssertThat, motor->lastPosition, Equals(1337)));
  }

  {
    test_printf("Testing AsyncVelIntegratedController");

    auto motor = std::make_shared<MockMotor>();

    AsyncVelIntegratedController controller(motor, createSettledUtilPtr(),
                                            std::make_unique<MockRate>());

    controller.setTarget(100);
    test("Should be on by default", TEST_BODY(AssertThat, motor->lastVelocity, Equals(100)));

    controller.flipDisable();
    test("Disabling the controller should turn the motor off",
         TEST_BODY(AssertThat, motor->lastVoltage, Equals(0)));

    controller.flipDisable();
    test("Re-enabling the controller should move the motor to the previous target",
         TEST_BODY(AssertThat, motor->lastVelocity, Equals(100)));

    controller.flipDisable();
    controller.reset();
    test("Resetting the controller should not change the current target",
         TEST_BODY(AssertThat, motor->lastVoltage, Equals(0)));
    controller.flipDisable();
    motor->lastVelocity = 1337; // Sample value to check it doesn't change
    test("Re-enabling the controller after a reset should not move the motor",
         TEST_BODY(AssertThat, motor->lastVelocity, Equals(1337)));
  }
}

void testFilteredControllerInput() {
  using namespace okapi;
  using namespace snowhouse;

  {
    test_printf("Testing FilteredControllerInput");

    class MockControllerInput : public ControllerInput {
      public:
      double controllerGet() override {
        return 1;
      }
    };

    MockControllerInput mockInput;
    PassthroughFilter filter;
    FilteredControllerInput<MockControllerInput, PassthroughFilter> input(mockInput, filter);

    for (int i = 0; i < 10; i++) {
      auto testName = "FilteredControllerInput i = " + std::to_string(i);
      test(testName, TEST_BODY(AssertThat, input.controllerGet(), EqualsWithDelta(1, 0.0001)));
    }
  }
}

void testControllers() {
  testIterativeControllers();
  testAsyncControllers();
  testFilteredControllerInput();
}
