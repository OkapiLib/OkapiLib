#ifndef PROS_VERSION_MAJOR

#include "okapi/api/chassis/controller/chassisScales.hpp"
#include "okapi/api/chassis/model/skidSteerModel.hpp"
#include "okapi/api/chassis/model/xDriveModel.hpp"
#include "okapi/api/control/util/flywheelSimulator.hpp"
#include "okapi/api/device/button/buttonBase.hpp"
#include "okapi/api/device/motor/abstractMotor.hpp"
#include "okapi/api/filter/averageFilter.hpp"
#include "okapi/api/filter/composableFilter.hpp"
#include "okapi/api/filter/demaFilter.hpp"
#include "okapi/api/filter/ekfFilter.hpp"
#include "okapi/api/filter/emaFilter.hpp"
#include "okapi/api/filter/medianFilter.hpp"
#include "okapi/api/filter/passthroughFilter.hpp"
#include "okapi/api/filter/velMath.hpp"
#include "test/crossPlatformTestRunner.hpp"
#include <memory>

using namespace okapi;
using namespace snowhouse;
using namespace fakeit;

void testChassisScales() {
  using namespace okapi;
  using namespace snowhouse;

  test_printf("Testing ChassisScales");

  {
    ChassisScales scales({0.5, 0.3});
    test("ChassisScales should accept raw scales", [&]() {
      AssertThat(scales.straight, Equals(0.5));
      AssertThat(scales.turn, Equals(0.3));
    });
  }

  {
    ChassisScales scales({4_in, 11.5_in});
    test("ChassisScales should calculate scales from wheelbase", [&]() {
      AssertThat(scales.straight, EqualsWithDelta(1127.86968, 0.0001));
      AssertThat(scales.turn, EqualsWithDelta(2.875, 0.001));
    });
  }
}

void testSkidSteerModel() {
  using namespace okapi;
  using namespace snowhouse;

  test_printf("Testing SkidSteerModel");

  class MockMotor : public AbstractMotor {
    public:
    void controllerSet(const double ivalue) override {
    }

    int32_t moveAbsolute(const double iposition, const std::int32_t ivelocity) const override {
      return 0;
    }

    int32_t moveRelative(const double iposition, const std::int32_t ivelocity) const override {
      return 0;
    }

    double getTargetPosition() const override {
      return 0;
    }

    double getPosition() const override {
      return 0;
    }

    int32_t getTargetVelocity() const override {
      return 0;
    }

    double getActualVelocity() const override {
      return 0;
    }

    int32_t tarePosition() const override {
      return 0;
    }

    int32_t setBrakeMode(const brakeMode imode) const override {
      return 0;
    }

    int32_t setCurrentLimit(const std::int32_t ilimit) const override {
      return 0;
    }

    int32_t setEncoderUnits(const encoderUnits iunits) const override {
      return 0;
    }

    int32_t setGearing(const gearset igearset) const override {
      return 0;
    }

    int32_t setReversed(const bool ireverse) const override {
      return 0;
    }

    int32_t setVoltageLimit(const std::int32_t ilimit) const override {
      return 0;
    }

    std::shared_ptr<ContinuousRotarySensor> getEncoder() const override {
      return std::shared_ptr<ContinuousRotarySensor>();
    }

    std::int32_t moveVelocity(const std::int16_t ivelocity) const override {
      lastVelocity = ivelocity;
      return 1;
    }

    std::int32_t moveVoltage(const std::int16_t ivoltage) const override {
      lastVoltage = ivoltage;
      return 1;
    }

    mutable std::int16_t lastVelocity{};
    mutable std::int16_t lastVoltage{};
  };

  auto leftMotor = std::make_shared<MockMotor>();
  auto rightMotor = std::make_shared<MockMotor>();
  SkidSteerModel model(leftMotor, rightMotor, 127);

  model.forward(0.5);
  test("SkidSteerModel forward should power all motors forward", [&]() {
    AssertThat(leftMotor->lastVelocity, Equals(63));
    AssertThat(rightMotor->lastVelocity, Equals(63));
  });

  model.forward(10);
  test("SkidSteerModel forward should bound its input", [&]() {
    AssertThat(leftMotor->lastVelocity, Equals(127));
    AssertThat(rightMotor->lastVelocity, Equals(127));
  });

  model.rotate(0.5);
  test("SkidSteerModel rotate should power left motors positive and right motors negative", [&]() {
    AssertThat(leftMotor->lastVelocity, Equals(63));
    AssertThat(rightMotor->lastVelocity, Equals(-63));
  });

  model.rotate(10);
  test("SkidSteerModel rotate should bound its input", [&]() {
    AssertThat(leftMotor->lastVelocity, Equals(127));
    AssertThat(rightMotor->lastVelocity, Equals(-127));
  });

  model.driveVector(0.25, 0.25);
  test("SkidSteerModel driveVector should make a swing turn", [&]() {
    AssertThat(leftMotor->lastVelocity, Equals(63));
    AssertThat(rightMotor->lastVelocity, Equals(0));
  });

  model.driveVector(0.9, 0.25);
  test("SkidSteerModel driveVector should make a bounded swing turn", [&]() {
    AssertThat(leftMotor->lastVelocity, Equals(127));
    AssertThat(rightMotor->lastVelocity, Equals(71));
  });

  leftMotor->lastVelocity = 100;
  rightMotor->lastVelocity = 100;
  model.stop();
  test("SkidSteerModel stop should set the motors to 0", [&]() {
    AssertThat(leftMotor->lastVelocity, Equals(0));
    AssertThat(rightMotor->lastVelocity, Equals(0));
  });

  model.left(0.5);
  test("SkidSteerModel left should set the left motors",
       [&]() { AssertThat(leftMotor->lastVelocity, Equals(63)); });

  model.right(0.5);
  test("SkidSteerModel right should set the right motors",
       [&]() { AssertThat(rightMotor->lastVelocity, Equals(63)); });

  model.tank(0.5, 0.5);
  test("SkidSteerModel tank should set the left and right voltages", [&]() {
    AssertThat(leftMotor->lastVoltage, Equals(63));
    AssertThat(rightMotor->lastVoltage, Equals(63));
  });

  model.tank(10, 10);
  test("SkidSteerModel tank should bound its inputs", [&]() {
    AssertThat(leftMotor->lastVoltage, Equals(127));
    AssertThat(rightMotor->lastVoltage, Equals(127));
  });

  model.tank(0.2, 0.2, 0.5);
  test("SkidSteerModel tank should apply threshold", [&]() {
    AssertThat(leftMotor->lastVoltage, Equals(0));
    AssertThat(rightMotor->lastVoltage, Equals(0));
  });

  model.arcade(0.5, 0);
  test("SkidSteerModel arcade should move the robot forward", [&]() {
    AssertThat(leftMotor->lastVoltage, Equals(63));
    AssertThat(rightMotor->lastVoltage, Equals(63));
  });

  model.arcade(0, 0.5);
  test("SkidSteerModel arcade should turn the robot", [&]() {
    AssertThat(leftMotor->lastVoltage, Equals(63));
    AssertThat(rightMotor->lastVoltage, Equals(-63));
  });

  model.arcade(10, 0);
  test("SkidSteerModel arcade should bound its inputs", [&]() {
    AssertThat(leftMotor->lastVoltage, Equals(127));
    AssertThat(rightMotor->lastVoltage, Equals(127));
  });

  model.arcade(0.2, 0, 0.5);
  test("SkidSteerModel arcade should apply threshold", [&]() {
    AssertThat(leftMotor->lastVoltage, Equals(0));
    AssertThat(rightMotor->lastVoltage, Equals(0));
  });
}

void testXDriveModel() {
  using namespace okapi;
  using namespace snowhouse;

  test_printf("Testing XDriveModel");

  class MockMotor : public AbstractMotor {
    public:
    void controllerSet(const double ivalue) override {
    }

    int32_t moveAbsolute(const double iposition, const std::int32_t ivelocity) const override {
      return 0;
    }

    int32_t moveRelative(const double iposition, const std::int32_t ivelocity) const override {
      return 0;
    }

    double getTargetPosition() const override {
      return 0;
    }

    double getPosition() const override {
      return 0;
    }

    int32_t getTargetVelocity() const override {
      return 0;
    }

    double getActualVelocity() const override {
      return 0;
    }

    int32_t tarePosition() const override {
      return 0;
    }

    int32_t setBrakeMode(const brakeMode imode) const override {
      return 0;
    }

    int32_t setCurrentLimit(const std::int32_t ilimit) const override {
      return 0;
    }

    int32_t setEncoderUnits(const encoderUnits iunits) const override {
      return 0;
    }

    int32_t setGearing(const gearset igearset) const override {
      return 0;
    }

    int32_t setReversed(const bool ireverse) const override {
      return 0;
    }

    int32_t setVoltageLimit(const std::int32_t ilimit) const override {
      return 0;
    }

    std::shared_ptr<ContinuousRotarySensor> getEncoder() const override {
      return std::shared_ptr<ContinuousRotarySensor>();
    }

    std::int32_t moveVelocity(const std::int16_t ivelocity) const override {
      lastVelocity = ivelocity;
      return 1;
    }

    std::int32_t moveVoltage(const std::int16_t ivoltage) const override {
      lastVoltage = ivoltage;
      return 1;
    }

    mutable std::int16_t lastVelocity{};
    mutable std::int16_t lastVoltage{};
  };

  auto topLeftMotor = std::make_shared<MockMotor>();
  auto topRightMotor = std::make_shared<MockMotor>();
  auto bottomRightMotor = std::make_shared<MockMotor>();
  auto bottomLeftMotor = std::make_shared<MockMotor>();
  XDriveModel model(topLeftMotor, topRightMotor, bottomRightMotor, bottomLeftMotor, 127);

  model.forward(0.5);
  test("XDriveModel forward should power all motors forward", [&]() {
    AssertThat(topLeftMotor->lastVelocity, Equals(63));
    AssertThat(topRightMotor->lastVelocity, Equals(63));
    AssertThat(bottomRightMotor->lastVelocity, Equals(63));
    AssertThat(bottomLeftMotor->lastVelocity, Equals(63));
  });

  model.forward(10);
  test("XDriveModel forward should bound its input", [&]() {
    AssertThat(topLeftMotor->lastVelocity, Equals(127));
    AssertThat(topRightMotor->lastVelocity, Equals(127));
    AssertThat(bottomRightMotor->lastVelocity, Equals(127));
    AssertThat(bottomLeftMotor->lastVelocity, Equals(127));
  });

  model.rotate(0.5);
  test("XDriveModel rotate should power left motors positive and right motors negative", [&]() {
    AssertThat(topLeftMotor->lastVelocity, Equals(63));
    AssertThat(topRightMotor->lastVelocity, Equals(-63));
    AssertThat(bottomRightMotor->lastVelocity, Equals(-63));
    AssertThat(bottomLeftMotor->lastVelocity, Equals(63));
  });

  model.rotate(10);
  test("XDriveModel rotate should bound its input", [&]() {
    AssertThat(topLeftMotor->lastVelocity, Equals(127));
    AssertThat(topRightMotor->lastVelocity, Equals(-127));
    AssertThat(bottomRightMotor->lastVelocity, Equals(-127));
    AssertThat(bottomLeftMotor->lastVelocity, Equals(127));
  });

  model.driveVector(0.25, 0.25);
  test("XDriveModel driveVector should make a swing turn", [&]() {
    AssertThat(topLeftMotor->lastVelocity, Equals(63));
    AssertThat(topRightMotor->lastVelocity, Equals(0));
    AssertThat(bottomRightMotor->lastVelocity, Equals(0));
    AssertThat(bottomLeftMotor->lastVelocity, Equals(63));
  });

  model.driveVector(0.9, 0.25);
  test("XDriveModel driveVector should make a bounded swing turn", [&]() {
    AssertThat(topLeftMotor->lastVelocity, Equals(127));
    AssertThat(topRightMotor->lastVelocity, Equals(71));
    AssertThat(bottomRightMotor->lastVelocity, Equals(71));
    AssertThat(bottomLeftMotor->lastVelocity, Equals(127));
  });

  topLeftMotor->lastVelocity = 100;
  topRightMotor->lastVelocity = 100;
  bottomRightMotor->lastVelocity = 100;
  bottomLeftMotor->lastVelocity = 100;
  model.stop();
  test("XDriveModel stop should set the motors to 0", [&]() {
    AssertThat(topLeftMotor->lastVelocity, Equals(0));
    AssertThat(topRightMotor->lastVelocity, Equals(0));
    AssertThat(bottomRightMotor->lastVelocity, Equals(0));
    AssertThat(bottomLeftMotor->lastVelocity, Equals(0));
  });

  model.left(0.5);
  test("XDriveModel left should set the left motors", [&]() {
    AssertThat(topLeftMotor->lastVelocity, Equals(63));
    AssertThat(bottomLeftMotor->lastVelocity, Equals(63));
  });

  model.right(0.5);
  test("XDriveModel right should set the right motors", [&]() {
    AssertThat(topRightMotor->lastVelocity, Equals(63));
    AssertThat(bottomRightMotor->lastVelocity, Equals(63));
  });

  model.tank(0.5, 0.5);
  test("XDriveModel tank should set the left and right voltages", [&]() {
    AssertThat(topLeftMotor->lastVelocity, Equals(63));
    AssertThat(topRightMotor->lastVoltage, Equals(63));
    AssertThat(bottomRightMotor->lastVoltage, Equals(63));
    AssertThat(bottomLeftMotor->lastVoltage, Equals(63));
  });

  model.tank(10, 10);
  test("XDriveModel tank should bound its inputs", [&]() {
    AssertThat(topLeftMotor->lastVoltage, Equals(127));
    AssertThat(topRightMotor->lastVoltage, Equals(127));
    AssertThat(bottomRightMotor->lastVoltage, Equals(127));
    AssertThat(bottomLeftMotor->lastVoltage, Equals(127));
  });

  model.tank(0.2, 0.2, 0.5);
  test("XDriveModel tank should apply threshold", [&]() {
    AssertThat(topLeftMotor->lastVoltage, Equals(0));
    AssertThat(topRightMotor->lastVoltage, Equals(0));
    AssertThat(bottomRightMotor->lastVoltage, Equals(0));
    AssertThat(bottomLeftMotor->lastVoltage, Equals(0));
  });

  model.arcade(0.5, 0);
  test("XDriveModel arcade should move the robot forward", [&]() {
    AssertThat(topLeftMotor->lastVoltage, Equals(63));
    AssertThat(topRightMotor->lastVoltage, Equals(63));
    AssertThat(bottomRightMotor->lastVoltage, Equals(63));
    AssertThat(bottomLeftMotor->lastVoltage, Equals(63));
  });

  model.arcade(0, 0.5);
  test("XDriveModel arcade should turn the robot", [&]() {
    AssertThat(topLeftMotor->lastVoltage, Equals(63));
    AssertThat(topRightMotor->lastVoltage, Equals(-63));
    AssertThat(bottomRightMotor->lastVoltage, Equals(-63));
    AssertThat(bottomLeftMotor->lastVoltage, Equals(63));
  });

  model.arcade(10, 0);
  test("XDriveModel arcade should bound its inputs", [&]() {
    AssertThat(topLeftMotor->lastVoltage, Equals(127));
    AssertThat(topRightMotor->lastVoltage, Equals(127));
    AssertThat(bottomRightMotor->lastVoltage, Equals(127));
    AssertThat(bottomLeftMotor->lastVoltage, Equals(127));
  });

  model.arcade(0.2, 0, 0.5);
  test("XDriveModel arcade should apply threshold", [&]() {
    AssertThat(topLeftMotor->lastVoltage, Equals(0));
    AssertThat(topRightMotor->lastVoltage, Equals(0));
    AssertThat(bottomRightMotor->lastVoltage, Equals(0));
    AssertThat(bottomLeftMotor->lastVoltage, Equals(0));
  });
}

void testChassisModels() {
  testSkidSteerModel();
  testXDriveModel();
}

void testControlUtils() {

  {
    test_printf("Testing FlywheelSimulator");

    FlywheelSimulator sim;

    sim.setTorque(0.3);
    sim.step();

    test("FlywheelSimulator i = 0 angle",
         TEST_BODY(AssertThat, sim.getAngle(), EqualsWithDelta(0.000020193, 0.00000005)));
    test("FlywheelSimulator i = 0 omega",
         TEST_BODY(AssertThat, sim.getOmega(), EqualsWithDelta(0.0020193, 0.000005)));
    test("FlywheelSimulator i = 0 accel",
         TEST_BODY(AssertThat, sim.getAcceleration(), EqualsWithDelta(20.193, 0.0005)));
  }
}

void testButtons() {

  class MockButton : public ButtonBase {
    public:
    bool currentlyPressed() override {
      printf("???\n");
      return false;
    }
  };

  {
    test_printf("Testing Button");

    {
      MockButton spyMe;
      Mock<MockButton> mockFactory(spyMe);
      When(Method(mockFactory, currentlyPressed)).Return(false).Return(true).Return(false);
      Spy(Method(mockFactory, isPressed));
      MockButton &btn = mockFactory.get();

      test("Button isPressed should be false",
           TEST_BODY(AssertThat, btn.isPressed(), Equals(false)));
      test("Button isPressed should be true", TEST_BODY(AssertThat, btn.isPressed(), Equals(true)));
      test("Button isPressed should be false",
           TEST_BODY(AssertThat, btn.isPressed(), Equals(false)));
    }

    {
      MockButton spyMe;
      Mock<MockButton> mockFactory(spyMe);
      When(Method(mockFactory, currentlyPressed))
        .Return(false)
        .Return(true)
        .Return(true)
        .Return(false)
        .Return(false);
      Spy(Method(mockFactory, changed));
      MockButton &btn = mockFactory.get();

      test("Button changed should be false", TEST_BODY(AssertThat, btn.changed(), Equals(false)));
      test("Button changed should be true", TEST_BODY(AssertThat, btn.changed(), Equals(true)));
      test("Button changed should be false", TEST_BODY(AssertThat, btn.changed(), Equals(false)));
      test("Button changed should be true", TEST_BODY(AssertThat, btn.changed(), Equals(true)));
      test("Button changed should be false", TEST_BODY(AssertThat, btn.changed(), Equals(false)));
    }

    {
      MockButton spyMe;
      Mock<MockButton> mockFactory(spyMe);
      When(Method(mockFactory, currentlyPressed))
        .Return(false)
        .Return(true)
        .Return(true)
        .Return(false)
        .Return(false);
      Spy(Method(mockFactory, changed));
      Spy(Method(mockFactory, changedToPressed));
      MockButton &btn = mockFactory.get();

      test("Button changedToPressed should be false",
           TEST_BODY(AssertThat, btn.changedToPressed(), Equals(false)));
      test("Button changedToPressed should be true",
           TEST_BODY(AssertThat, btn.changedToPressed(), Equals(true)));
      test("Button changedToPressed should be false",
           TEST_BODY(AssertThat, btn.changedToPressed(), Equals(false)));
      test("Button changedToPressed should be true",
           TEST_BODY(AssertThat, btn.changedToPressed(), Equals(false)));
      test("Button changedToPressed should be false",
           TEST_BODY(AssertThat, btn.changedToPressed(), Equals(false)));
    }

    {
      MockButton spyMe;
      Mock<MockButton> mockFactory(spyMe);
      When(Method(mockFactory, currentlyPressed))
        .Return(false)
        .Return(true)
        .Return(true)
        .Return(false)
        .Return(false);
      Spy(Method(mockFactory, changed));
      Spy(Method(mockFactory, changedToReleased));
      MockButton &btn = mockFactory.get();

      test("Button changedToReleased should be false",
           TEST_BODY(AssertThat, btn.changedToReleased(), Equals(false)));
      test("Button changedToReleased should be true",
           TEST_BODY(AssertThat, btn.changedToReleased(), Equals(false)));
      test("Button changedToReleased should be false",
           TEST_BODY(AssertThat, btn.changedToReleased(), Equals(false)));
      test("Button changedToReleased should be true",
           TEST_BODY(AssertThat, btn.changedToReleased(), Equals(true)));
      test("Button changedToReleased should be false",
           TEST_BODY(AssertThat, btn.changedToReleased(), Equals(false)));
    }
  }
}

void testFilters() {
  auto assertThatFilterAndFilterOutputAreEqual = [](okapi::Filter *filt, double input, double value,
                                                    double delta) {
    auto text = "Assert that filter and filter output are equal and correct for input = " +
                std::to_string(input);
    test(text, [&]() {
      AssertThat(filt->filter(input), EqualsWithDelta(value, delta));
      AssertThat(filt->getOutput(), EqualsWithDelta(value, delta));
    });
  };

  {
    test_printf("Testing AverageFilter");

    AverageFilter<5> filt;

    for (int i = 0; i < 10; i++) {
      auto testName = "AverageFilter i = " + std::to_string(i);
      switch (i) {
      case 0: {
        assertThatFilterAndFilterOutputAreEqual(&filt, i, 0, 0.0001);
        break;
      }

      case 1: {
        assertThatFilterAndFilterOutputAreEqual(&filt, i, 0.2, 0.0001);
        break;
      }

      case 2: {
        assertThatFilterAndFilterOutputAreEqual(&filt, i, 0.6, 0.0001);
        break;
      }

      case 3: {
        assertThatFilterAndFilterOutputAreEqual(&filt, i, 1.2, 0.0001);
        break;
      }

      default: {
        assertThatFilterAndFilterOutputAreEqual(&filt, i, i - 2, 0.0001);
        break;
      }
      }
    }
  }

  {
    test_printf("Testing MedianFilter");

    MedianFilter<5> filt;

    for (int i = 0; i < 10; i++) {
      auto testName = "MedianFilter i = " + std::to_string(i);
      if (i < 3) {
        assertThatFilterAndFilterOutputAreEqual(&filt, i, 0, 0.0001);
      } else {
        assertThatFilterAndFilterOutputAreEqual(&filt, i, i - 2, 0.0001);
      }
    }
  }

  {
    test_printf("Testing EmaFilter");

    EmaFilter filt(0.5);
    assertThatFilterAndFilterOutputAreEqual(&filt, 0, 0, 0.0001);
    assertThatFilterAndFilterOutputAreEqual(&filt, 1, 0.5, 0.0001);
    assertThatFilterAndFilterOutputAreEqual(&filt, 2, 1.25, 0.0001);
    assertThatFilterAndFilterOutputAreEqual(&filt, -3, -0.875, 0.0001);

    EmaFilter filt2(1);
    assertThatFilterAndFilterOutputAreEqual(&filt2, 5, 5, 0.0001);
    assertThatFilterAndFilterOutputAreEqual(&filt2, 6, 6, 0.0001);
    assertThatFilterAndFilterOutputAreEqual(&filt2, 7, 7, 0.0001);
  }

  {
    test_printf("Testing DemaFilter");

    DemaFilter filt(0.5, 0.05);

    assertThatFilterAndFilterOutputAreEqual(&filt, 0, 0, 0.0001);
    assertThatFilterAndFilterOutputAreEqual(&filt, 1, 0.525, 0.0001);
    assertThatFilterAndFilterOutputAreEqual(&filt, 2, 1.3244, 0.0001);
    assertThatFilterAndFilterOutputAreEqual(&filt, 2, 1.7410, 0.0001);
    assertThatFilterAndFilterOutputAreEqual(&filt, 2, 1.9557, 0.0001);

    DemaFilter filt2(1, 0);
    assertThatFilterAndFilterOutputAreEqual(&filt2, 5, 5, 0.0001);
    assertThatFilterAndFilterOutputAreEqual(&filt2, 6, 6, 0.0001);
    assertThatFilterAndFilterOutputAreEqual(&filt2, 7, 7, 0.0001);
  }

  {
    test_printf("Testing EKFFilter");

    EKFFilter filt(0.0001, ipow(0.2, 2));
    assertThatFilterAndFilterOutputAreEqual(&filt, 0, 0, 0.0001);
    assertThatFilterAndFilterOutputAreEqual(&filt, 0.5, 0.2454, 0.0001);
    assertThatFilterAndFilterOutputAreEqual(&filt, -0.5, -0.0008, 0.0001);
    assertThatFilterAndFilterOutputAreEqual(&filt, 0.5, 0.1242, 0.0001);
    assertThatFilterAndFilterOutputAreEqual(&filt, 0, 0.0992, 0.0001);
  }

  {
    test_printf("Testing ComposableFilter");

    ComposableFilter filt(
      {std::make_shared<AverageFilter<3>>(), std::make_shared<AverageFilter<3>>()});
    assertThatFilterAndFilterOutputAreEqual(&filt, 1, 0.1111, 0.0001);
    assertThatFilterAndFilterOutputAreEqual(&filt, 2, 0.4444, 0.0001);
    assertThatFilterAndFilterOutputAreEqual(&filt, 3, 1.1111, 0.0001);

    for (int i = 4; i < 10; i++) {
      assertThatFilterAndFilterOutputAreEqual(&filt, i, i - 2, 0.0001);
    }
  }

  {
    test_printf("Testing PassthroughFilter");

    PassthroughFilter filt;
    for (int i = 0; i < 5; i++) {
      assertThatFilterAndFilterOutputAreEqual(&filt, i, i, 0.0001);
    }
  }

  {
    test_printf("Testing VelMath");

    {
      class MockTimer : public AbstractTimer {
        public:
        QTime millis() const override {
          return QTime();
        }

        QTime getStartingTime() const override {
          return QTime();
        }

        QTime getDtFromStart() const override {
          return QTime();
        }

        void placeMark() override {
        }

        void placeHardMark() override {
        }

        QTime clearHardMark() override {
          return second;
        }

        QTime getDtFromMark() const override {
          return QTime();
        }

        QTime getDtFromHardMark() const override {
          return QTime();
        }

        bool repeat(const QTime time) override {
          return false;
        }

        bool repeat(const QFrequency frequency) override {
          return false;
        }

        QTime getDt() override {
          return 10_ms;
        }
      };

      VelMath velMath(360, std::make_shared<PassthroughFilter>(), std::make_unique<MockTimer>());

      for (int i = 0; i < 10; i++) {
        if (i == 0) {
          test("VelMath " + std::to_string(i), [&]() {
            AssertThat(velMath.step(i * 10).convert(rpm), EqualsWithDelta(0, 0.01));
            AssertThat(velMath.getVelocity().convert(rpm), EqualsWithDelta(0, 0.01));
          });
        } else {
          // 10 ticks per 100 ms should be ~16.67 rpm
          test("VelMath " + std::to_string(i), [&]() {
            AssertThat(velMath.step(i * 10).convert(rpm), EqualsWithDelta(166.67, 0.01));
            AssertThat(velMath.getVelocity().convert(rpm), EqualsWithDelta(166.67, 0.01));
          });
        }
      }
    }
  }
}

void testUtil() {
  {
    test_printf("Testing ipow");

    test_printf("Integer tests");
    test("0^0 == 1", TEST_BODY(AssertThat, ipow(0, 0), Equals(1)));
    test("0^1 == 0", TEST_BODY(AssertThat, ipow(0, 1), Equals(0)));
    test("1^0 == 1", TEST_BODY(AssertThat, ipow(1, 0), Equals(1)));
    test("1^1 == 1", TEST_BODY(AssertThat, ipow(1, 1), Equals(1)));
    test("2^1 == 2", TEST_BODY(AssertThat, ipow(2, 1), Equals(2)));
    test("2^2 == 4", TEST_BODY(AssertThat, ipow(2, 2), Equals(4)));

    test_printf("Floating point tests");
    test("0.5^1 == 0.5", TEST_BODY(AssertThat, ipow(0.5, 1), EqualsWithDelta(0.5, 0.0001)));
    test("2.5^2 == 6.25", TEST_BODY(AssertThat, ipow(2.5, 2), EqualsWithDelta(6.25, 0.0001)));
  }

  {
    test_printf("Testing cutRange");

    test("1 : [-2, 2] -> 0", TEST_BODY(AssertThat, cutRange(1, -2, 2), EqualsWithDelta(2, 0.0001)));
    test("2 : [-2, 2] -> 0", TEST_BODY(AssertThat, cutRange(2, -2, 2), EqualsWithDelta(2, 0.0001)));
    test("0 : [-2, 2] -> 0", TEST_BODY(AssertThat, cutRange(0, -2, 2), EqualsWithDelta(2, 0.0001)));
    test("-2 : [-2, 2] -> 0",
         TEST_BODY(AssertThat, cutRange(-2, -2, 2), EqualsWithDelta(-2, 0.0001)));
    test("-3 : [-2, 2] -> -3",
         TEST_BODY(AssertThat, cutRange(-3, -2, 2), EqualsWithDelta(-3, 0.0001)));
    test("3 : [-2, 2] -> -3",
         TEST_BODY(AssertThat, cutRange(3, -2, 2), EqualsWithDelta(3, 0.0001)));
  }

  {
    test_printf("Testing deadband");

    test("0 : [-2, 2] -> 0", TEST_BODY(AssertThat, deadband(0, -2, 2), EqualsWithDelta(0, 0.0001)));
    test("1 : [-2, 2] -> 0", TEST_BODY(AssertThat, deadband(1, -2, 2), EqualsWithDelta(0, 0.0001)));
    test("2 : [-2, 2] -> 0", TEST_BODY(AssertThat, deadband(2, -2, 2), EqualsWithDelta(0, 0.0001)));
    test("-2 : [-2, 2] -> 0",
         TEST_BODY(AssertThat, deadband(-2, -2, 2), EqualsWithDelta(0, 0.0001)));
    test("3 : [-2, 2] -> 3", TEST_BODY(AssertThat, deadband(3, -2, 2), EqualsWithDelta(3, 0.0001)));
    test("-3 : [-2, 2] -> -3",
         TEST_BODY(AssertThat, deadband(-3, -2, 2), EqualsWithDelta(-3, 0.0001)));
  }

  {
    test_printf("Testing remapRange");

    test("0 : [-1, 1] -> [-2, 2]",
         TEST_BODY(AssertThat, remapRange(0, -1, 1, -2, 2), EqualsWithDelta(0, 0.0001)));
    test("0.1 : [-1, 1] -> [-2, 2]",
         TEST_BODY(AssertThat, remapRange(0.1, -1, 1, -2, 2), EqualsWithDelta(0.2, 0.0001)));
    test("-0.1 : [-1, 1] -> [2, -2]",
         TEST_BODY(AssertThat, remapRange(-0.1, -1, 1, 2, -2), EqualsWithDelta(0.2, 0.0001)));
    test("0 : [-1, 1] -> [-5, 2]",
         TEST_BODY(AssertThat, remapRange(0, -1, 1, -5, 2), EqualsWithDelta(-1.5, 0.0001)));
  }
}

int main() {
  testChassisScales();
  testChassisModels();
  testControlUtils();
  testButtons();
  testFilters();
  testUtil();

  test_print_report();

  return 0;
}

#endif
