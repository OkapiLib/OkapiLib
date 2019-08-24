/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#pragma once

#include "okapi/api/chassis/controller/chassisController.hpp"
#include "okapi/api/chassis/model/skidSteerModel.hpp"
#include "okapi/api/control/async/asyncPosIntegratedController.hpp"
#include "okapi/api/control/async/asyncVelIntegratedController.hpp"
#include "okapi/api/control/async/asyncWrapper.hpp"
#include "okapi/api/control/iterative/iterativePosPidController.hpp"
#include "okapi/api/control/util/flywheelSimulator.hpp"
#include "okapi/api/control/util/settledUtil.hpp"
#include "okapi/api/device/motor/abstractMotor.hpp"
#include "okapi/api/device/rotarysensor/continuousRotarySensor.hpp"
#include "okapi/api/odometry/odometry.hpp"
#include "okapi/api/util/abstractRate.hpp"
#include "okapi/api/util/abstractTimer.hpp"
#include "okapi/api/util/timeUtil.hpp"
#include <atomic>
#include <chrono>
#include <gtest/gtest.h>

namespace okapi {

class MockContinuousRotarySensor : public ContinuousRotarySensor {
  public:
  double controllerGet() override;

  int32_t reset() override;

  double get() const override;

  mutable std::int32_t value{0};
};

/**
 * A motor mock that saves the last set position, velocity, and voltage.
 */
class MockMotor : public AbstractMotor {
  public:
  MockMotor();

  void controllerSet(double ivalue) override;

  int32_t moveAbsolute(double iposition, std::int32_t ivelocity) override;

  int32_t moveRelative(double iposition, std::int32_t ivelocity) override;

  double getTargetPosition() override;

  double getPosition() override;

  int32_t getTargetVelocity() override;

  double getActualVelocity() override;

  int32_t tarePosition() override;

  int32_t setBrakeMode(brakeMode imode) override;

  int32_t setCurrentLimit(std::int32_t ilimit) override;

  int32_t setEncoderUnits(encoderUnits iunits) override;

  int32_t setGearing(gearset igearset) override;

  int32_t setReversed(bool ireverse) override;

  int32_t setVoltageLimit(std::int32_t ilimit) override;

  std::shared_ptr<ContinuousRotarySensor> getEncoder() override;

  virtual std::shared_ptr<MockContinuousRotarySensor> getMockEncoder();

  std::int32_t moveVelocity(std::int16_t ivelocity) override;

  std::int32_t moveVoltage(std::int16_t ivoltage) override;

  int32_t getCurrentDraw() override;

  int32_t getDirection() override;

  double getEfficiency() override;

  int32_t isOverCurrent() override;

  int32_t isOverTemp() override;

  int32_t isStopped() override;

  int32_t getZeroPositionFlag() override;

  uint32_t getFaults() override;

  uint32_t getFlags() override;

  int32_t getRawPosition(std::uint32_t *timestamp) override;

  double getPower() override;

  double getTemperature() override;

  double getTorque() override;

  int32_t getVoltage() override;

  int32_t modifyProfiledVelocity(std::int32_t ivelocity) override;

  AbstractMotor::brakeMode getBrakeMode() override;

  int32_t getCurrentLimit() override;

  AbstractMotor::encoderUnits getEncoderUnits() override;

  AbstractMotor::gearset getGearing() override;

  std::shared_ptr<MockContinuousRotarySensor> encoder;
  mutable std::int16_t lastVelocity{0};
  mutable std::int16_t maxVelocity{0};
  mutable std::int16_t lastVoltage{0};
  mutable std::int16_t lastPosition{0};
  mutable std::int32_t lastProfiledMaxVelocity{0};
  AbstractMotor::gearset gearset{AbstractMotor::gearset::green};
  AbstractMotor::encoderUnits encoderUnits{AbstractMotor::encoderUnits::counts};
  AbstractMotor::brakeMode brakeMode{AbstractMotor::brakeMode::coast};
};

class ThreadedMockMotor : public AbstractMotor {
  public:
  ThreadedMockMotor();

  void controllerSet(double ivalue) override;

  int32_t moveAbsolute(double iposition, std::int32_t ivelocity) override;

  int32_t moveRelative(double iposition, std::int32_t ivelocity) override;

  int32_t moveVelocity(std::int16_t ivelocity) override;

  int32_t moveVoltage(std::int16_t ivoltage) override;

  int32_t modifyProfiledVelocity(std::int32_t ivelocity) override;

  double getTargetPosition() override;

  double getPosition() override;

  int32_t tarePosition() override;

  int32_t getTargetVelocity() override;

  double getActualVelocity() override;

  int32_t getCurrentDraw() override;

  int32_t getDirection() override;

  double getEfficiency() override;

  int32_t isOverCurrent() override;

  int32_t isOverTemp() override;

  int32_t isStopped() override;

  int32_t getZeroPositionFlag() override;

  uint32_t getFaults() override;

  uint32_t getFlags() override;

  int32_t getRawPosition(std::uint32_t *timestamp) override;

  double getPower() override;

  double getTemperature() override;

  double getTorque() override;

  int32_t getVoltage() override;

  int32_t setBrakeMode(brakeMode imode) override;

  brakeMode getBrakeMode() override;

  int32_t setCurrentLimit(std::int32_t ilimit) override;

  int32_t getCurrentLimit() override;

  int32_t setEncoderUnits(encoderUnits iunits) override;

  encoderUnits getEncoderUnits() override;

  int32_t setGearing(gearset igearset) override;

  gearset getGearing() override;

  int32_t setReversed(bool ireverse) override;

  int32_t setVoltageLimit(std::int32_t ilimit) override;

  std::shared_ptr<ContinuousRotarySensor> getEncoder() override;

  void startThread();

  void stopThread();

  void threadFunc();

  std::thread thread;
  int reverse{1};
  AbstractMotor::gearset gearset{AbstractMotor::gearset::green};
  AbstractMotor::encoderUnits encoderUnits{AbstractMotor::encoderUnits::counts};
  AbstractMotor::brakeMode brakeMode{AbstractMotor::brakeMode::coast};
  std::shared_ptr<MockContinuousRotarySensor> encoder;

  // User set position
  double targetPosition{0};

  // User set profiled velocity target
  std::int32_t targetProfiledVelocity{toUnderlyingType(gearset)};

  // User set velocity
  std::int16_t targetVelocity{0};

  // User set voltage
  std::int16_t setVoltage{0};

  // User set voltage limit
  std::int32_t voltageLimit{12000};

  // User set current limit
  std::int32_t currentLimit{2500};

  enum e_mode { position = 1, velocity = 2, voltage = 3 } mode{velocity};
  double actualPosition{0};
  double actualVelocity{0};
  int dt{1};
  bool threadShouldStop{false};
};

/**
 * A timer mock that implements all features using the system timer.
 */
class MockTimer : public AbstractTimer {
  public:
  MockTimer();

  QTime millis() const override;

  std::chrono::system_clock::time_point epoch = std::chrono::high_resolution_clock::from_time_t(0);
};

/**
 * A timer mock that always returns a constant dt and 0 for other methods.
 */
class ConstantMockTimer : public AbstractTimer {
  public:
  explicit ConstantMockTimer(QTime idt);

  QTime millis() const override;

  QTime getDt() override;

  QTime readDt() const override;

  QTime getStartingTime() const override;

  QTime getDtFromStart() const override;

  void placeMark() override;

  QTime clearMark() override;

  void placeHardMark() override;

  QTime clearHardMark() override;

  QTime getDtFromMark() const override;

  QTime getDtFromHardMark() const override;

  bool repeat(QTime time) override;

  bool repeat(QFrequency frequency) override;

  QTime dtToReturn;
};

class MockRate : public AbstractRate {
  public:
  MockRate();

  void delay(QFrequency ihz) override;

  void delay(int ihz) override;

  void delayUntil(QTime itime) override;

  void delayUntil(uint32_t ims) override;
};

class MockControllerInput : public ControllerInput<double> {
  public:
  virtual ~MockControllerInput() = default;

  double controllerGet() override {
    return reading;
  }

  double reading{0};
};

std::unique_ptr<SettledUtil> createSettledUtilPtr(double iatTargetError = 50,
                                                  double iatTargetDerivative = 5,
                                                  QTime iatTargetTime = 250_ms);

TimeUtil createTimeUtil();

TimeUtil createConstantTimeUtil(QTime idt);

TimeUtil createTimeUtil(const Supplier<std::unique_ptr<AbstractTimer>> &itimerSupplier);

TimeUtil createTimeUtil(const Supplier<std::unique_ptr<SettledUtil>> &isettledUtilSupplier);

class SimulatedSystem : public ControllerInput<double>, public ControllerOutput<double> {
  public:
  explicit SimulatedSystem(FlywheelSimulator &simulator);

  virtual ~SimulatedSystem();

  double controllerGet() override;

  void controllerSet(double ivalue) override;

  void step();

  static void trampoline(void *system);

  void startThread();

  void join();

  FlywheelSimulator &simulator;
  MockRate rate{};
  std::atomic_bool dtorCalled{false};
  std::thread thread;
};

class MockAsyncPosIntegratedController : public AsyncPosIntegratedController {
  public:
  MockAsyncPosIntegratedController();

  explicit MockAsyncPosIntegratedController(const TimeUtil &itimeUtil);

  bool isSettled() override;

  bool isSettledOverride{true};
  using AsyncPosIntegratedController::maxVelocity;
};

class MockAsyncVelIntegratedController : public AsyncVelIntegratedController {
  public:
  MockAsyncVelIntegratedController();

  void setTarget(double itarget) override;

  bool isSettled() override;

  void controllerSet(double ivalue) override;

  bool isSettledOverride{true};

  double lastTarget{0};
  double maxTarget{0};

  double lastControllerOutputSet{0};
  double maxControllerOutputSet{0};
};

class MockIterativeController : public IterativePosPIDController {
  public:
  MockIterativeController();

  explicit MockIterativeController(double ikP);

  bool isSettled() override;

  bool isSettledOverride{true};
};

void assertMotorsHaveBeenStopped(MockMotor *leftMotor, MockMotor *rightMotor);

void assertMotorsGearsetEquals(AbstractMotor::gearset expected,
                               const std::initializer_list<MockMotor> &motors);

void assertMotorsBrakeModeEquals(AbstractMotor::brakeMode expected,
                                 const std::initializer_list<MockMotor> &motors);

void assertMotorsEncoderUnitsEquals(AbstractMotor::encoderUnits expected,
                                    const std::initializer_list<MockMotor> &motors);

template <typename I, typename O>
void assertControllerIsSettledWhenDisabled(ClosedLoopController<I, O> &controller, I target) {
  controller.flipDisable(false);
  EXPECT_FALSE(controller.isDisabled());
  controller.setTarget(target);
  EXPECT_EQ(controller.getTarget(), target);
  EXPECT_FALSE(controller.isSettled());

  controller.flipDisable();
  EXPECT_TRUE(controller.isDisabled());
  EXPECT_TRUE(controller.isSettled());
}

template <typename I, typename O>
void assertWaitUntilSettledWorksWhenDisabled(AsyncController<I, O> &controller) {
  controller.flipDisable(true);
  EXPECT_TRUE(controller.isDisabled());
  controller.waitUntilSettled();
}

void assertAsyncControllerFollowsDisableLifecycle(AsyncController<double, double> &controller,
                                                  std::int16_t &domainValue,
                                                  std::int16_t &voltageValue,
                                                  int expectedOutput);

void assertIterativeControllerFollowsDisableLifecycle(
  IterativeController<double, double> &controller);

void assertControllerFollowsTargetLifecycle(ClosedLoopController<double, double> &controller);

void assertIterativeControllerScalesControllerSetTargets(
  IterativeController<double, double> &controller);

void assertAsyncWrapperScalesControllerSetTargets(AsyncWrapper<double, double> &controller);

void assertOdomStateEquals(double x, double y, double theta, const OdomState &actual);

void assertOdomStateEquals(Odometry *odom, QLength x, QLength y, QAngle theta);

class MockReadOnlyChassisModel : public ReadOnlyChassisModel {
  public:
  std::valarray<int32_t> getSensorVals() const override {
    return {0, 0};
  }
};

class MockChassisModel : public ChassisModel {
  public:
  MockChassisModel() : ChassisModel() {
  }

  void forward(double ispeed) override {
    lastForward = ispeed;
  }

  void driveVector(double iySpeed, double izRotation) override {
    lastVectorY = iySpeed;
    lastVectorZ = izRotation;
  }

  void driveVectorVoltage(double iySpeed, double izRotation) override {
    lastVectorY = iySpeed;
    lastVectorZ = izRotation;
  }

  void rotate(double ispeed) override {
    lastRotate = ispeed;
  }

  void stop() override {
    stopWasCalled = true;
  }

  void tank(double ileftSpeed, double irightSpeed, double) override {
    lastTankLeft = ileftSpeed;
    lastTankRight = irightSpeed;
  }

  void arcade(double iySpeed, double izRotation, double) override {
    lastArcadeY = iySpeed;
    lastArcadeZ = izRotation;
  }

  void left(double ispeed) override {
    lastLeft = ispeed;
  }

  void right(double ispeed) override {
    lastRight = ispeed;
  }

  void resetSensors() override {
    resetSensorsWasCalled = true;
  }

  void setBrakeMode(AbstractMotor::brakeMode mode) override {
    lastBrakeMode = mode;
  }

  void setEncoderUnits(AbstractMotor::encoderUnits units) override {
    lastEncoderUnits = units;
  }

  void setGearing(AbstractMotor::gearset gearset) override {
    lastGearset = gearset;
  }

  std::valarray<int32_t> getSensorVals() const override {
    return {0, 0};
  }

  void setMaxVelocity(double imaxVelocity) override {
    maxVelocity = imaxVelocity;
  }
  double getMaxVelocity() const override {
    return maxVelocity;
  }
  void setMaxVoltage(double imaxVoltage) override {
    maxVoltage = imaxVoltage;
  }
  double getMaxVoltage() const override {
    return maxVoltage;
  }

  mutable double lastForward{0};
  mutable double lastVectorY{0};
  mutable double lastVectorZ{0};
  mutable double lastRotate{0};
  mutable bool stopWasCalled{false};
  mutable double lastTankLeft{0};
  mutable double lastTankRight{0};
  mutable double lastArcadeY{0};
  mutable double lastArcadeZ{0};
  mutable double lastLeft{0};
  mutable double lastRight{0};
  mutable bool resetSensorsWasCalled{false};
  double maxVelocity{600};
  double maxVoltage{12000};
  mutable AbstractMotor::brakeMode lastBrakeMode{AbstractMotor::brakeMode::invalid};
  mutable AbstractMotor::encoderUnits lastEncoderUnits{AbstractMotor::encoderUnits::invalid};
  mutable AbstractMotor::gearset lastGearset{AbstractMotor::gearset::invalid};
};

class MockSkidSteerModel : public SkidSteerModel {
  public:
  explicit MockSkidSteerModel(
    const std::shared_ptr<MockMotor> &ileftMtr = std::make_shared<MockMotor>(),
    const std::shared_ptr<MockMotor> &irightMtr = std::make_shared<MockMotor>())
    : SkidSteerModel(ileftMtr,
                     irightMtr,
                     ileftMtr->getMockEncoder(),
                     irightMtr->getMockEncoder(),
                     600,
                     v5MotorMaxVoltage),
      leftMtr(ileftMtr),
      rightMtr(irightMtr),
      leftEnc(ileftMtr->getMockEncoder()),
      rightEnc(irightMtr->getMockEncoder()) {
  }

  std::valarray<std::int32_t> getSensorVals() const override {
    return std::valarray<std::int32_t>{static_cast<int>(leftEnc->get()),
                                       static_cast<int>(rightEnc->get())};
  }

  void setSensorVals(std::int32_t left, std::int32_t right) {
    leftEnc->value = left;
    rightEnc->value = right;
  }

  std::shared_ptr<MockMotor> leftMtr;
  std::shared_ptr<MockMotor> rightMtr;
  std::shared_ptr<MockContinuousRotarySensor> leftEnc;
  std::shared_ptr<MockContinuousRotarySensor> rightEnc;
};

class MockChassisController : public ChassisController {
  public:
  void moveDistance(QLength itarget) override {
    lastMoveDistanceTargetQLength = itarget;
  }
  void moveDistance(double itarget) override {
    lastMoveDistanceTargetDouble = itarget;
  }
  void moveDistanceAsync(QLength itarget) override {
    moveDistance(itarget);
  }
  void moveDistanceAsync(double itarget) override {
    moveDistance(itarget);
  }
  void turnAngle(QAngle idegTarget) override {
    lastTurnAngleTargetQAngle = idegTarget;
  }
  void turnAngle(double idegTarget) override {
    lastTurnAngleTargetDouble = idegTarget;
  }
  void turnAngleAsync(QAngle idegTarget) override {
    turnAngle(idegTarget);
  }
  void turnAngleAsync(double idegTarget) override {
    turnAngle(idegTarget);
  }
  void setTurnsMirrored(bool ishouldMirror) override {
    turnsMirrored = ishouldMirror;
  }
  void waitUntilSettled() override {
    waitUntilSettledCalled++;
  }
  void stop() override {
    stopCalled++;
  }
  ChassisScales getChassisScales() const override {
    return scales;
  }
  AbstractMotor::GearsetRatioPair getGearsetRatioPair() const override {
    return gearset;
  }
  std::shared_ptr<ChassisModel> getModel() override {
    return chassisModel;
  }
  ChassisModel &model() override {
    return *chassisModel;
  }

  QLength lastMoveDistanceTargetQLength;
  double lastMoveDistanceTargetDouble;
  QAngle lastTurnAngleTargetQAngle;
  double lastTurnAngleTargetDouble;
  bool turnsMirrored{false};
  int waitUntilSettledCalled{0};
  int stopCalled{0};
  ChassisScales scales{{4.125_in, 10_in}, imev5GreenTPR};
  AbstractMotor::GearsetRatioPair gearset{AbstractMotor::gearset::green};
  std::shared_ptr<MockChassisModel> chassisModel = std::make_shared<MockChassisModel>();
};
} // namespace okapi
