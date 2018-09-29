/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "test/tests/api/implMocks.hpp"
#include "okapi/api/control/util/settledUtil.hpp"
#include "okapi/api/device/motor/abstractMotor.hpp"
#include "okapi/api/util/abstractTimer.hpp"
#include <chrono>
#include <memory>

namespace okapi {
double MockContinuousRotarySensor::controllerGet() {
  return value;
}

int32_t MockContinuousRotarySensor::reset() {
  value = 0;
  return 0;
}

double MockContinuousRotarySensor::get() const {
  return value;
}

MockMotor::MockMotor() : encoder(std::make_shared<MockContinuousRotarySensor>()) {
}

void MockMotor::controllerSet(const double ivalue) {
  moveVelocity((int16_t)ivalue);
}

int32_t MockMotor::moveAbsolute(const double iposition, const std::int32_t ivelocity) const {
  lastPosition = (int16_t)iposition;
  lastProfiledMaxVelocity = ivelocity;
  return 0;
}

int32_t MockMotor::moveRelative(const double iposition, const std::int32_t ivelocity) const {
  lastPosition += iposition;
  lastProfiledMaxVelocity = ivelocity;
  return 0;
}

double MockMotor::getTargetPosition() const {
  return 0;
}

double MockMotor::getPosition() const {
  return encoder->get();
}

int32_t MockMotor::getTargetVelocity() const {
  return 0;
}

double MockMotor::getActualVelocity() const {
  return 0;
}

int32_t MockMotor::tarePosition() const {
  return 0;
}

int32_t MockMotor::setBrakeMode(const AbstractMotor::brakeMode imode) {
  brakeMode = imode;
  return 0;
}

int32_t MockMotor::setCurrentLimit(const std::int32_t) const {
  return 0;
}

int32_t MockMotor::setEncoderUnits(const AbstractMotor::encoderUnits iunits) {
  encoderUnits = iunits;
  return 0;
}

int32_t MockMotor::setGearing(const AbstractMotor::gearset igearset) {
  gearset = igearset;
  return 0;
}

int32_t MockMotor::setReversed(const bool) const {
  return 0;
}

int32_t MockMotor::setVoltageLimit(const std::int32_t) const {
  return 0;
}

std::shared_ptr<ContinuousRotarySensor> MockMotor::getEncoder() const {
  return encoder;
}

std::int32_t MockMotor::moveVelocity(const std::int16_t ivelocity) const {
  lastVelocity = ivelocity;
  if (ivelocity > maxVelocity) {
    maxVelocity = ivelocity;
  }
  return 1;
}

std::int32_t MockMotor::moveVoltage(const std::int16_t ivoltage) const {
  lastVoltage = ivoltage;
  return 1;
}

int32_t MockMotor::getCurrentDraw() const {
  return 0;
}

int32_t MockMotor::getDirection() const {
  return 0;
}

double MockMotor::getEfficiency() const {
  return 0;
}

int32_t MockMotor::isOverCurrent() const {
  return 0;
}

int32_t MockMotor::isOverTemp() const {
  return 0;
}

int32_t MockMotor::isStopped() const {
  return 0;
}

int32_t MockMotor::getZeroPositionFlag() const {
  return 0;
}

uint32_t MockMotor::getFaults() const {
  return 0;
}

uint32_t MockMotor::getFlags() const {
  return 0;
}

int32_t MockMotor::getRawPosition(std::uint32_t *) const {
  return encoder->get();
}

double MockMotor::getPower() const {
  return 0;
}

double MockMotor::getTemperature() const {
  return 0;
}

double MockMotor::getTorque() const {
  return 0;
}

int32_t MockMotor::getVoltage() const {
  return 0;
}

int32_t MockMotor::modifyProfiledVelocity(std::int32_t) const {
  return 0;
}

int32_t MockMotor::setPosPID(double, double, double, double) const {
  return 0;
}

int32_t
MockMotor::setPosPIDFull(double, double, double, double, double, double, double, double) const {
  return 0;
}

int32_t MockMotor::setVelPID(double, double, double, double) const {
  return 0;
}

int32_t
MockMotor::setVelPIDFull(double, double, double, double, double, double, double, double) const {
  return 0;
}

AbstractMotor::brakeMode MockMotor::getBrakeMode() const {
  return brakeMode;
}

int32_t MockMotor::getCurrentLimit() const {
  return 2500;
}

AbstractMotor::encoderUnits MockMotor::getEncoderUnits() const {
  return encoderUnits;
}

AbstractMotor::gearset MockMotor::getGearing() const {
  return gearset;
}

MockTimer::MockTimer() : AbstractTimer(millis()) {
}

QTime MockTimer::millis() const {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
           std::chrono::high_resolution_clock::now() - epoch)
           .count() *
         millisecond;
}

ConstantMockTimer::ConstantMockTimer(const QTime idt) : AbstractTimer(0_ms), dtToReturn(idt) {
}

QTime ConstantMockTimer::millis() const {
  return 0_ms;
}

QTime ConstantMockTimer::getDt() {
  return dtToReturn;
}

QTime ConstantMockTimer::readDt() const {
  return dtToReturn;
}

QTime ConstantMockTimer::getStartingTime() const {
  return 0_ms;
}

QTime ConstantMockTimer::getDtFromStart() const {
  return dtToReturn;
}

void ConstantMockTimer::placeMark() {
}

void ConstantMockTimer::placeHardMark() {
}

QTime ConstantMockTimer::clearHardMark() {
  return 0_ms;
}

QTime ConstantMockTimer::getDtFromMark() const {
  return dtToReturn;
}

QTime ConstantMockTimer::getDtFromHardMark() const {
  return dtToReturn;
}

bool ConstantMockTimer::repeat(QTime) {
  return false;
}

bool ConstantMockTimer::repeat(QFrequency) {
  return false;
}

QTime ConstantMockTimer::clearMark() {
  return 0_ms;
}

MockRate::MockRate() = default;

void MockRate::delay(QFrequency ihz) {
  delay(ihz.convert(Hz));
}

void MockRate::delay(int ihz) {
  std::this_thread::sleep_for(std::chrono::milliseconds(1000 / ihz));
}

void MockRate::delayUntil(QTime itime) {
  delayUntil(itime.convert(millisecond));
}

void MockRate::delayUntil(uint32_t ims) {
  std::this_thread::sleep_for(std::chrono::milliseconds(ims));
}

std::unique_ptr<SettledUtil> createSettledUtilPtr(const double iatTargetError,
                                                  const double iatTargetDerivative,
                                                  const QTime iatTargetTime) {
  return std::make_unique<SettledUtil>(
    std::make_unique<MockTimer>(), iatTargetError, iatTargetDerivative, iatTargetTime);
}

TimeUtil createTimeUtil() {
  return TimeUtil(
    Supplier<std::unique_ptr<AbstractTimer>>([]() { return std::make_unique<MockTimer>(); }),
    Supplier<std::unique_ptr<AbstractRate>>([]() { return std::make_unique<MockRate>(); }),
    Supplier<std::unique_ptr<SettledUtil>>([]() { return createSettledUtilPtr(); }));
}

TimeUtil createConstantTimeUtil(const QTime idt) {
  return TimeUtil(
    Supplier<std::unique_ptr<AbstractTimer>>(
      [=]() { return std::make_unique<ConstantMockTimer>(idt); }),
    Supplier<std::unique_ptr<AbstractRate>>([]() { return std::make_unique<MockRate>(); }),
    Supplier<std::unique_ptr<SettledUtil>>([]() { return createSettledUtilPtr(); }));
}

TimeUtil createTimeUtil(const Supplier<std::unique_ptr<AbstractTimer>> &itimerSupplier) {
  return TimeUtil(
    itimerSupplier,
    Supplier<std::unique_ptr<AbstractRate>>([]() { return std::make_unique<MockRate>(); }),
    Supplier<std::unique_ptr<SettledUtil>>([]() { return createSettledUtilPtr(); }));
}

TimeUtil createTimeUtil(const Supplier<std::unique_ptr<SettledUtil>> &isettledUtilSupplier) {
  return TimeUtil(
    Supplier<std::unique_ptr<AbstractTimer>>([]() { return std::make_unique<MockTimer>(); }),
    Supplier<std::unique_ptr<AbstractRate>>([]() { return std::make_unique<MockRate>(); }),
    isettledUtilSupplier);
}

SimulatedSystem::SimulatedSystem(FlywheelSimulator &isimulator) : simulator(isimulator) {
}

SimulatedSystem::~SimulatedSystem() {
  dtorCalled.store(true, std::memory_order::memory_order_relaxed);
}

double SimulatedSystem::controllerGet() {
  return simulator.getAngle();
}

void SimulatedSystem::controllerSet(double ivalue) {
  simulator.setTorque(ivalue);
}

void SimulatedSystem::step() {
  while (!dtorCalled.load(std::memory_order::memory_order_relaxed)) {
    simulator.step();
    rate.delayUntil(10_ms);
  }
}

void SimulatedSystem::trampoline(void *system) {
  if (system) {
    static_cast<SimulatedSystem *>(system)->step();
  }
}

void SimulatedSystem::startThread() {
  thread = std::thread(trampoline, this);
}

void SimulatedSystem::join() {
  dtorCalled.store(true, std::memory_order::memory_order_relaxed);
  thread.join();
}

MockAsyncPosIntegratedController::MockAsyncPosIntegratedController()
  : AsyncPosIntegratedController(std::make_shared<MockMotor>(), createTimeUtil()) {
}

MockAsyncPosIntegratedController::MockAsyncPosIntegratedController(const TimeUtil &itimeUtil)
  : AsyncPosIntegratedController(std::make_shared<MockMotor>(), itimeUtil) {
}

bool MockAsyncPosIntegratedController::isSettled() {
  return isSettledOverride || AsyncPosIntegratedController::isSettled();
}

MockAsyncVelIntegratedController::MockAsyncVelIntegratedController()
  : AsyncVelIntegratedController(std::make_shared<MockMotor>(), createTimeUtil()) {
}

bool MockAsyncVelIntegratedController::isSettled() {
  return isSettledOverride || AsyncVelIntegratedController::isSettled();
}

void MockAsyncVelIntegratedController::setTarget(const double itarget) {
  lastTarget = itarget;

  if (itarget > maxTarget) {
    maxTarget = itarget;
  }

  AsyncVelIntegratedController::setTarget(itarget);
}

void MockAsyncVelIntegratedController::controllerSet(const double ivalue) {
  lastControllerOutputSet = ivalue;

  if (ivalue > maxControllerOutputSet) {
    maxControllerOutputSet = ivalue;
  }

  AsyncVelIntegratedController::controllerSet(ivalue);
}

MockIterativeController::MockIterativeController()
  : IterativePosPIDController(0, 0, 0, 0, createTimeUtil()) {
}

MockIterativeController::MockIterativeController(const double ikP)
  : IterativePosPIDController(ikP, 0, 0, 0, createTimeUtil()) {
}

bool MockIterativeController::isSettled() {
  return isSettledOverride || IterativePosPIDController::isSettled();
}

void assertMotorsHaveBeenStopped(MockMotor *leftMotor, MockMotor *rightMotor) {
  EXPECT_DOUBLE_EQ(leftMotor->lastVoltage, 0);
  EXPECT_DOUBLE_EQ(leftMotor->lastVelocity, 0);
  EXPECT_DOUBLE_EQ(rightMotor->lastVoltage, 0);
  EXPECT_DOUBLE_EQ(rightMotor->lastVelocity, 0);
}

void assertMotorsGearsetEquals(const AbstractMotor::gearset expected,
                               const std::initializer_list<MockMotor> &motors) {
  for (auto &motor : motors) {
    EXPECT_EQ(expected, motor.gearset);
  }
}

void assertMotorsBrakeModeEquals(const AbstractMotor::brakeMode expected,
                                 const std::initializer_list<MockMotor> &motors) {
  for (auto &motor : motors) {
    EXPECT_EQ(expected, motor.brakeMode);
  }
}

void assertMotorsEncoderUnitsEquals(const AbstractMotor::encoderUnits expected,
                                    const std::initializer_list<MockMotor> &motors) {
  for (auto &motor : motors) {
    EXPECT_EQ(expected, motor.encoderUnits);
  }
}

void assertAsyncControllerFollowsDisableLifecycle(AsyncController<double, double> &controller,
                                                  std::int16_t &domainValue,
                                                  std::int16_t &voltageValue,
                                                  int expectedOutput) {
  EXPECT_FALSE(controller.isDisabled()) << "Should not be disabled at the start.";

  controller.setTarget(100);
  EXPECT_EQ(domainValue, expectedOutput) << "Should be on by default.";

  controller.flipDisable();
  EXPECT_TRUE(controller.isDisabled()) << "Should be disabled after flipDisable";
  EXPECT_EQ(voltageValue, 0) << "Disabling the controller should turn the motor off";

  controller.flipDisable();
  EXPECT_FALSE(controller.isDisabled()) << "Should not be disabled after flipDisable";
  EXPECT_EQ(domainValue, expectedOutput)
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

void assertIterativeControllerFollowsDisableLifecycle(
  IterativeController<double, double> &controller) {
  EXPECT_FALSE(controller.isDisabled()) << "Should not be disabled at the start.";

  controller.setTarget(100);
  EXPECT_NE(controller.step(0), 0) << "Should be on by default.";
  EXPECT_NE(controller.getOutput(), 0);

  controller.flipDisable();
  EXPECT_TRUE(controller.isDisabled()) << "Should be disabled after flipDisable";
  // Run getOutput before step to check that it really does respect disabled
  EXPECT_EQ(controller.getOutput(), 0);
  EXPECT_EQ(controller.step(0), 0) << "Disabling the controller should give zero output";
  EXPECT_EQ(controller.getOutput(), 0);

  controller.flipDisable();
  EXPECT_FALSE(controller.isDisabled()) << "Should not be disabled after flipDisable";
  EXPECT_NE(controller.getOutput(), 0);
  EXPECT_NE(controller.step(0), 0);
  EXPECT_NE(controller.getOutput(), 0);

  controller.flipDisable();
  EXPECT_TRUE(controller.isDisabled()) << "Should be disabled after flipDisable";
  controller.reset();
  EXPECT_TRUE(controller.isDisabled()) << "Should be disabled after reset";
  EXPECT_EQ(controller.getOutput(), 0);
  EXPECT_EQ(controller.step(0), 0);
}

void assertControllerFollowsTargetLifecycle(ClosedLoopController<double, double> &controller) {
  EXPECT_DOUBLE_EQ(0, controller.getError()) << "Should start with 0 error";
  controller.setTarget(100);
  EXPECT_DOUBLE_EQ(controller.getError(), 100);
  controller.setTarget(0);
  EXPECT_DOUBLE_EQ(controller.getError(), 0);
}

void assertIterativeControllerScalesControllerSetTargets(
  IterativeController<double, double> &controller) {
  EXPECT_DOUBLE_EQ(controller.getTarget(), 0);
  controller.setOutputLimits(-100, 100);
  controller.controllerSet(0.5);
  EXPECT_DOUBLE_EQ(controller.getTarget(), 50);
}

void assertAsyncWrapperScalesControllerSetTargets(AsyncWrapper<double, double> &controller) {
  EXPECT_DOUBLE_EQ(controller.getTarget(), 0);
  controller.setOutputLimits(-100, 100);
  controller.controllerSet(0.5);
  EXPECT_DOUBLE_EQ(controller.getTarget(), 50);
}
} // namespace okapi
