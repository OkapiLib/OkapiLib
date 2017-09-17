#ifndef OKAPI_CHASSISMODEL
#define OKAPI_CHASSISMODEL

#include <array>
#include <initializer_list>
#include <valarray>
#include <API.h>
#include <memory>
#include "control/motor.h"

namespace okapi {
  class ChassisModel {
  public:
    ChassisModel() {}
    virtual ~ChassisModel() = default;
    virtual void driveForward(const int power) = 0;
    virtual void driveVector(const int distPower, const int anglePower) = 0;
    virtual void turnClockwise(const int power) = 0;
    virtual void tank(const int leftVal, const int rightVal) = 0;
    virtual void arcade(const int verticalVal, const int horizontalVal) = 0;
    virtual std::valarray<int> getEncoderVals() const = 0;
  };

  class ChassisModelParams {
  public:
    ChassisModelParams() {}
    virtual ~ChassisModelParams() = default;

    /**
     * Allocates a new ChassisModel
     * @return shared_ptr pointing to new ChassisModel
     */
    virtual std::shared_ptr<ChassisModel> make() const = 0;
  };

  template<size_t motorsPerSide>
  class SkidSteerModel;

  template<size_t motorsPerSide>
  class SkidSteerModelParams final : public ChassisModelParams {
  public:
    SkidSteerModelParams(const std::initializer_list<Motor>& imotorList, Encoder ileftEnc, Encoder irightEnc):
      motorList(imotorList),
      leftEnc(ileftEnc),
      rightEnc(irightEnc) {}

    virtual ~SkidSteerModelParams() = default;

    std::shared_ptr<ChassisModel> make() const override {
      return std::make_shared<SkidSteerModel<motorsPerSide>>(*this);
    }

    const std::initializer_list<Motor>& motorList;
    Encoder leftEnc, rightEnc;
  };

  template<size_t motorsPerSide>
  class SkidSteerModel final : public ChassisModel {
  public:
    /**
     * Model for a skid steer drive (wheels parallel with robot's direction of
     * motion). When all motors are powered +127, the robot should move forward
     * in a straight line at full speed.
     * @param imotors   Motors in the format: {{left side motors}, {right side motors}}
     * @param ileftEnc  Left side encoder
     * @param irightEnc Right side encoder
     */
    SkidSteerModel(const std::initializer_list<Motor>& imotorList, Encoder ileftEnc, Encoder irightEnc):
      leftEnc(ileftEnc),
      rightEnc(irightEnc) {
        for (size_t i = 0; i < imotorList.size(); i++)
          motors[i] = *(imotorList.begin() + i);
    }

    SkidSteerModel(const SkidSteerModelParams<motorsPerSide>& iparams):
      leftEnc(iparams.leftEnc),
      rightEnc(iparams.rightEnc) {
        for (size_t i = 0; i < iparams.motorList.size(); i++)
          motors[i] = *(iparams.motorList.begin() + i);
    }

    SkidSteerModel(const SkidSteerModel<motorsPerSide>& other):
      motors(other.motors),
      leftEnc(other.leftEnc),
      rightEnc(other.rightEnc) {}

    virtual ~SkidSteerModel() {
      delete &motors;
    }

    void driveForward(const int power) override {
      for (size_t i = 0; i < motorsPerSide * 2; i++)
        // motorSet(motors[i], power);
        motors[i].set(power);
    }

    void driveVector(const int distPower, const int anglePower) override {
      for (size_t i = 0; i < motorsPerSide; i++)
        // motorSet(motors[i], distPower + anglePower);
        motors[i].set(distPower + anglePower);
      for (size_t i = motorsPerSide; i < motorsPerSide * 2; i++)
        // motorSet(motors[i], distPower - anglePower);
        motors[i].set(distPower - anglePower);
    }

    void turnClockwise(const int power) override {
      for (size_t i = 0; i < motorsPerSide; i++)
        // motorSet(motors[i], power);
        motors[i].set(power);
      for (size_t i = motorsPerSide; i < motorsPerSide * 2; i++)
        // motorSet(motors[i], -1 * power);
        motors[i].set(-1 * power);
    }

    void tank(const int leftVal, const int rightVal) override {
      for (size_t i = 0; i < motorsPerSide; i++)
        // motorSet(motors[i], leftVal);
        motors[i].setTS(leftVal);
      for (size_t i = motorsPerSide; i < motorsPerSide * 2; i++)
        // motorSet(motors[i], rightVal);
        motors[i].setTS(rightVal);
    }

    void arcade(const int verticalVal, const int horizontalVal) override {
      for (size_t i = 0; i < motorsPerSide; i++)
        // motorSet(motors[i], verticalVal + horizontalVal);
        motors[i].setTS(verticalVal + horizontalVal);
      for (size_t i = motorsPerSide; i < motorsPerSide * 2; i++)
        // motorSet(motors[i], verticalVal - horizontalVal);
        motors[i].setTS(verticalVal - horizontalVal);
    }

    std::valarray<int> getEncoderVals() const override {
      return std::valarray<int>{encoderGet(leftEnc), encoderGet(rightEnc)};
    }
  private:
    std::array<Motor, motorsPerSide * 2> motors;
    Encoder leftEnc, rightEnc;
  };

  template<size_t motorsPerCorner>
  class XDriveModel;

  template<size_t motorsPerCorner>
  class XDriveModelParams final : public ChassisModelParams {
  public:
    XDriveModelParams(const std::initializer_list<unsigned char>& imotorList, Encoder ileftEnc, Encoder irightEnc):
      motorList(imotorList),
      leftEnc(ileftEnc),
      rightEnc(irightEnc) {}

    virtual ~XDriveModelParams() {}

    std::shared_ptr<ChassisModel> make() const override {
      return std::make_shared<XDriveModel<motorsPerCorner>>(*this);
    }

    const std::initializer_list<unsigned char>& motorList;
    Encoder leftEnc, rightEnc;
  };

  template<size_t motorsPerCorner>
  class XDriveModel final : public ChassisModel {
  public:
    /**
     * Model for an x drive (wheels at 45 deg from a skid steer drive). When all
     * motors are powered +127, the robot should move forward in a straight line
     * at full speed.
     * @param imotors Motors in the format: {{top left motors}, {top right motors}, {bottom right motors}, {bottom left motors}}
     */
    XDriveModel(const std::initializer_list<unsigned char>& imotorList, Encoder ileftEnc, Encoder irightEnc):
      leftEnc(ileftEnc),
      rightEnc(irightEnc) {
        for (size_t i = 0; i < imotorList.size(); i++)
          motors[i] = *(imotorList.begin() + i);
    }

    XDriveModel(const XDriveModelParams<motorsPerCorner>& iparams):
      leftEnc(iparams.leftEnc),
      rightEnc(iparams.rightEnc) {
        for (size_t i = 0; i < iparams.motorList.size(); i++)
          motors[i] = *(iparams.motorList.begin() + i);
    }

    XDriveModel(const XDriveModel<motorsPerCorner>& other):
      motors(other.motors),
      leftEnc(other.leftEnc),
      rightEnc(other.rightEnc) {}

    virtual ~XDriveModel() {
      delete &motors;
    }

    void driveForward(const int power) override {
      for (size_t i = 0; i < motorsPerCorner; i++)
        motorSet(motors[i], power);
    }

    void driveVector(const int distPower, const int anglePower) override {
      for (size_t i = 0; i < motorsPerCorner; i++)
        motorSet(motors[i], distPower + anglePower);
      for (size_t i = motorsPerCorner; i < motorsPerCorner * 2; i++)
        motorSet(motors[i], distPower - anglePower);
      for (size_t i = motorsPerCorner * 2; i < motorsPerCorner * 3; i++)
        motorSet(motors[i], distPower - anglePower);
      for (size_t i = motorsPerCorner * 3; i < motorsPerCorner * 4; i++)
        motorSet(motors[i], distPower + anglePower);
    }

    void turnClockwise(const int power) override {
      for (size_t i = 0; i < motorsPerCorner; i++)
        motorSet(motors[i], power);
      for (size_t i = motorsPerCorner; i < motorsPerCorner * 2; i++)
        motorSet(motors[i], -1 * power);
      for (size_t i = motorsPerCorner * 2; i < motorsPerCorner * 3; i++)
        motorSet(motors[i], -1 * power);
      for (size_t i = motorsPerCorner * 3; i < motorsPerCorner * 4; i++)
        motorSet(motors[i], power);
    }

    void tank(const int leftVal, const int rightVal) override {
      for (size_t i = 0; i < motorsPerCorner; i++)
        motorSet(motors[i], leftVal);
      for (size_t i = motorsPerCorner; i < motorsPerCorner * 2; i++)
        motorSet(motors[i], rightVal);
      for (size_t i = motorsPerCorner * 2; i < motorsPerCorner * 3; i++)
        motorSet(motors[i], rightVal);
      for (size_t i = motorsPerCorner * 3; i < motorsPerCorner * 4; i++)
        motorSet(motors[i], leftVal);
    }

    void arcade(const int verticalVal, const int horizontalVal) override {
      for (size_t i = 0; i < motorsPerCorner; i++)
        motorSet(motors[i], verticalVal + horizontalVal);
      for (size_t i = motorsPerCorner; i < motorsPerCorner * 2; i++)
        motorSet(motors[i], verticalVal - horizontalVal);
      for (size_t i = motorsPerCorner * 2; i < motorsPerCorner * 3; i++)
        motorSet(motors[i], verticalVal - horizontalVal);
      for (size_t i = motorsPerCorner * 3; i < motorsPerCorner * 4; i++)
        motorSet(motors[i], verticalVal + horizontalVal);
    }

    void xArcade(const int verticalVal, const int horizontalVal, const int rotateVal) {
        for (size_t i = 0; i < motorsPerCorner; i++)
          motorSet(motors[i], verticalVal + horizontalVal + rotateVal);
        for (size_t i = motorsPerCorner; i < motorsPerCorner * 2; i++)
          motorSet(motors[i], verticalVal - horizontalVal - rotateVal);
        for (size_t i = motorsPerCorner * 2; i < motorsPerCorner * 3; i++)
          motorSet(motors[i], verticalVal + horizontalVal - rotateVal);
        for (size_t i = motorsPerCorner * 3; i < motorsPerCorner * 4; i++)
          motorSet(motors[i], verticalVal - horizontalVal + rotateVal);
    }

    std::valarray<int> getEncoderVals() const override {
      return std::valarray<int>{encoderGet(leftEnc), encoderGet(rightEnc)};
    }
  private:
    std::array<unsigned char, motorsPerCorner * 4> motors;
    Encoder leftEnc, rightEnc;
  };
}

#endif /* end of include guard: OKAPI_CHASSISMODEL */
