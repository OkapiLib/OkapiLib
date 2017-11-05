#ifndef OKAPI_CHASSISMODEL
#define OKAPI_CHASSISMODEL

#include <array>
#include <initializer_list>
#include <valarray>
#include <API.h>
#include <memory>
#include "device/motor.h"
#include "device/quadEncoder.h"

namespace okapi {
  class ChassisModel {
  public:
    ChassisModel() {}
    virtual ~ChassisModel() = default;
    virtual void driveForward(const int power) = 0;
    virtual void driveVector(const int distPower, const int anglePower) = 0;
    virtual void turnClockwise(const int power) = 0;
    virtual void stop() = 0;
    virtual void tank(const int leftVal, const int rightVal, const int threshold = 0) = 0;
    virtual void arcade(int verticalVal, int horizontalVal, const int threshold = 0) = 0;
    virtual void left(const int val) = 0;
    virtual void leftTS(const int val) = 0;
    virtual void right(const int val) = 0;
    virtual void rightTS(const int val) = 0;
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
  class SkidSteerModelParams : public ChassisModelParams {
  public:
    SkidSteerModelParams(const std::array<Motor, motorsPerSide * 2>& imotorList, QuadEncoder ileftEnc, QuadEncoder irightEnc):
      motorList(imotorList),
      leftEnc(ileftEnc),
      rightEnc(irightEnc) {}

    virtual ~SkidSteerModelParams() = default;

    std::shared_ptr<ChassisModel> make() const override {
      return std::make_shared<SkidSteerModel<motorsPerSide>>(*this);
    }

    const std::array<Motor, motorsPerSide * 2>& motorList;
    QuadEncoder leftEnc, rightEnc;
  };

  template<size_t motorsPerSide>
  class SkidSteerModel : public ChassisModel {
  public:
    /**
     * Model for a skid steer drive (wheels parallel with robot's direction of
     * motion). When all motors are powered +127, the robot should move forward
     * in a straight line at full speed.
     * @param imotors   Motors in the format: {{left side motors}, {right side motors}}
     * @param ileftEnc  Left side encoder
     * @param irightEnc Right side encoder
     */
    SkidSteerModel(const std::array<Motor, motorsPerSide * 2>& imotorList, const QuadEncoder ileftEnc, const QuadEncoder irightEnc):
      motors(imotorList),
      leftEnc(ileftEnc),
      rightEnc(irightEnc) {}

    SkidSteerModel(const SkidSteerModelParams<motorsPerSide>& iparams):
      motors(iparams.motorList),
      leftEnc(iparams.leftEnc),
      rightEnc(iparams.rightEnc) {}

    SkidSteerModel(const SkidSteerModel<motorsPerSide>& other):
      motors(other.motors),
      leftEnc(other.leftEnc),
      rightEnc(other.rightEnc) {}

    virtual ~SkidSteerModel() { delete &motors; }

    void driveForward(const int power) override {
      for (size_t i = 0; i <= motorsPerSide * 2; i++)
        motors[i].setTS(power);
    }

    void driveVector(const int distPower, const int anglePower) override {
      for (size_t i = 0; i < motorsPerSide; i++)
        motors[i].setTS(distPower + anglePower);
      for (size_t i = motorsPerSide; i < motorsPerSide * 2; i++)
        motors[i].setTS(distPower - anglePower);
    }

    void turnClockwise(const int power) override {
      for (size_t i = 0; i < motorsPerSide; i++)
        motors[i].setTS(power);
      for (size_t i = motorsPerSide; i < motorsPerSide * 2; i++)
        motors[i].setTS(-1 * power);
    }

    void stop() override {
      for (size_t i = 0; i <= motorsPerSide * 2; i++)
        motors[i].set(0);
    }

    void tank(const int leftVal, const int rightVal, const int threshold = 0) override {
      using namespace std;

      if (fabs(leftVal) < threshold) {
        for (size_t i = 0; i < motorsPerSide; i++)
          motors[i].set(0);
      } else {
        for (size_t i = 0; i < motorsPerSide; i++)
          motors[i].set(leftVal);
      }
      
      if (fabs(rightVal) < threshold) {
        for (size_t i = motorsPerSide; i < motorsPerSide * 2; i++)
          motors[i].set(0);
      } else {
        for (size_t i = motorsPerSide; i < motorsPerSide * 2; i++)
          motors[i].set(rightVal);
      }
    }

    void arcade(int verticalVal, int horizontalVal, const int threshold = 0) override {
      using namespace std;
      
      if (fabs(verticalVal) < threshold)
        verticalVal = 0;
      if (fabs(horizontalVal) < threshold)
        horizontalVal = 0;
      
      for (size_t i = 0; i < motorsPerSide; i++)
        motors[i].set(verticalVal + horizontalVal);
      for (size_t i = motorsPerSide; i < motorsPerSide * 2; i++)
        motors[i].set(verticalVal - horizontalVal);
    }

    void left(const int val) override {
      for (size_t i = 0; i < motorsPerSide; i++)
        motors[i].set(val);
    }
    
    void leftTS(const int val) override {
      for (size_t i = 0; i < motorsPerSide; i++)
        motors[i].setTS(val);
    }

    void right(const int val) override {
      for (size_t i = motorsPerSide; i < motorsPerSide * 2; i++)
        motors[i].set(val);
    }
    
    void rightTS(const int val) override {
      for (size_t i = motorsPerSide; i < motorsPerSide * 2; i++)
        motors[i].setTS(val);
    }

    std::valarray<int> getEncoderVals() const override {
      return std::valarray<int>{leftEnc.get(), rightEnc.get()};
    }
  private:
    const std::array<Motor, motorsPerSide * 2> motors;
    const QuadEncoder leftEnc, rightEnc;
  };

  template<size_t motorsPerCorner>
  class XDriveModel;

  template<size_t motorsPerCorner>
  class XDriveModelParams : public ChassisModelParams {
  public:
    XDriveModelParams(const std::array<unsigned char, motorsPerCorner * 4>& imotorList, const QuadEncoder ileftEnc, const QuadEncoder irightEnc):
      motorList(imotorList),
      leftEnc(ileftEnc),
      rightEnc(irightEnc) {}

    virtual ~XDriveModelParams() {}

    std::shared_ptr<ChassisModel> make() const override {
      return std::make_shared<XDriveModel<motorsPerCorner>>(*this);
    }

    const std::array<unsigned char, motorsPerCorner * 4>& motorList;
    const QuadEncoder leftEnc, rightEnc;
  };

  template<size_t motorsPerCorner>
  class XDriveModel : public ChassisModel {
  public:
    /**
     * Model for an x drive (wheels at 45 deg from a skid steer drive). When all
     * motors are powered +127, the robot should move forward in a straight line
     * at full speed.
     * @param imotors Motors in the format: {{top left motors}, {top right motors}, {bottom right motors}, {bottom left motors}}
     */
    XDriveModel(const std::array<unsigned char, motorsPerCorner * 4>& imotorList, const QuadEncoder ileftEnc, const QuadEncoder irightEnc):
      motors(imotorList),
      leftEnc(ileftEnc),
      rightEnc(irightEnc) {}

    XDriveModel(const XDriveModelParams<motorsPerCorner>& iparams):
      motors(iparams.motorList),
      leftEnc(iparams.leftEnc),
      rightEnc(iparams.rightEnc) {}

    XDriveModel(const XDriveModel<motorsPerCorner>& other):
      motors(other.motors),
      leftEnc(other.leftEnc),
      rightEnc(other.rightEnc) {}

    virtual ~XDriveModel() { delete &motors; }

    void driveForward(const int power) override {
      for (size_t i = 0; i < motorsPerCorner * 4; i++)
        motors[i].setTS(power);
    }

    void driveVector(const int distPower, const int anglePower) override {
      for (size_t i = 0; i < motorsPerCorner; i++)
        motors[i].setTS(distPower + anglePower);
      for (size_t i = motorsPerCorner; i < motorsPerCorner * 2; i++)
        motors[i].setTS(distPower - anglePower);
      for (size_t i = motorsPerCorner * 2; i < motorsPerCorner * 3; i++)
        motors[i].setTS(distPower - anglePower);
      for (size_t i = motorsPerCorner * 3; i < motorsPerCorner * 4; i++)
        motors[i].setTS(distPower + anglePower);
    }

    void turnClockwise(const int power) override {
      for (size_t i = 0; i < motorsPerCorner; i++)
        motors[i].setTS(power);
      for (size_t i = motorsPerCorner; i < motorsPerCorner * 2; i++)
        motors[i].setTS(-1 * power);
      for (size_t i = motorsPerCorner * 2; i < motorsPerCorner * 3; i++)
        motors[i].setTS(-1 * power);
      for (size_t i = motorsPerCorner * 3; i < motorsPerCorner * 4; i++)
        motors[i].setTS(power);
    }

    void stop() override {
      for (size_t i = 0; i < motorsPerCorner * 4; i++)
        motors[i].set(0);
    }

    void tank(const int leftVal, const int rightVal, const int threshold = 0) override {
      using namespace std;

      if (fabs(leftVal) < threshold) {
        for (size_t i = 0; i < motorsPerCorner; i++)
          motors[i].set(0);
        for (size_t i = motorsPerCorner * 3; i < motorsPerCorner * 4; i++)
          motors[i].set(0);
      } else {
        for (size_t i = 0; i < motorsPerCorner; i++)
          motors[i].set(leftVal);
        for (size_t i = motorsPerCorner * 3; i < motorsPerCorner * 4; i++)
          motors[i].set(leftVal);
      }
      
      if (fabs(rightVal) < threshold) {
        for (size_t i = motorsPerCorner; i < motorsPerCorner * 2; i++)
          motors[i].set(0);
        for (size_t i = motorsPerCorner * 2; i < motorsPerCorner * 3; i++)
          motors[i].set(0);
      } else {
        for (size_t i = motorsPerCorner; i < motorsPerCorner * 2; i++)
          motors[i].set(rightVal);
        for (size_t i = motorsPerCorner * 2; i < motorsPerCorner * 3; i++)
          motors[i].set(rightVal);
      }
    }

    void arcade(int verticalVal, int horizontalVal, const int threshold = 0) override {
      using namespace std;
      
      if (fabs(verticalVal) < threshold)
        verticalVal = 0;
      if (fabs(horizontalVal) < threshold)
        horizontalVal = 0;
      
      for (size_t i = 0; i < motorsPerCorner; i++)
        motors[i].set(verticalVal + horizontalVal);
      for (size_t i = motorsPerCorner; i < motorsPerCorner * 2; i++)
        motors[i].set(verticalVal - horizontalVal);
      for (size_t i = motorsPerCorner * 2; i < motorsPerCorner * 3; i++)
        motors[i].set(verticalVal - horizontalVal);
      for (size_t i = motorsPerCorner * 3; i < motorsPerCorner * 4; i++)
        motors[i].set(verticalVal + horizontalVal);
    }

    void xArcade(int verticalVal, int horizontalVal, int rotateVal, const int threshold = 0) {
      using namespace std;
      
      if (fabs(verticalVal) < threshold)
        verticalVal = 0;
      if (fabs(horizontalVal) < threshold)
        horizontalVal = 0;
      if (fabs(rotateVal) < threshold)
        rotateVal = 0;
      
      for (size_t i = 0; i < motorsPerCorner; i++)
        motors[i].set(verticalVal + horizontalVal + rotateVal);
      for (size_t i = motorsPerCorner; i < motorsPerCorner * 2; i++)
        motors[i].set(verticalVal - horizontalVal - rotateVal);
      for (size_t i = motorsPerCorner * 2; i < motorsPerCorner * 3; i++)
        motors[i].set(verticalVal + horizontalVal - rotateVal);
      for (size_t i = motorsPerCorner * 3; i < motorsPerCorner * 4; i++)
        motors[i].set(verticalVal - horizontalVal + rotateVal);
    }
    
    void left(const int val) override {
      for (size_t i = 0; i < motorsPerCorner; i++)
        motors[i].set(val);
      for (size_t i = motorsPerCorner * 3; i < motorsPerCorner * 4; i++)
        motors[i].set(val);
    }
    
    void leftTS(const int val) override {
      for (size_t i = 0; i < motorsPerCorner; i++)
        motors[i].setTS(val);
      for (size_t i = motorsPerCorner * 3; i < motorsPerCorner * 4; i++)
        motors[i].setTS(val);
    }

    void right(const int val) override {
      for (size_t i = motorsPerCorner; i < motorsPerCorner * 3; i++)
        motors[i].set(val);
    }
    
    void rightTS(const int val) override {
      for (size_t i = motorsPerCorner; i < motorsPerCorner * 3; i++)
        motors[i].setTS(val);
    }

    std::valarray<int> getEncoderVals() const override {
      return std::valarray<int>{leftEnc.get(), rightEnc.get()};
    }
  private:
    const std::array<unsigned char, motorsPerCorner * 4> motors;
    const QuadEncoder leftEnc, rightEnc;
  };
}

#endif /* end of include guard: OKAPI_CHASSISMODEL */
