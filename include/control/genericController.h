#ifndef OKAPI_GENERICCONTROLLER
#define OKAPI_GENERICCONTROLLER

#include <memory>
#include <array>
#include "control/controlObject.h"
#include "control/motor.h"

namespace okapi {
  template<size_t motorNum>
  class GenericController {
  public:
    GenericController(const std::initializer_list<Motor> &imotorList, const std::shared_ptr<ControlObject> &iptr):
      controller(iptr) {
        for (size_t i = 0; i < imotorList.size(); i++)
        motors[i] = *(imotorList.begin() + i);
      }

    void loop(const float ireading) {
      controller->loop(ireading);
      for (size_t i = 0; i < motors.size(); i++)
        motors[i].setTS(controller->getOutput());
    }
  private:
    std::shared_ptr<ControlObject> controller;
    std::array<Motor, motorNum> motors;
  };
}

#endif /* OKAPI_GENERICCONTROLLER */