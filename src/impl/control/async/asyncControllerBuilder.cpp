/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/impl/control/async/asyncControllerBuilder.hpp"
#include "okapi/api/control/iterative/iterativePosPidController.hpp"
#include "okapi/api/control/iterative/iterativeVelPidController.hpp"
#include "okapi/api/filter/filteredControllerInput.hpp"
#include "okapi/api/filter/passthroughFilter.hpp"
#include <numeric>

namespace okapi {
AsyncControllerBuilder::AsyncControllerBuilder(const TimeUtil &itimeUtil) : timeUtil(itimeUtil) {
}

AsyncControllerBuilder::~AsyncControllerBuilder() {
  printf("AsyncControllerBuilder dtor!\n");
}

// //////////////////////////////////////////////////////
//                                                     //
//                        INPUT                        //
//                                                     //
// //////////////////////////////////////////////////////

AsyncControllerBuilder &AsyncControllerBuilder::input(ADIEncoder iencoder) {
  m_input = std::make_shared<ADIEncoder>(iencoder);
  return *this;
}

AsyncControllerBuilder &AsyncControllerBuilder::input(IntegratedEncoder iencoder) {
  m_input = std::make_shared<IntegratedEncoder>(iencoder);
  return *this;
}

// AsyncControllerBuilder &AsyncControllerBuilder::input(Motor imotor) {
//  m_input = imotor.getEncoder();
//  return *this;
//}

AsyncControllerBuilder &AsyncControllerBuilder::input(MotorGroup imotor) {
  m_input = imotor.getEncoder();
  return *this;
}

AsyncControllerBuilder &AsyncControllerBuilder::input(Potentiometer ipotentiometer) {
  m_input = std::make_shared<Potentiometer>(ipotentiometer);
  return *this;
}

AsyncControllerBuilder &AsyncControllerBuilder::input(ADIUltrasonic iultrasonic) {
  m_input = std::make_shared<ADIUltrasonic>(iultrasonic);
  return *this;
}

// //////////////////////////////////////////////////////
//                                                     //
//                        FILTER                       //
//                                                     //
// //////////////////////////////////////////////////////

AsyncControllerBuilder &AsyncControllerBuilder::filter(EmaFilter ifilter) {
  m_filters.push_back(std::make_shared<EmaFilter>(ifilter));
  return *this;
}

AsyncControllerBuilder &AsyncControllerBuilder::filter(DemaFilter ifilter) {
  m_filters.push_back(std::make_shared<DemaFilter>(ifilter));
  return *this;
}

AsyncControllerBuilder &AsyncControllerBuilder::filter(ComposableFilter ifilter) {
  m_filters.push_back(std::make_shared<ComposableFilter>(ifilter));
  return *this;
}

AsyncControllerBuilder &AsyncControllerBuilder::filter(std::shared_ptr<Filter> ifilter) {
  m_filters.push_back(ifilter);
  return *this;
}

// //////////////////////////////////////////////////////
//                                                     //
//                     CONTROLLERS                     //
//                                                     //
// //////////////////////////////////////////////////////

AsyncControllerBuilder &AsyncControllerBuilder::posPid(const double ikP,
                                                       const double ikI,
                                                       const double ikD,
                                                       const double ikBias) {
  m_controllers.emplace_back(
    std::make_shared<IterativePosPIDController>(ikP, ikI, ikD, ikBias, timeUtil));
  return *this;
}

AsyncControllerBuilder &AsyncControllerBuilder::velPid(const double ikP,
                                                       const double ikD,
                                                       const double ikF,
                                                       std::unique_ptr<VelMath> ivelMath) {
  m_controllers.emplace_back(
    std::make_shared<IterativeVelPIDController>(ikP, ikD, ikF, std::move(ivelMath), timeUtil));
  return *this;
}

AsyncControllerBuilder &
AsyncControllerBuilder::lambda(std::function<double(double)> istepFunction) {
  m_controllers.emplace_back(
    std::make_shared<IterativeLambdaBasedController>(istepFunction, timeUtil));
  return *this;
}

// //////////////////////////////////////////////////////
//                                                     //
//                        OUTPUT                       //
//                                                     //
// //////////////////////////////////////////////////////

AsyncControllerBuilder &AsyncControllerBuilder::output(Motor imotor) {
  m_output = std::make_shared<Motor>(imotor);
  return *this;
}

AsyncControllerBuilder &AsyncControllerBuilder::output(MotorGroup imotor) {
  m_output = std::make_shared<MotorGroup>(imotor);
  return *this;
}

AsyncControllerBuilder &AsyncControllerBuilder::output(std::shared_ptr<AbstractMotor> imotor) {
  m_output = imotor;
  return *this;
}

AsyncWrapper<double, double> AsyncControllerBuilder::build() const {
  std::shared_ptr<Filter> outFilter;
  if (m_filters.empty()) {
    outFilter = std::make_shared<PassthroughFilter>();
  } else {
    outFilter = std::make_shared<ComposableFilter>(m_filters);
  }

  struct WrappedIterativeController {
    std::shared_ptr<ControllerInput<double>> m_input;
    std::shared_ptr<Filter> outFilter;
    std::vector<std::shared_ptr<IterativeController<double, double>>> m_controllers;
    std::shared_ptr<ControllerOutput<double>> m_output;

    WrappedIterativeController(
      std::shared_ptr<ControllerInput<double>> input,
      std::vector<std::shared_ptr<Filter>> filters,
      std::vector<std::shared_ptr<IterativeController<double, double>>> controllers,
      std::shared_ptr<ControllerOutput<double>> output)
      : m_input(input), m_controllers(controllers), m_output(output) {
      std::shared_ptr<Filter> outFilter;
      if (filters.empty()) {
        outFilter = std::make_shared<PassthroughFilter>();
      } else {
        outFilter = std::make_shared<ComposableFilter>(filters);
      }
    }

    virtual ~WrappedIterativeController() {
      printf("WrappedIterativeController dtor!");
    }

    double operator()(const double error) {
      // return std::accumulate(std::next(m_controllers.begin()), m_controllers.end(),
      //                        m_controllers.front()->step(outFilter->filter(error)),
      //                        [](double prevOutput, auto &cnt) {
      //                          printf("prevOut %1.2f\n", prevOutput);
      //                          return cnt->step(prevOutput);
      //                        });
      printf("step");
      return 0;
    }
  };

  struct WrappedAsyncController : public AsyncWrapper<double, double> {
    std::shared_ptr<ControllerInput<double>> m_input;
    std::shared_ptr<ControllerOutput<double>> m_output;

    WrappedAsyncController(std::shared_ptr<ControllerInput<double>> input,
                           std::shared_ptr<ControllerOutput<double>> output,
                           std::unique_ptr<IterativeLambdaBasedController> controller,
                           const TimeUtil &timeUtil)
      : AsyncWrapper<double, double>(input,
                                     output,
                                     std::move(controller),
                                     timeUtil.getRateSupplier(),
                                     timeUtil.getSettledUtil()),
        m_input(input),
        m_output(output) {
    }

    ~WrappedAsyncController() override {
      printf("WrappedAsyncController dtor!");
    }
  };

  return WrappedAsyncController(
    m_input,
    m_output,
    std::make_unique<IterativeLambdaBasedController>(
      WrappedIterativeController(m_input, m_filters, m_controllers, m_output), timeUtil),
    timeUtil);
}
} // namespace okapi
