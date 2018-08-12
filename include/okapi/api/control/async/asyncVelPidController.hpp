/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_ASYNCVELPIDCONTROLLER_HPP_
#define _OKAPI_ASYNCVELPIDCONTROLLER_HPP_

#include "okapi/api/control/async/asyncVelocityController.hpp"
#include "okapi/api/control/async/asyncWrapper.hpp"
#include "okapi/api/control/controllerInput.hpp"
#include "okapi/api/control/controllerOutput.hpp"
#include "okapi/api/control/iterative/iterativeVelPidController.hpp"
#include "okapi/api/util/timeUtil.hpp"
#include <memory>

namespace okapi {
class AsyncVelPIDController : public AsyncWrapper<double, double>,
                              public AsyncVelocityController<double, double> {
  public:
  AsyncVelPIDController(
    std::shared_ptr<ControllerInput<double>> iinput,
    std::shared_ptr<ControllerOutput<double>> ioutput,
    const TimeUtil &itimeUtil,
    double ikP,
    double ikD,
    double ikF,
    double ikSF,
    std::unique_ptr<VelMath> ivelMath,
    std::unique_ptr<Filter> iderivativeFilter = std::make_unique<PassthroughFilter>());
};
} // namespace okapi

#endif
