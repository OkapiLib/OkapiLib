/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_ASYNCPOSPIDCONTROLLER_HPP_
#define _OKAPI_ASYNCPOSPIDCONTROLLER_HPP_

#include "okapi/api/control/async/asyncPositionController.hpp"
#include "okapi/api/control/async/asyncWrapper.hpp"
#include "okapi/api/control/controllerInput.hpp"
#include "okapi/api/control/controllerOutput.hpp"
#include "okapi/api/control/iterative/iterativePosPidController.hpp"
#include "okapi/api/util/timeUtil.hpp"
#include <memory>

namespace okapi {
class AsyncPosPIDController : public AsyncWrapper<double, double>,
                              public AsyncPositionController<double, double> {
  public:
  AsyncPosPIDController(
    std::shared_ptr<ControllerInput<double>> iinput,
    std::shared_ptr<ControllerOutput<double>> ioutput,
    const TimeUtil &itimeUtil,
    double ikP,
    double ikI,
    double ikD,
    double ikBias = 0,
    std::unique_ptr<Filter> iderivativeFilter = std::make_unique<PassthroughFilter>());
};
} // namespace okapi

#endif
