/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_ASYNCPOSPIDCONTROLLER_HPP_
#define _OKAPI_ASYNCPOSPIDCONTROLLER_HPP_

#include "api.h"
#include "okapi/control/async/asyncPositionController.hpp"
#include "okapi/control/async/asyncWrapper.hpp"
#include "okapi/control/controllerInput.hpp"
#include "okapi/control/controllerOutput.hpp"
#include "okapi/control/iterative/iterativePosPidController.hpp"
#include <memory>

namespace okapi {
class AsyncPosPIDControllerArgs : public AsyncPositionControllerArgs {
  public:
  AsyncPosPIDControllerArgs(std::shared_ptr<ControllerInput> iinput,
                            std::shared_ptr<ControllerOutput> ioutput,
                            const IterativePosPIDControllerArgs &iparams);

  std::shared_ptr<ControllerInput> input;
  std::shared_ptr<ControllerOutput> output;
  const IterativePosPIDControllerArgs params;
};

class AsyncPosPIDController : public AsyncWrapper, public AsyncPositionController {
  public:
  AsyncPosPIDController(std::shared_ptr<ControllerInput> iinput,
                        std::shared_ptr<ControllerOutput> ioutput, const double ikP,
                        const double ikI, const double ikD, const double ikBias = 0);

  AsyncPosPIDController(std::shared_ptr<ControllerInput> iinput,
                        std::shared_ptr<ControllerOutput> ioutput,
                        const IterativePosPIDControllerArgs &iparams);
};
} // namespace okapi

#endif
