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
#include "okapi/api/control/controllerInput.hpp"
#include "okapi/api/control/controllerOutput.hpp"
#include "okapi/impl/control/async/asyncWrapper.hpp"
#include "okapi/impl/control/iterative/iterativePosPidController.hpp"
#include <memory>

namespace okapi {
class AsyncPosPIDControllerArgs : public AsyncPositionControllerArgs {
  public:
  AsyncPosPIDControllerArgs(std::shared_ptr<ControllerInput> iinput,
                            std::shared_ptr<ControllerOutput> ioutput,
                            IterativePosPIDControllerArgs &iparams);

  std::shared_ptr<ControllerInput> input;
  std::shared_ptr<ControllerOutput> output;
  const IterativePosPIDControllerArgs params;
};

class AsyncPosPIDController : public AsyncWrapper, public AsyncPositionController {
  public:
  AsyncPosPIDController(std::shared_ptr<ControllerInput> iinput,
                        std::shared_ptr<ControllerOutput> ioutput, double ikP, double ikI,
                        double ikD, double ikBias = 0);

  AsyncPosPIDController(std::shared_ptr<ControllerInput> iinput,
                        std::shared_ptr<ControllerOutput> ioutput,
                        IterativePosPIDControllerArgs &iparams);
};
} // namespace okapi

#endif
