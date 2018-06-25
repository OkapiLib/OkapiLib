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
#include "okapi/api/control/controllerInput.hpp"
#include "okapi/api/control/controllerOutput.hpp"
#include "okapi/impl/control/async/asyncWrapper.hpp"
#include "okapi/impl/control/iterative/iterativeVelPidController.hpp"
#include <memory>

namespace okapi {
class AsyncVelPIDControllerArgs : public AsyncVelocityControllerArgs {
  public:
  AsyncVelPIDControllerArgs(std::shared_ptr<ControllerInput> iinput,
                            std::shared_ptr<ControllerOutput> ioutput,
                            IterativeVelPIDControllerArgs &iparams);

  std::shared_ptr<ControllerInput> input;
  std::shared_ptr<ControllerOutput> output;
  const IterativeVelPIDControllerArgs params;
};

class AsyncVelPIDController : public AsyncWrapper, public AsyncVelocityController {
  public:
  AsyncVelPIDController(std::shared_ptr<ControllerInput> iinput,
                        std::shared_ptr<ControllerOutput> ioutput, double ikP, double ikD,
                        double ikF);

  AsyncVelPIDController(std::shared_ptr<ControllerInput> iinput,
                        std::shared_ptr<ControllerOutput> ioutput,
                        IterativeVelPIDControllerArgs &iparams);
};
} // namespace okapi

#endif
