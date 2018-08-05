/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_CONTROLLERRUNNERFACTORY_HPP_
#define _OKAPI_CONTROLLERRUNNERFACTORY_HPP_

#include "okapi/api/control/util/controllerRunner.hpp"
#include "okapi/impl/util/rate.hpp"

namespace okapi {
template <typename Input, typename Output> class ControllerRunnerFactory {
  public:
  static ControllerRunner<Input, Output> create() {
    return ControllerRunner<Input, Output>(std::make_unique<Rate>());
  }
};
} // namespace okapi

#endif
