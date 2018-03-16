/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#ifndef _OKAPI_ASYNCPOSITIONCONTROLLER_HPP_
#define _OKAPI_ASYNCPOSITIONCONTROLLER_HPP_

#include <memory>
#include "okapi/control/async/asyncController.hpp"

namespace okapi {
class AsyncPositionController;

class AsyncPositionControllerParams : public AsyncControllerParams {
public:
  /**
   * Constructs a new AsyncPositionController.
   *
   * @return shared_ptr to the AsyncPositionController
   */
  virtual std::shared_ptr<AsyncPositionController> make() const = 0;
};

class AsyncPositionController : public AsyncController {};
} // namespace okapi

#endif
