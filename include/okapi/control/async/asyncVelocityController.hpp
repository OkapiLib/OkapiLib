/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#ifndef _OKAPI_ASYNCVELOCITYCONTROLLER_HPP_
#define _OKAPI_ASYNCVELOCITYCONTROLLER_HPP_

#include "okapi/control/async/asyncController.hpp"
#include <memory>

namespace okapi {
class AsyncVelocityController;

class AsyncVelocityControllerParams : public AsyncControllerParams {
  public:
  /**
   * Constructs a new AsyncVelocityController.
   *
   * @return shared_ptr to the AsyncVelocityController
   */
  virtual std::shared_ptr<AsyncVelocityController> make() const = 0;
};

class AsyncVelocityController : public AsyncController {};
} // namespace okapi

#endif
