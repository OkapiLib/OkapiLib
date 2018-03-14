/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#ifndef _OKAPI_ASYNCCONTROLLER_HPP_
#define _OKAPI_ASYNCCONTROLLER_HPP_

namespace okapi {
class AsyncControllerParams {};

class AsyncController {
  public:
  virtual void setTarget(double target) = 0;
};
} // namespace okapi

#endif
