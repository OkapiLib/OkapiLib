/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#include "okapi/api/filter/passthroughFilter.hpp"

namespace okapi {
PassthroughFilter::PassthroughFilter() = default;

double PassthroughFilter::filter(const double ireading) {
  lastOutput = ireading;
  return lastOutput;
}

double PassthroughFilter::getOutput() const {
  return lastOutput;
}
} // namespace okapi
