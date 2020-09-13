/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/odometry/odomState.hpp"
#include <sstream>

namespace okapi {
bool OdomState::operator==(const OdomState &rhs) const {
  return x == rhs.x && y == rhs.y && theta == rhs.theta;
}

bool OdomState::operator!=(const OdomState &rhs) const {
  return !(rhs == *this);
}

std::string OdomState::str() const {
  std::ostringstream os;
  os << "OdomState(x=" << std::to_string(x.convert(meter))
     << "m, y=" << std::to_string(y.convert(meter))
     << "m, theta=" << std::to_string(theta.convert(degree)) << "deg)";
  return os.str();
}
} // namespace okapi
