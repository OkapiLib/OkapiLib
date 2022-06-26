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

std::string OdomState::str(QLength idistanceUnit,
                           std::string distUnitName,
                           QAngle iangleUnit,
                           std::string angleUnitName) const {
  char buf[150];
  snprintf(buf,
           sizeof(buf),
           "OdomState(x=%.2f%s, y=%.2f%s, theta=%.2f%s)",
           x.convert(idistanceUnit),
           distUnitName.c_str(),
           y.convert(idistanceUnit),
           distUnitName.c_str(),
           theta.convert(iangleUnit),
           angleUnitName.c_str());
  return std::string(buf);
}

std::string OdomState::str(QLength idistanceUnit, QAngle iangleUnit) const {
  return str(idistanceUnit,
             "_" + std::string(getShortUnitName(idistanceUnit)),
             iangleUnit,
             "_" + std::string(getShortUnitName(iangleUnit)));
}

} // namespace okapi
