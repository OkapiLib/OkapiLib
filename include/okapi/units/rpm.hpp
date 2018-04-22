/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_RPM_HPP_
#define _OKAPI_RPM_HPP_

namespace okapi {
inline namespace units {
class RPM {
  public:
  constexpr RPM(const long double iRPMs) : value(iRPMs) {
  }

  constexpr RPM(const RPM &other) : value(other.value) {
  }

  ~RPM() = default;

  static constexpr RPM fromRPM(const long double iRPM) {
    return RPM(iRPM);
  }

  static constexpr RPM fromDPS(const long double iDPS) {
    return RPM(iDPS * 0.166666667);
  }

  constexpr long double toRPM() const {
    return value;
  }

  constexpr long double toDPS() const {
    return value * 6;
  }

  constexpr RPM operator+(const RPM &rhs) const {
    return RPM(value + rhs.value);
  }

  constexpr RPM operator+(const long double rhs) const {
    return RPM(value + rhs);
  }

  constexpr RPM operator-(const RPM &rhs) const {
    return RPM(value - rhs.value);
  }

  constexpr RPM operator-(const long double rhs) const {
    return RPM(value - rhs);
  }

  constexpr RPM operator*(const long double rhs) const {
    return RPM(value * rhs);
  }

  constexpr RPM operator/(const long double rhs) const {
    return RPM(value / rhs);
  }

  protected:
  const long double value;
};

constexpr RPM operator"" _rpm(const long double iRPMs) {
  return RPM(iRPMs);
}

constexpr RPM operator"" _dps(const long double iradians) {
  return RPM(iradians * 0.166666667);
}
} // namespace units
} // namespace okapi

#endif
