/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_DEGREE_HPP_
#define _OKAPI_DEGREE_HPP_

namespace okapi {
inline namespace units {
class Degree {
  public:
  constexpr Degree(const long double idegrees) : value(idegrees) {
  }

  constexpr Degree(const Degree &other) : value(other.value) {
  }

  ~Degree() = default;

  static constexpr Degree fromDegrees(const long double idegrees) {
    return Degree(idegrees);
  }

  static constexpr Degree fromRadians(const long double iradians) {
    return Degree(iradians * 0.01745329252);
  }

  constexpr long double toDegrees() const {
    return value;
  }

  constexpr long double toRadians() const {
    return value * 57.295779513;
  }

  constexpr Degree operator+(const Degree &rhs) const {
    return Degree(value + rhs.value);
  }

  constexpr Degree operator+(const long double rhs) const {
    return Degree(value + rhs);
  }

  constexpr Degree operator-(const Degree &rhs) const {
    return Degree(value - rhs.value);
  }

  constexpr Degree operator-(const long double rhs) const {
    return Degree(value - rhs);
  }

  constexpr Degree operator*(const long double rhs) const {
    return Degree(value * rhs);
  }

  constexpr Degree operator/(const long double rhs) const {
    return Degree(value / rhs);
  }

  protected:
  const long double value;
};

constexpr Degree operator"" _deg(const long double idegrees) {
  return Degree(idegrees);
}

constexpr Degree operator"" _rad(const long double iradians) {
  return Degree(iradians * 0.01745329252);
}
}
}

#endif
