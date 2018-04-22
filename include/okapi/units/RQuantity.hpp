/**
 * @author Mikhail Semenov
 * @author Benjamin Jurke
 * @author Ryan Benasutti, WPI
 *
 * This code is a modified version of Benjamin Jurke's work in 2015. You can read his blog post
 * here:
 * https://benjaminjurke.com/content/articles/2015/compile-time-numerical-unit-dimension-checking/
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_RQUANTITY_HPP_
#define _OKAPI_RQUANTITY_HPP_

#include <ratio>

template <typename MassDim, typename LengthDim, typename TimeDim, typename AngleDim>
class RQuantity {
  public:
  constexpr RQuantity() : value(0.0) {
  }

  constexpr RQuantity(double val) : value(val) {
  }

  constexpr RQuantity(long double val) : value(static_cast<double>(val)) {
  }

  // The intrinsic operations for a quantity with a unit is addition and subtraction
  constexpr RQuantity const &operator+=(const RQuantity &rhs) {
    value += rhs.value;
    return *this;
  }

  constexpr RQuantity const &operator-=(const RQuantity &rhs) {
    value -= rhs.value;
    return *this;
  }

  // Returns the value of the quantity in multiples of the specified unit
  constexpr double convert(const RQuantity &rhs) const {
    return value / rhs.value;
  }

  // returns the raw value of the quantity (should not be used)
  constexpr double getValue() const {
    return value;
  }

  private:
  double value;
};

// Predefined (physical unit) quantity types:
// ------------------------------------------
#define QUANTITY_TYPE(_Mdim, _Ldim, _Tdim, _Adim, name)                                            \
  typedef RQuantity<std::ratio<_Mdim>, std::ratio<_Ldim>, std::ratio<_Tdim>, std::ratio<_Adim>>    \
    name;

// Replacement of "double" type
QUANTITY_TYPE(0, 0, 0, 0, Number)

// Physical quantity types
QUANTITY_TYPE(1, 0, 0, 0, QMass)
QUANTITY_TYPE(0, 1, 0, 0, QLength)
QUANTITY_TYPE(0, 2, 0, 0, QArea)
QUANTITY_TYPE(0, 3, 0, 0, QVolume)
QUANTITY_TYPE(0, 0, 1, 0, QTime)
QUANTITY_TYPE(0, 1, -1, 0, QSpeed)
QUANTITY_TYPE(0, 1, -2, 0, QAcceleration)
QUANTITY_TYPE(0, 1, -3, 0, QJerk)
QUANTITY_TYPE(0, 0, -1, 0, QFrequency)
QUANTITY_TYPE(1, 1, -2, 0, QForce)
QUANTITY_TYPE(1, -1, -2, 0, QPressure)

// Angle type:
QUANTITY_TYPE(0, 0, 0, 1, QAngle)
QUANTITY_TYPE(0, 0, -1, 1, QAngularSpeed)
QUANTITY_TYPE(0, 0, -2, 1, QAngularAcceleration)
QUANTITY_TYPE(0, 0, -3, 1, QAngularJerk)

// Standard arithmetic operators:
// ------------------------------
template <typename M, typename L, typename T, typename A>
constexpr RQuantity<M, L, T, A> operator+(const RQuantity<M, L, T, A> &lhs,
                                          const RQuantity<M, L, T, A> &rhs) {
  return RQuantity<M, L, T, A>(lhs.getValue() + rhs.getValue());
}
template <typename M, typename L, typename T, typename A>
constexpr RQuantity<M, L, T, A> operator-(const RQuantity<M, L, T, A> &lhs,
                                          const RQuantity<M, L, T, A> &rhs) {
  return RQuantity<M, L, T, A>(lhs.getValue() - rhs.getValue());
}
template <typename M1, typename L1, typename T1, typename A1, typename M2, typename L2, typename T2,
          typename A2>
constexpr RQuantity<std::ratio_add<M1, M2>, std::ratio_add<L1, L2>, std::ratio_add<T1, T2>,
                    std::ratio_add<A1, A2>>
operator*(const RQuantity<M1, L1, T1, A1> &lhs, const RQuantity<M2, L2, T2, A2> &rhs) {
  return RQuantity<std::ratio_add<M1, M2>, std::ratio_add<L1, L2>, std::ratio_add<T1, T2>,
                   std::ratio_add<A1, A2>>(lhs.getValue() * rhs.getValue());
}
template <typename M, typename L, typename T, typename A>
constexpr RQuantity<M, L, T, A> operator*(const double &lhs, const RQuantity<M, L, T, A> &rhs) {
  return RQuantity<M, L, T, A>(lhs * rhs.getValue());
}
template <typename M1, typename L1, typename T1, typename A1, typename M2, typename L2, typename T2,
          typename A2>
constexpr RQuantity<std::ratio_subtract<M1, M2>, std::ratio_subtract<L1, L2>,
                    std::ratio_subtract<T1, T2>, std::ratio_subtract<A1, A2>>
operator/(const RQuantity<M1, L1, T1, A1> &lhs, const RQuantity<M2, L2, T2, A2> &rhs) {
  return RQuantity<std::ratio_subtract<M1, M2>, std::ratio_subtract<L1, L2>,
                   std::ratio_subtract<T1, T2>, std::ratio_subtract<A1, A2>>(lhs.getValue() /
                                                                             rhs.getValue());
}
template <typename M, typename L, typename T, typename A>
constexpr RQuantity<std::ratio_subtract<std::ratio<0>, M>, std::ratio_subtract<std::ratio<0>, L>,
                    std::ratio_subtract<std::ratio<0>, T>, std::ratio_subtract<std::ratio<0>, A>>
operator/(double x, const RQuantity<M, L, T, A> &rhs) {
  return RQuantity<std::ratio_subtract<std::ratio<0>, M>, std::ratio_subtract<std::ratio<0>, L>,
                   std::ratio_subtract<std::ratio<0>, T>, std::ratio_subtract<std::ratio<0>, A>>(
    x / rhs.getValue());
}
template <typename M, typename L, typename T, typename A>
constexpr RQuantity<M, L, T, A> operator/(const RQuantity<M, L, T, A> &rhs, double x) {
  return RQuantity<M, L, T, A>(rhs.getValue() / x);
}

// Comparison operators for quantities:
// ------------------------------------
template <typename M, typename L, typename T, typename A>
constexpr bool operator==(const RQuantity<M, L, T, A> &lhs, const RQuantity<M, L, T, A> &rhs) {
  return (lhs.getValue() == rhs.getValue());
}
template <typename M, typename L, typename T, typename A>
constexpr bool operator!=(const RQuantity<M, L, T, A> &lhs, const RQuantity<M, L, T, A> &rhs) {
  return (lhs.getValue() != rhs.getValue());
}
template <typename M, typename L, typename T, typename A>
constexpr bool operator<=(const RQuantity<M, L, T, A> &lhs, const RQuantity<M, L, T, A> &rhs) {
  return (lhs.getValue() <= rhs.getValue());
}
template <typename M, typename L, typename T, typename A>
constexpr bool operator>=(const RQuantity<M, L, T, A> &lhs, const RQuantity<M, L, T, A> &rhs) {
  return (lhs.getValue() >= rhs.getValue());
}
template <typename M, typename L, typename T, typename A>
constexpr bool operator<(const RQuantity<M, L, T, A> &lhs, const RQuantity<M, L, T, A> &rhs) {
  return (lhs.getValue() < rhs.getValue());
}
template <typename M, typename L, typename T, typename A>
constexpr bool operator>(const RQuantity<M, L, T, A> &lhs, const RQuantity<M, L, T, A> &rhs) {
  return (lhs.getValue() > rhs.getValue());
}

// Predefined units:
// -----------------

// Predefined mass units:
constexpr QMass kg(1.0); // SI base unit
constexpr QMass gramme = 0.001 * kg;
constexpr QMass tonne = 1000 * kg;
constexpr QMass ounce = 0.028349523125 * kg;
constexpr QMass pound = 16 * ounce;
constexpr QMass stone = 14 * pound;

// Predefined length-derived units
constexpr QLength meter(1.0); // SI base unit
constexpr QLength decimeter = meter / 10;
constexpr QLength centimeter = meter / 100;
constexpr QLength millimeter = meter / 1000;
constexpr QLength kilometer = 1000 * meter;
constexpr QLength inch = 2.54 * centimeter;
constexpr QLength foot = 12 * inch;
constexpr QLength yard = 3 * foot;
constexpr QLength mile = 5280 * foot;

constexpr QArea kilometer2 = kilometer * kilometer;
constexpr QArea meter2 = meter * meter;
constexpr QArea decimeter2 = decimeter * decimeter;
constexpr QArea centimeter2 = centimeter * centimeter;
constexpr QArea millimeter2 = millimeter * millimeter;
constexpr QArea inch2 = inch * inch;
constexpr QArea foot2 = foot * foot;
constexpr QArea mile2 = mile * mile;

constexpr QVolume kilometer3 = kilometer2 * kilometer;
constexpr QVolume meter3 = meter2 * meter;
constexpr QVolume decimeter3 = decimeter2 * decimeter;
constexpr QVolume litre = decimeter3;
constexpr QVolume centimeter3 = centimeter2 * centimeter;
constexpr QVolume millimeter3 = millimeter2 * millimeter;
constexpr QVolume inch3 = inch2 * inch;
constexpr QVolume foot3 = foot2 * foot;
constexpr QVolume mile3 = mile2 * mile;

// Predefined time-derived units:
constexpr QTime second(1.0); // SI base unit
constexpr QTime millisecond = second / 1000;
constexpr QTime minute = 60 * second;
constexpr QTime hour = 60 * minute;
constexpr QTime day = 24 * hour;

constexpr QFrequency Hz(1.0);

// Predefined mixed units:
constexpr QAcceleration G = 9.80665 * meter / (second * second);

constexpr QForce newton(1.0);
constexpr QForce poundforce = pound * G;
constexpr QForce kilopond = kg * G;

constexpr QPressure Pascal(1.0);
constexpr QPressure bar = 100000 * Pascal;
constexpr QPressure psi = pound * G / inch2;

constexpr QSpeed miph = mile / hour;

// Physical unit literals:
// -----------------------

// literals for length units
constexpr QLength operator"" _mm(long double x) {
  return static_cast<double>(x) * millimeter;
}
constexpr QLength operator"" _cm(long double x) {
  return static_cast<double>(x) * centimeter;
}
constexpr QLength operator"" _m(long double x) {
  return static_cast<double>(x) * meter;
}
constexpr QLength operator"" _km(long double x) {
  return static_cast<double>(x) * kilometer;
}
constexpr QLength operator"" _mi(long double x) {
  return static_cast<double>(x) * mile;
}
constexpr QLength operator"" _yd(long double x) {
  return static_cast<double>(x) * yard;
}
constexpr QLength operator"" _ft(long double x) {
  return static_cast<double>(x) * foot;
}
constexpr QLength operator"" _in(long double x) {
  return static_cast<double>(x) * inch;
}
constexpr QLength operator"" _mm(unsigned long long int x) {
  return static_cast<double>(x) * millimeter;
}
constexpr QLength operator"" _cm(unsigned long long int x) {
  return static_cast<double>(x) * centimeter;
}
constexpr QLength operator"" _m(unsigned long long int x) {
  return static_cast<double>(x) * meter;
}
constexpr QLength operator"" _km(unsigned long long int x) {
  return static_cast<double>(x) * kilometer;
}
constexpr QLength operator"" _mi(unsigned long long int x) {
  return static_cast<double>(x) * mile;
}
constexpr QLength operator"" _yd(unsigned long long int x) {
  return static_cast<double>(x) * yard;
}
constexpr QLength operator"" _ft(unsigned long long int x) {
  return static_cast<double>(x) * foot;
}
constexpr QLength operator"" _in(unsigned long long int x) {
  return static_cast<double>(x) * inch;
}

// literals for speed units
constexpr QSpeed operator"" _mps(long double x) {
  return QSpeed(x);
}
constexpr QSpeed operator"" _miph(long double x) {
  return static_cast<double>(x) * mile / hour;
}
constexpr QSpeed operator"" _kmph(long double x) {
  return static_cast<double>(x) * kilometer / hour;
}
constexpr QSpeed operator"" _mps(unsigned long long int x) {
  return QSpeed(static_cast<long double>(x));
}
constexpr QSpeed operator"" _miph(unsigned long long int x) {
  return static_cast<double>(x) * mile / hour;
}
constexpr QSpeed operator"" _kmph(unsigned long long int x) {
  return static_cast<double>(x) * kilometer / hour;
}

// literal for frequency unit
constexpr QFrequency operator"" _Hz(long double x) {
  return QFrequency(x);
}
constexpr QFrequency operator"" _Hz(unsigned long long int x) {
  return QFrequency(static_cast<long double>(x));
}

// literals for time units
constexpr QTime operator"" _s(long double x) {
  return QTime(x);
}
constexpr QTime operator"" _min(long double x) {
  return static_cast<double>(x) * minute;
}
constexpr QTime operator"" _h(long double x) {
  return static_cast<double>(x) * hour;
}
constexpr QTime operator"" _day(long double x) {
  return static_cast<double>(x) * day;
}
constexpr QTime operator"" _s(unsigned long long int x) {
  return QTime(static_cast<double>(x));
}
constexpr QTime operator"" _min(unsigned long long int x) {
  return static_cast<double>(x) * minute;
}
constexpr QTime operator"" _h(unsigned long long int x) {
  return static_cast<double>(x) * hour;
}
constexpr QTime operator"" _day(unsigned long long int x) {
  return static_cast<double>(x) * day;
}

// literals for mass units
constexpr QMass operator"" _kg(long double x) {
  return QMass(x);
}
constexpr QMass operator"" _g(long double x) {
  return static_cast<double>(x) * gramme;
}
constexpr QMass operator"" _t(long double x) {
  return static_cast<double>(x) * tonne;
}
constexpr QMass operator"" _oz(long double x) {
  return static_cast<double>(x) * ounce;
}
constexpr QMass operator"" _lb(long double x) {
  return static_cast<double>(x) * pound;
}
constexpr QMass operator"" _st(long double x) {
  return static_cast<double>(x) * stone;
}
constexpr QMass operator"" _kg(unsigned long long int x) {
  return QMass(static_cast<double>(x));
}
constexpr QMass operator"" _g(unsigned long long int x) {
  return static_cast<double>(x) * gramme;
}
constexpr QMass operator"" _t(unsigned long long int x) {
  return static_cast<double>(x) * tonne;
}
constexpr QMass operator"" _oz(unsigned long long int x) {
  return static_cast<double>(x) * ounce;
}
constexpr QMass operator"" _lb(unsigned long long int x) {
  return static_cast<double>(x) * pound;
}
constexpr QMass operator"" _st(unsigned long long int x) {
  return static_cast<double>(x) * stone;
}

// literals for acceleration units
constexpr QAcceleration operator"" _mps2(long double x) {
  return QAcceleration(x);
}
constexpr QAcceleration operator"" _mps2(unsigned long long int x) {
  return QAcceleration(static_cast<double>(x));
}
constexpr QAcceleration operator"" _G(long double x) {
  return static_cast<double>(x) * G;
}
constexpr QAcceleration operator"" _G(unsigned long long int x) {
  return static_cast<double>(x) * G;
}

// literals for force units
constexpr QForce operator"" _n(long double x) {
  return QForce(x);
}
constexpr QForce operator"" _n(unsigned long long int x) {
  return QForce(static_cast<double>(x));
}
constexpr QForce operator"" _lbf(long double x) {
  return static_cast<double>(x) * poundforce;
}
constexpr QForce operator"" _lbf(unsigned long long int x) {
  return static_cast<double>(x) * poundforce;
}
constexpr QForce operator"" _kp(long double x) {
  return static_cast<double>(x) * kilopond;
}
constexpr QForce operator"" _kp(unsigned long long int x) {
  return static_cast<double>(x) * kilopond;
}

// literals for pressure units
constexpr QPressure operator"" _Pa(long double x) {
  return QPressure(x);
}
constexpr QPressure operator"" _Pa(unsigned long long int x) {
  return QPressure(static_cast<double>(x));
}
constexpr QPressure operator"" _bar(long double x) {
  return static_cast<double>(x) * bar;
}
constexpr QPressure operator"" _bar(unsigned long long int x) {
  return static_cast<double>(x) * bar;
}
constexpr QPressure operator"" _psi(long double x) {
  return static_cast<double>(x) * psi;
}
constexpr QPressure operator"" _psi(unsigned long long int x) {
  return static_cast<double>(x) * psi;
}

// Angular unit literals:
// ----------------------
constexpr long double operator"" _pi(long double x) {
  return static_cast<double>(x) * 3.1415926535897932384626433832795;
}
constexpr long double operator"" _pi(unsigned long long int x) {
  return static_cast<double>(x) * 3.1415926535897932384626433832795;
}

// Predefined angle units:
constexpr QAngle radian(1.0);
constexpr QAngle degree = static_cast<double>(2_pi / 360.0) * radian;

constexpr QAngularSpeed rpm = (360.0 * degree) / minute;
constexpr QAngularSpeed radps = radian / second;

// literals for angle units
constexpr QAngle operator"" _rad(long double x) {
  return QAngle(x);
}
constexpr QAngle operator"" _rad(unsigned long long int x) {
  return QAngle(static_cast<double>(x));
}
constexpr QAngle operator"" _deg(long double x) {
  return static_cast<double>(x) * degree;
}
constexpr QAngle operator"" _deg(unsigned long long int x) {
  return static_cast<double>(x) * degree;
}

// Conversion macro, which utilizes the string literals
#define ConvertTo(_x, _y) (_x).convert(1.0_##_y)

// Typesafe mathematical operations:
// ---------------------------------
template <typename M, typename L, typename T, typename A>
constexpr RQuantity<std::ratio_divide<M, std::ratio<2>>, std::ratio_divide<L, std::ratio<2>>,
                    std::ratio_divide<T, std::ratio<2>>, std::ratio_divide<A, std::ratio<2>>>
Qsqrt(const RQuantity<M, L, T, A> &num) {
  return RQuantity<std::ratio_divide<M, std::ratio<2>>, std::ratio_divide<L, std::ratio<2>>,
                   std::ratio_divide<T, std::ratio<2>>, std::ratio_divide<A, std::ratio<2>>>(
    sqrt(num.getValue()));
}

// Typesafe trigonometric operations
inline double sin(const QAngle &num) {
  return sin(num.getValue());
}
inline double cos(const QAngle &num) {
  return cos(num.getValue());
}
inline double tan(const QAngle &num) {
  return tan(num.getValue());
}

#endif
