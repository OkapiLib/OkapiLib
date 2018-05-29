/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_CHASSISSCALES_HPP_
#define _OKAPI_CHASSISSCALES_HPP_

#include "okapi/units/QAngle.hpp"
#include "okapi/units/QLength.hpp"
#include "okapi/units/RQuantity.hpp"
#include <initializer_list>
#include <vector>

namespace okapi {
class ChassisScales {
  public:
  /**
   * The two scales a Chassis Controller needs to do all of its closed-loop control. First index is
   * the straight scale, second index is the turn scale. The straight scale converts motor degrees
   * to meters and the turn scale converts motor degrees to robot turn degrees. Read the clawbot
   * programming tutorial for more information behind the meaning of these two numbers.
   *
   * @param  iscales {straight scale, turn scale}
   */
  ChassisScales(const std::initializer_list<double> &iscales) {
    std::vector<double> vec(iscales);
    straight = vec.at(0);
    turn = vec.at(1);
  }

  /**
   * The two scales a Chassis Controller needs to do all of its closed-loop control. First index is
   * the wheel diameter, second index is the wheelbase width. Read the clawbot programming tutorial
   * for more information behind the meaning of these two numbers.
   *
   * The wheelbase diameter is the center-to-center distance between the wheels (center-to-center
   * meaning the width between the centers of both wheels). For example, if you are using four inch
   * omni wheels and there are 11.5 inches between the centers of each wheel, you would call the
   * constructor like so:
   *   ChassisScales scales({4_in, 11.5_in});
   *
   *                             Wheel diameter
   *
   *                              +-+
   *                              | |
   *                              v v
   *
   *                     +--->    ===             ===
   *                     |         +               +
   *                     |        ++---------------++
   *                     |        |                 |
   *    Wheelbase Width  |        |                 |
   *                     |        |                 |
   *                     |        |                 |
   *                     |        ++---------------++
   *                     |         +               +
   *                     +--->    ===             ===
   *
   *
   * @param  iwheelbase {wheel diameter, wheelbase width}
   */
  ChassisScales(const std::initializer_list<QLength> &iwheelbase) {
    std::vector<QLength> vec(iwheelbase);
    straight = 360 / (vec.at(0).convert(meter) * 1_pi);
    turn = vec.at(1).convert(meter) / vec.at(0).convert(meter);
  }

  virtual ~ChassisScales() = default;

  double straight;
  double turn;
};
} // namespace okapi

#endif
