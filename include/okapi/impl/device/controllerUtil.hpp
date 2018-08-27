/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_CONTROLLERUTIL_HPP_
#define _OKAPI_CONTROLLERUTIL_HPP_

#include "api.h"

namespace okapi {
/**
 * Which controller role this has.
 */
enum class ControllerId { master = 0, partner = 1 };

/**
 * The analog sticks.
 */
enum class ControllerAnalog { leftX = 0, leftY = 1, rightX = 2, rightY = 3 };

/**
 * Various buttons.
 */
enum class ControllerDigital {
  L1 = 6,
  L2 = 7,
  R1 = 8,
  R2 = 9,
  up = 10,
  down = 11,
  left = 12,
  right = 13,
  X = 14,
  B = 15,
  Y = 16,
  A = 17
};

class ControllerUtil {
  public:
  /**
   * Maps an `id` to the PROS enum equivalent.
   */
  static pros::controller_id_e_t idToProsEnum(ControllerId in);

  /**
   * Maps an `analog` to the PROS enum equivalent.
   */
  static pros::controller_analog_e_t analogToProsEnum(ControllerAnalog in);

  /**
   * Maps a `digital` to the PROS enum equivalent.
   */
  static pros::controller_digital_e_t digitalToProsEnum(ControllerDigital in);
};
} // namespace okapi

#endif
