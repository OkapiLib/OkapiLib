/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_FILTEREDCONTROLLERINPUT_HPP_
#define _OKAPI_FILTEREDCONTROLLERINPUT_HPP_

#include "okapi/control/controllerInput.hpp"
#include "okapi/filter/filter.hpp"
#include <memory>

namespace okapi {
template <typename InputType, typename FilterType>
class FilteredControllerInput : public ControllerInput {
  public:
  /**
   * A filtered controller input. Applies a filter to the controller input. Useful if you want to
   * place a filter between a control input and a control loop.
   *
   * @param iinput ControllerInput type
   * @param ifilter Filter type
   */
  FilteredControllerInput(InputType iinput, FilterType ifilter)
    : input(std::make_unique<InputType>(iinput)), filter(std::make_unique<FilterType>(ifilter)) {
  }

  /**
   * Gets the sensor value for use in a control loop. This method might be automatically called in
   * another thread by the controller.
   *
   * @return the current filtered sensor value.
   */
  virtual double controllerGet() override {
    return filter->filter(input->controllerGet());
  }

  protected:
  std::unique_ptr<InputType> input;
  std::unique_ptr<FilterType> filter;
};
} // namespace okapi

#endif
