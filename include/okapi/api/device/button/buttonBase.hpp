#ifndef _OKAPI_BUTTONBASE_HPP_
#define _OKAPI_BUTTONBASE_HPP_

#include "okapi/api/device/button/abstractButton.hpp"

namespace okapi {
class ButtonBase : public AbstractButton {
  public:
  explicit ButtonBase(bool iinverted = false);

  /**
   * Return whether the button is currently pressed.
   **/
  bool isPressed() override;

  /**
   * Return whether the state of the button changed since the last time this method was called.
   **/
  bool changed() override;

  /**
   * Return whether the state of the button changed to pressed since the last time this method was
   *called.
   **/
  bool changedToPressed() override;

  /**
   * Return whether the state of the button to not pressed since the last time this method was
   *called.
   **/
  bool changedToReleased() override;

  protected:
  bool inverted{false};
  bool wasPressedLast_c{false};
  bool wasPressedLast_ctp{false};
  bool wasPressedLast_ctr{false};

  virtual bool currentlyPressed() = 0;

  private:
  bool changedImpl(bool &prevState);
};
} // namespace okapi

#endif
