## Buttons

The Vex EDR system has three kinds of buttons in it: push buttons & limit switches, joystick buttons, and LCD module buttons. Okapi provides a class to interact with all three.

Say you have a button on your robot you want to use to control something. This is the simplest use, so Okapi makes this easy:

```c++
#include <device/button.h>

Button foo = 1_b; //literal for Button(1, false)
```

`foo` is now a non-inverted button in digital port 1. For reference, an inverted button returns true when it isn't pressed, and false when it is (opposite of a normal button).

Now that we have our button, we can check various things:

```c++
foo.isPressed(); //Returns true when the button is pressed (or false when pressed if inverted)
foo.edge(); //Returns true when the button changes from pressed to not pressed, or vice versa
foo.risingEdge(); //Returns true when the button changes from not pressed to pressed
foo.fallingEdge(); //Returns true when the button changes from pressed to not pressed
```

For example, say we wanted to do something whenever the button was pressed inside of our control loop:

```c++
while (1) { //Placeholder for the actual control loop
    //Trigger whenever we press the button (but not when we release it)
    if (foo.risingEdge()) {
        doSomething();
    }

    //Trigger whenever we release the button (but not when we press it)
    if (foo.fallingEdge()) {
        doSomethingElse();
    }
}
```

We use all the same exact code with the other two kinds of buttons, joystick & LCD:

```c++
Button foo(1, 8, JOY_DOWN); //Joystic button for the bottom of the right d-pad
Button foo(uart1, LCD_BTN_CENTER); //LCD center button in port uart1
```
