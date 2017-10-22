## Generic Controller

Often times, you have a closed-loop controller, sensor, and a few motors acting together as a system, and you wind up with code that powers each motor individually like this:

```c++
#include <device/pid.h>

int port1 = 1, port2 = 2; //Motor ports
int pot = 1; //Potentiometer
Pid controller(1, 0.2, 0.3); //PID controller with kP = 1, kI = 0.2, kD = 0.3
controller.setTarget(1500); //Set the target to 1500 on the potentiometer

while (1) {
    controller.loop(analogRead(pot)); //Loop the controller with the new pot reading
    //Power the motors with the output of the controller
    motorSet(port1, controller.getOutput());
    motorSet(port2, controller.getOutput());
    taskDelay(15);
```

The motor motors and control structure we have the messier this becomes. So instead, Okapi provides a way to do this in a cleaner fashion:

```c++
#include <device/pid.h>
#include <control/genericController.h>

int pot = 1;
//Two motors in ports 1 and 2 like before
//Use make_shared to give GenericController the instance of your ControlObject
GenericController<2> liftController({1_m, 2_m}, std::make_shared<Pid>(Pid(1, 0.2, 0.3)));
liftController.setTarget(1500);

while (1) {
    controller.loop(analogRead(pot)); //Loop the controller and power the motors
    taskDelay(15);
}
```
