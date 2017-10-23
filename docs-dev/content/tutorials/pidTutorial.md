## PID

Proportional-Integral-Derivative (PID) control is a simple and useful closed-loop controller. Say you have a subsystem with a motor and a potentiometer:

```c++
#include <device/pid.h>

int port = 1; //Motor port
int pot = 1; //Potentiometer
Pid controller(1, 0.2, 0.3); //PID controller with kP = 1, kI = 0.2, kD = 0.3
controller.setTarget(1500); //Set the target to 1500 on the potentiometer

while (1) {
    controller.step(analogRead(pot)); //Loop the controller with the new pot reading
    motorSet(port, controller.getOutput()); //Power the motor with the output of the controller
    taskDelay(15);
}
```
