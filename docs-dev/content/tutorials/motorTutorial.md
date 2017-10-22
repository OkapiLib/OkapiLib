## Motor

Writing the full `Motor` constructor any time you want to specify a motor gets quite tedious, so Okapi has some literals that make specifying motors much easier:

```c++
Motor foo(2); //Motor on port 2
Motor foo = 2_m; //Equivalent literal

Motor foo(2, true); //Reversed motor on port 2
Motor foo = 2_rm; //Equivalent literal
```

Okapi also provides cubic control if you prefer to drive that way:

```c++
CubicMotor foo(2); //Motor on port 2
CubicMotor foo = 2_3m; //Equivalent literal

CubicMotor foo(2, true); //Reversed motor on port 2
CubicMotor foo = 2_3rm; //Equivalent literal
```

Both `Motor` and `CubicMotor` can be given to a `ChassisModel`, and the behavior of `tank` and `arcade` will change accordingly.
