# Logging

## Enable the Default Logger

OkapiLib has a [Logger](okapi::Logger) class which is used internally by many of
OkapiLib's classes. If you are trying to work through a problem with your code
or with OkapiLib, enabling the default [Logger](okapi::Logger) can help you
gather more information. Enable it with:
```cpp
Logger::setDefaultLogger(std::make_shared<Logger>(std::make_unique<Timer>(), "/ser/sout", Logger::LogLevel::debug));
```

Place that code in a place where it will run before the code you are debugging.
The first line of `void initialize()` is a good place.
