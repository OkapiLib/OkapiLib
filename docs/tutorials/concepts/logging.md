# Logging

## Enable the Default Logger

OkapiLib has a [Logger](@ref okapi::Logger) class which is used internally by many of
OkapiLib's classes. It is a good idea to use the warn
[LogLevel](@ref okapi::Logger::LogLevel) all the time to catch warnings or errors as they arise
during development (or during a match! you can log to a file on the SD card with the path
`"/usd/test_logging.txt"`). Enable it with:
```cpp
Logger::setDefaultLogger(
    std::make_shared<Logger>(
        TimeUtilFactory::createDefault().getTimer(), // It needs a Timer
        "/ser/sout", // Output to the PROS terminal
        Logger::LogLevel::warn // Show errors and warnings
    )
);
```

Place that code in a place where it will run before the code you are debugging.
The first line of `initialize` is a good place.
