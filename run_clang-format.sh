#!/bin/bash
find include/okapi/ -iname *.hpp | xargs clang-format-5.0 -i -style=file
find include/test/tests/ -iname *.hpp | xargs clang-format-5.0 -i -style=file
find src/ -iname *.cpp | xargs clang-format-5.0 -i -style=file
find test/ -iname *.cpp | xargs clang-format-5.0 -i -style=file
clang-format-5.0 -i -style=file include/test/crossPlatformTestRunner.hpp
clang-format-5.0 -i -style=file include/test/testRunner.hpp
