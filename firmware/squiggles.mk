DOWNLOAD_SQUIGGLES:=$(shell mkdir -p cmake-build-debug && cd cmake-build-debug && cmake -DCMAKE_BUILD_TYPE=Debug -G "CodeBlocks - Unix Makefiles" .. && mkdir -p ../src/squiggles && find ./squiggles-src/src -type f -name '*.cpp' ! -name 'main.cpp' | xargs cp -t ../src/squiggles && cd ..)
SQUIGGLES_INC_DIR:=$(ROOT)/cmake-build-debug/squiggles-src/include
INCLUDE+=-iquote"$(SQUIGGLES_INC_DIR)"