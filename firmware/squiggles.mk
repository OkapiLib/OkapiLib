DOWNLOAD_SQUIGGLES:=$(shell mkdir -p cmake && cd cmake-build-debug && cmake -DCMAKE_BUILD_TYPE=Debug -G "CodeBlocks - Unix Makefiles" .. && make && cd ..)
SQUIGGLES_INC_DIR=$(ROOT)/cmake-build-debug/squiggles-src/include
SQUIGGLES_BUILD_DIR=$(ROOT)/cmake-build-debug/squiggles-build

INCLUDE+=-iquote"$(SQUIGGLES_INC_DIR)"
ELF_DEPS+=$(SQUIGGLES_BUILD_DIR)/libsquigglesd.a