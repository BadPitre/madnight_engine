# Test game Makefile - builds a .ps-exe linking against the engine
# located in the parent directory of this folder.

TARGET = madnight-engine-game
TYPE = ps-exe

ENGINEDIR := $(abspath $(dir $(lastword $(MAKEFILE_LIST)))/..)/

CPPFLAGS += -I$(ENGINEDIR)src
CPPFLAGS += -I$(ENGINEDIR)third_party/nugget

rwildcard=$(wildcard $1$2) $(foreach d,$(wildcard $1*),$(call rwildcard,$d/,$2))
SRCS := $(call rwildcard,src/,*.cpp)

CXXFLAGS = -std=c++20
CPPFLAGS_debug = -g -O0

OUTDIR = build

include $(ENGINEDIR)madnight-engine.mk
include $(ENGINEDIR)third_party/nugget/psyqo-paths/psyqo-paths.mk
include $(ENGINEDIR)third_party/nugget/psyqo/psyqo.mk

.PHONY: all
all: $(TARGET).$(TYPE)
	@mkdir -p $(OUTDIR)
	@mv -f $(TARGET).ps-exe $(OUTDIR)/
	@mv -f $(TARGET).elf $(OUTDIR)/ 2>/dev/null || true
	@mv -f $(TARGET).map $(OUTDIR)/ 2>/dev/null || true
	@echo "Build complete: $(OUTDIR)/$(TARGET).ps-exe"
