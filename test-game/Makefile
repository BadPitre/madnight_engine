# Test game Makefile - builds a .ps-exe linking against the engine
# located in the parent directory of this folder.

TARGET = madnight-engine-game
TYPE = ps-exe

CPPFLAGS += -I../src
CPPFLAGS += -I../third_party/nugget

rwildcard=$(wildcard $1$2) $(foreach d,$(wildcard $1*),$(call rwildcard,$d/,$2))
SRCS := $(call rwildcard,src/,*.cpp)

CXXFLAGS = -std=c++20
CPPFLAGS_debug = -g -O0

include ../madnight-engine.mk
include ../third_party/nugget/psyqo-paths/psyqo-paths.mk
include ../third_party/nugget/psyqo/psyqo.mk
