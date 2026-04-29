# Project Name
TARGET := drone

# Define CPP_SOURCES as the list of all .cpp files in the current directory and
# its subdirectories, excluding 'prototype.cpp'.
CPP_SOURCES := $(shell find . -type f -name "*.cpp")

# Library Locations
LIBDAISY_DIR := ../../libDaisy/
DAISYSP_DIR := ../../DaisySP/

CPP_STANDARD := -std=gnu++17

# Core location, and generic Makefile.
SYSTEM_FILES_DIR = $(LIBDAISY_DIR)/core
include $(SYSTEM_FILES_DIR)/Makefile
