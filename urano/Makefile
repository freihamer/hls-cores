NAME = urano
SRCS= urano.cpp
LOCAL_CONFIG = -legup-config=config.tcl
SHLS_ROOT_DIR ?= $(abspath $(dir $(shell which shls))/..)

LEVEL = $(SHLS_ROOT_DIR)/examples

USER_CXX_FLAG += -O1 -pg -Wall -Wno-strict-aliasing -Wno-unused-label -Wno-unknown-pragmas -Wno-attributes -I$(SHLS_ROOT_DIR)/smarthls-library

include $(LEVEL)/Makefile.common