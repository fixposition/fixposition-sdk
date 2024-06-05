########################################################################################################################

.PHONY: default
default:
	@echo "Make what? Try 'make help'!"
	@exit 1

# Defaults
BUILD_TYPE = Release

# User vars
-include config.mk

ifneq ($(MAKECMDGOALS),help)
    ifeq ($(INSTALL_PREFIX),)
        $(error Please provide a INSTALL_PREFIX. Try 'make help'!)
    endif
    ifeq ($(ROS_PACKAGE_PATH),)
        $(warning No ROS_PACKAGE_PATH! ROS functionality won't be compiled-in!)
    endif
endif

.PHONY: help
help:
	@echo "Usage:"
	@echo
	@echo "    make <target> INSTALL_PREFIX=... [BUILD_TYPE=Release|Debug] [VERBOSE=1]"
	@echo
	@echo "Where possible <target>s are:"
	@echo
	@echo "    clean               Clean build directory"
	@echo "    cmake               Configure packages"
	@echo "    build               Build packages"
	@echo "    install             Install packages (into INSTALL_PREFIX path)"
	@echo
	@echo "Typically you want to do something like this:"
	@echo
	@echo "     source /opt/ros/noetic/setup.bash"
	@echo "     make install INSTALL_PREFIX=~/fpsdk"
	@echo
	@echo "Notes:"
	@echo
	@echo "- ROS_PACKAGE_PATH can be provided through catkin/ros env (recommended) or on the command line"
	@echo "- Command line variables can be stored into a config.mk file, which is automatically loaded"
	@echo "- When changing the INSTALL_PREFIX or config.mk, a 'make clean' will be required"
	@echo

########################################################################################################################

TOUCH  := touch
MKDIR  := mkdir
ECHO   := echo
RM     := rm
CMAKE  := cmake

ifeq ($(VERBOSE),1)
V =
V1 =
V2 =
V12 =
RM += -v
MV += -v
CP += -v
MKDIR += -v
else
ZIP += -q
UNZIP += -q
V = @
V1 = > /dev/null
V2 = 2> /dev/null
V12 = 2>&1 > /dev/null
endif

fancyterm := true
ifeq ($(TERM),dumb)
fancyterm := false
endif
ifeq ($(TERM),)
fancyterm := false
endif
ifneq ($(MSYSTEM),)
fancyterm := false
endif
ifeq ($(fancyterm),true)
HLW="\\e[1m"
HLO="\\e[m"
else
HLW=
HLO=
endif

########################################################################################################################

CMAKE_ARGS := -DCMAKE_INSTALL_PREFIX=$(INSTALL_PREFIX)
CMAKE_ARGS += -DCMAKE_BUILD_TYPE=$(BUILD_TYPE)
ifneq ($(ROS_PACKAGE_PATH),)
  CMAKE_ARGS += -DROS_PACKAGE_PATH=$(ROS_PACKAGE_PATH)
endif

BUILD_DIR = build/$(BUILD_TYPE)

# "All-in-one" targets
.PHONY: clean
clean:
	$(V)$(RM) -rf $(BUILD_DIR)

# ----------------------------------------------------------------------------------------------------------------------

.PHONY: cmake
cmake: $(BUILD_DIR)/.make-cmake

$(BUILD_DIR)/.make-cmake:
	@echo "$(HLW)***** Configure ($(BUILD_TYPE)) *****$(HLO)"
	$(V)$(CMAKE) -B $(BUILD_DIR) $(CMAKE_ARGS)
	$(V)$(TOUCH) $@

# ----------------------------------------------------------------------------------------------------------------------

.PHONY: build
build: $(BUILD_DIR)/.make-build

$(BUILD_DIR)/.make-build: $(BUILD_DIR)/.make-cmake
	@echo "$(HLW)***** Build ($(BUILD_TYPE)) *****$(HLO)"
	$(V)$(CMAKE) --build $(BUILD_DIR)
	$(V)$(TOUCH) $@


# ----------------------------------------------------------------------------------------------------------------------

.PHONY: install
install: $(BUILD_DIR)/.make-install

$(BUILD_DIR)/.make-install: $(BUILD_DIR)/.make-build
	@echo "$(HLW)***** Install ($(BUILD_TYPE)) *****$(HLO)"
	$(V)$(CMAKE) --install $(BUILD_DIR)
	$(V)$(TOUCH) $@

########################################################################################################################
