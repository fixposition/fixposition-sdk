########################################################################################################################

.PHONY: default
default:
	@echo "Make what? Try 'make help'!"

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
	@echo "    clean               Clean all build directories"
	@echo "    install             Build and install all packages (into INSTALL_PREFIX path)"
	@echo
	@echo "Or individual targets for these <package>s: $(packages)"
	@echo
	@echo "    clean_<package>     Clean <package>'s build directory"
	@echo "    build_<package>     Build <package>"
	@echo "    install_<package>   Build and install <package>"
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

CMAKE_ARGS := -DCMAKE_INSTALL_PREFIX=$(INSTALL_PREFIX) -DROS_PACKAGE_PATH=$(ROS_PACKAGE_PATH)
CMAKE_ARGS += -DCMAKE_BUILD_TYPE=$(BUILD_TYPE)

define makeTarget
  # 1) package name
  #$ ( info makeTarget: [$1])

  # Add to list of known packages
  packages += $(1)

  # Dependencies (any file in that project.. for lack of better knowledge)
  deps_$1 = $(MAKEFILE_LIST) $(filter-out $1/build%, $(sort $(wildcard $1/* $1/*/* $1/*/*/* $1/*/*/*/*)))

  # Build directory
  $1/build/$(BUILD_TYPE):
	$(V)$(MKDIR) -p $$@

  # Clean
  .PHONY: clean_$1
  clean_$1:
	$(V)$(RM) -rf $1/build

  # Configure cmake
  .PHONY: cmake_$1
  cmake_$1: $1/build/$(BUILD_TYPE)/.cmake
  $1/build/$(BUILD_TYPE)/.cmake: $$(deps_$1) | $1/build/$(BUILD_TYPE)
	@echo "$(HLW)***** Configure $1 ($(BUILD_TYPE)) *****$(HLO)"
	$(V)$(CMAKE) -B $1/build/$(BUILD_TYPE) -S $1 $(CMAKE_ARGS)
	$(V)$(TOUCH) $$@

  # Build (Note: $$(MAKE), not $(MAKE), so that e.g. -j4 is passed-through)
  .PHONY: build_$1
  build_$1: $1/build/$(BUILD_TYPE)/.build
  $1/build/$(BUILD_TYPE)/.build: $1/build/$(BUILD_TYPE)/.cmake
	@echo "$(HLW)***** Build $1 ($(BUILD_TYPE)) *****$(HLO)"
	$(V)$$(MAKE) -C $1/build/$(BUILD_TYPE)
	$(V)$(TOUCH) $$@

  # Install
  .PHONY: install_$1
  install_$1: $1/build/$(BUILD_TYPE)/.install
  $1/build/$(BUILD_TYPE)/.install: $1/build/$(BUILD_TYPE)/.build
	@echo "$(HLW)***** Install $1 ($(BUILD_TYPE)) *****$(HLO)"
	$(V)$$(MAKE) -C $1/build/$(BUILD_TYPE) install
	$(V)$(TOUCH) $$@

  # Global targets
  targets_clean   += clean_$1
  targets_cmake   += cmake_$1
  targets_build   += build_$1
  targets_install += install_$1

endef

# Make targets for all packages
$(eval $(call makeTarget,fpcommon))
$(eval $(call makeTarget,fpapps))

# We have a ROS environment
ifneq ($(ROS_PACKAGE_PATH),)
  $(eval $(call makeTarget,fpros1))
  fpros1/build/$(BUILD_TYPE)/.cmake: fpcommon/build/$(BUILD_TYPE)/.install
  fpapps/build/$(BUILD_TYPE)/.cmake: fpros1/build/$(BUILD_TYPE)/.install
# We're only building the non-ROS stuff
else
  fpapps/build/$(BUILD_TYPE)/.cmake: fpcommon/build/$(BUILD_TYPE)/.install
endif

# $ ( info deps_fpcommon = $(deps_fpcommon))
# $ ( info deps_fpros1 = $(deps_fpros1))
# $ ( info deps_fpapps = $(deps_fpapps))

# "All-in-one" targets
.PHONY: clean
clean: $(targets_clean)
.PHONY: cmake
cmake: $(targets_cmake)
.PHONY: build
build: $(targets_build)
.PHONY: install
install: $(targets_install)

########################################################################################################################
