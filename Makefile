########################################################################################################################

.PHONY: default
default:
	@echo "Make what? Try 'make help'!"
	@exit 1

# Defaults (keep in sync with help screen below)
BUILD_TYPE     = Debug
INSTALL_PREFIX = fpsdk
BUILD_TESTING  =
VERBOSE        = 0

# User vars
-include config.mk

# Check if we have ROS
FPSDK_USE_ROS1=
FPSDK_USE_ROS2=
ifneq ($(ROS_PACKAGE_PATH),)
	FPSDK_USE_ROS1=yes
else ifeq ($(ROS_VERSION),2)
	FPSDK_USE_ROS2=yes
# else
# 	$(info No ROS_PACKAGE_PATH (ROS1) and no ROS_VERSION (ROS2) found)
endif

# A unique ID for this exact config we're using
configuid=$(shell echo "$(BUILD_TYPE) $(INSTALL_PREFIX) $(BUILD_TESTING) $(FPSDK_VERSION_STRING) $(CC) $(CXX) $$(uname -a)" | md5sum | cut -d " " -f1)

.PHONY: help
help:
	@echo "Usage:"
	@echo
	@echo "    make <target> [INSTALL_PREFIX=...] [BUILD_TYPE=Debug|Release] [BUILD_TESTING=|ON|OFF] [FPSDK_VERSION_STRING=x.x.x-gggggggg] [VERBOSE=1]"
	@echo
	@echo "Where possible <target>s are:"
	@echo
	@echo "    clean               Clean build directory"
	@echo "    cmake               Configure packages"
	@echo "    build               Build packages"
	@echo "    pre-commit          Run pre-commit"
	@echo "    test                Run tests"
	@echo "    install             Install packages (into INSTALL_PREFIX path)"
	@echo "    doc                 Generate documentation (into build directory)"
	@echo "    doc-dev             Generate documentation and start webserver to view it"
	@echo "    lint                Run linter"
	@echo
	@echo "Typically you want to do something like this:"
	@echo
	@echo "     source /opt/ros/noetic/setup.bash  # if you have ROS..."
	@echo "     make install INSTALL_PREFIX=~/fpsdk"
	@echo
	@echo "Notes:"
	@echo
	@echo "- ROS_PACKAGE_PATH can be provided through catkin/ros env (recommended) or on the command line"
	@echo "- All <target>s but 'clean' and 'pre-commit' require that the same command-line variables are passed"
	@echo "- Command-line variables can be stored into a config.mk file, which is automatically loaded"
	@echo "- 'make ci' runs the CI (more or less) like on Github. INSTALL_PREFIX and BUILD_TYPE have no effect here."
	@echo

########################################################################################################################

TOUCH      := touch
MKDIR      := mkdir
ECHO       := echo
RM         := rm
CMAKE      := cmake
DOXYGEN    := doxygen
NICE       := nice
CAT        := cat
PYTHON     := python
SED        := sed
LN         := ln
TEE        := tee
RUN_CLANG_TIDY := run-clang-tidy

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

# Disable all built-in rules
.SUFFIXES:

########################################################################################################################

CMAKE_ARGS_BUILD :=
CMAKE_ARGS_INSTALL :=
CTEST_ARGS := --output-on-failure
CMAKE_ARGS := -DCMAKE_INSTALL_PREFIX=$(INSTALL_PREFIX)
CMAKE_ARGS += -DCMAKE_BUILD_TYPE=$(BUILD_TYPE)
ifneq ($(ROS_PACKAGE_PATH),)
  CMAKE_ARGS += -DROS_PACKAGE_PATH=$(ROS_PACKAGE_PATH)
endif
ifneq ($(BUILD_TESTING),)
  CMAKE_ARGS += -DBUILD_TESTING=$(BUILD_TESTING)
endif
ifneq ($(FPSDK_VERSION_STRING),)
  CMAKE_ARGS += -DVERSION_STRING=$(FPSDK_VERSION_STRING)
endif

ifeq ($(BUILD_TYPE),Release)
  CMAKE_ARGS_INSTALL += --strip
endif

MAKEFLAGS = --no-print-directory

ifneq ($(VERBOSE),0)
  CMAKE_ARGS_BUILD += --verbose
  CTEST_ARGS += --verbose
endif

ifeq ($(GITHUB_WORKSPACE),)
  PARALLEL := $(shell nproc --ignore=2)
  CMAKE_ARGS_BUILD = --parallel $(PARALLEL)
  NICE_BUILD=$(NICE) -19
else
  PARALLEL := 4
  CMAKE_ARGS_BUILD = --parallel $(PARALLEL)
  NICE_BUILD=
endif

BUILD_DIR = build/$(BUILD_TYPE)

# "All-in-one" targets
.PHONY: clean
clean:
	$(V)$(RM) -rf $(BUILD_DIR)

.PHONY: distclean
distclean:
	$(V)$(RM) -rf install build fpsdk

$(BUILD_DIR):
	$(V)$(MKDIR) -p $@

# Detect changed build config
ifneq ($(MAKECMDGOALS),)
ifneq ($(MAKECMDGOALS),help)
ifneq ($(MAKECMDGOALS),pre-commit)
ifneq ($(MAKECMDGOALS),ci)
ifneq ($(MAKECMDGOALS),distclean)
ifneq ($(MAKECMDGOALS),doc)
builddiruid=$(shell $(CAT) $(BUILD_DIR)/.make-uid 2>/dev/null || echo "none")
ifneq ($(builddiruid),$(configuid))
    dummy=$(shell $(RM) -f $(BUILD_DIR)/.make-uid)
endif
endif
endif
endif
endif
endif
endif

$(BUILD_DIR)/.make-uid: | $(BUILD_DIR)
	$(V)$(ECHO) $(configuid) > $@

# ----------------------------------------------------------------------------------------------------------------------

.PHONY: cmake
cmake: $(BUILD_DIR)/.make-cmake

deps_cmake := Makefile $(wildcard config.mk) $(wildcard $(sort $(wildcard CMakeLists.txt */CMakeLists.txt */cmake/*)))

$(BUILD_DIR)/.make-cmake: $(deps_cmake) $(BUILD_DIR)/.make-uid
	@echo "$(HLW)***** Configure ($(BUILD_TYPE)) *****$(HLO)"
	$(V)$(CMAKE) -B $(BUILD_DIR) $(CMAKE_ARGS)
	$(V)$(TOUCH) $@

# ----------------------------------------------------------------------------------------------------------------------

deps_build = $(sort $(wildcard fpsdk.sh fpsdk_doc/* \
    fpsdk_common/* fpsdk_common/*/* fpsdk_common/*/*/* fpsdk_common/*/*/*/* \
    fpsdk_ros1/* fpsdk_ros1/*/* fpsdk_ros1/*/*/* fpsdk_ros1/*/*/*/* \
    fpsdk_ros2/* fpsdk_ros2/*/* fpsdk_ros2/*/*/* fpsdk_ros2/*/*/*/* \
    fpsdk_apps/* fpsdk_apps/*/* fpsdk_apps/*/*/* fpsdk_apps/*/*/*/*))

.PHONY: build
build: $(BUILD_DIR)/.make-build

$(BUILD_DIR)/.make-build: $(deps_build) $(BUILD_DIR)/.make-cmake $(BUILD_DIR)/.make-uid
	@echo "$(HLW)***** Build ($(BUILD_TYPE)) *****$(HLO)"
	$(V)$(NICE_BUILD) $(CMAKE) --build $(BUILD_DIR) $(CMAKE_ARGS_BUILD) -- -k
	$(V)$(TOUCH) $@

# ----------------------------------------------------------------------------------------------------------------------

.PHONY: install
install: $(BUILD_DIR)/.make-install

$(BUILD_DIR)/.make-install: $(BUILD_DIR)/.make-build
	@echo "$(HLW)***** Install ($(BUILD_TYPE)) *****$(HLO)"
	$(V)$(CMAKE) --install $(BUILD_DIR) $(CMAKE_ARGS_INSTALL)
	$(V)$(TOUCH) $@

# ----------------------------------------------------------------------------------------------------------------------

.PHONY: test
test: $(BUILD_DIR)/.make-build
	@echo "$(HLW)***** Test ($(BUILD_TYPE)) *****$(HLO)"
	$(V)(cd $(BUILD_DIR)/fpsdk_common && ctest $(CTEST_ARGS))
ifneq ($(FPSDK_USE_ROS1),)
	$(V)(cd $(BUILD_DIR)/fpsdk_ros1 && ctest $(CTEST_ARGS))
endif
ifneq ($(FPSDK_USE_ROS2),)
	$(V)(cd $(BUILD_DIR)/fpsdk_ros2 && ctest $(CTEST_ARGS))
endif

# ----------------------------------------------------------------------------------------------------------------------

.PHONY: doc
doc: $(BUILD_DIR)/.make-doc
	@echo "now run: xdg-open $(BUILD_DIR)/doc/index.html"

$(BUILD_DIR)/.make-doc: $(BUILD_DIR)/.make-build fpsdk_doc/Doxyfile
	@echo "$(HLW)***** Doc ($(BUILD_TYPE)) *****$(HLO)"
	$(V)$(MKDIR) -p $(BUILD_DIR)/helpscreens
	$(V)$(BUILD_DIR)/fpsdk_apps/fpltool    -h > $(BUILD_DIR)/helpscreens/fpltool_helpscreen.txt
	$(V)$(BUILD_DIR)/fpsdk_apps/parsertool -h > $(BUILD_DIR)/helpscreens/parsertool_helpscreen.txt
	$(V)$(BUILD_DIR)/fpsdk_apps/timeconv   -h > $(BUILD_DIR)/helpscreens/timeconv_helpscreen.txt
	$(V)$(BUILD_DIR)/fpsdk_apps/timetool   -h > $(BUILD_DIR)/helpscreens/timetool_helpscreen.txt
	$(V)$(BUILD_DIR)/fpsdk_apps/yaml2shell -h > $(BUILD_DIR)/helpscreens/yaml2shell_helpscreen.txt
	$(V)./fpsdk.sh -h                         > $(BUILD_DIR)/helpscreens/fpsdk_helpscreen.txt
	$(V)( \
            cat fpsdk_doc/Doxyfile; \
            echo "PROJECT_NUMBER = $$(cat $(BUILD_DIR)/FPSDK_VERSION_STRING || echo 'unknown revision')"; \
            echo "OUTPUT_DIRECTORY = $(BUILD_DIR)"; \
            echo "EXAMPLE_PATH += $(BUILD_DIR)/helpscreens"; \
            if [ -d build-artefacts ]; then echo "EXAMPLE_PATH += build-artefacts"; fi; \
        ) | $(DOXYGEN) -
	$(V)$(TOUCH) $@

.PHONY: doc-dev
doc-dev: $(BUILD_DIR)/.make-doc
	$(V)(cd $(BUILD_DIR)/doc && $(PYTHON) -m http.server 8000)

# ----------------------------------------------------------------------------------------------------------------------

.PHONY: lint
lint: $(BUILD_DIR)/.lint-clang-tidy

$(BUILD_DIR)/.lint-clang-tidy: $(BUILD_DIR)/.make-build
	@echo "$(HLW)***** clang-tidy ($(BUILD_TYPE)) *****$(HLO)"
	$(V)$(NICE_BUILD) $(RUN_CLANG_TIDY) -p $(BUILD_DIR) -header-filter=.* -j $(PARALLEL) \
	    | $(TEE) $(BUILD_DIR)/clang-tidy.log
	$(V)$(TOUCH) $@

# ----------------------------------------------------------------------------------------------------------------------

.PHONY: ci
ci: $(BUILD_DIR)/.ci-bookworm $(BUILD_DIR)/.ci-noetic $(BUILD_DIR)/.ci-humble $(BUILD_DIR)/.ci-jazzy

$(BUILD_DIR)/.ci-bookworm: $(deps)
	@echo "$(HLW)***** CI (bookworm) *****$(HLO)"
ifeq ($(FPSDK_IMAGE),)
	$(V)docker/docker.sh run bookworm-ci ./docker/ci.sh
	$(V)$(TOUCH) $@
else
	@echo "This ($@) should not run inside Docker!"
	@false
endif

$(BUILD_DIR)/.ci-noetic: $(deps)
	@echo "$(HLW)***** CI (noetic) *****$(HLO)"
ifeq ($(FPSDK_IMAGE),)
	$(V)docker/docker.sh run noetic-ci ./docker/ci.sh
	$(V)$(TOUCH) $@
else
	@echo "This ($@) should not run inside Docker!"
	@false
endif

$(BUILD_DIR)/.ci-humble: $(deps)
	@echo "$(HLW)***** CI (humble) *****$(HLO)"
ifeq ($(FPSDK_IMAGE),)
	$(V)docker/docker.sh run humble-ci ./docker/ci.sh
	$(V)$(TOUCH) $@
else
	@echo "This ($@) should not run inside Docker!"
	@false
endif

$(BUILD_DIR)/.ci-jazzy: $(deps)
	@echo "$(HLW)***** CI (jazzy) *****$(HLO)"
ifeq ($(FPSDK_IMAGE),)
	$(V)docker/docker.sh run jazzy-ci ./docker/ci.sh
	$(V)$(TOUCH) $@
else
	@echo "This ($@) should not run inside Docker!"
	@false
endif

# ----------------------------------------------------------------------------------------------------------------------

.PHONY: pre-commit
pre-commit: | $(BUILD_DIR)
	@echo "$(HLW)***** pre-commit checks *****$(HLO)"
	$(V)$(MKDIR) -p $(BUILD_DIR)/.bin
	$(V)$(LN) -sf /usr/bin/clang-format-17 $(BUILD_DIR)/.bin/clang-format
	$(V)export PATH=$(BUILD_DIR)/.bin:$$PATH; pre-commit run --all-files --hook-stage manual || return 1

########################################################################################################################
