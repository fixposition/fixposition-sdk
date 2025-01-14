.SUFFIXES:

deps = $(sort $(wildcard CMakeList.txt Makefile ../../fpsdk_doc/* *.cpp \
    ../../fpsdk_common/* ../../fpsdk_common/*/* ../../fpsdk_common/*/*/* ../../fpsdk_common/*/*/*/* \
    ../../fpsdk_ros1/* ../../fpsdk_ros1/*/* ../../fpsdk_ros1/*/*/* ../../fpsdk_ros1/*/*/*/* \
    ../../fpsdk_ros2/* ../../fpsdk_ros2/*/* ../../fpsdk_ros2/*/*/* ../../fpsdk_ros2/*/*/*/* \
    ../../fpsdk_apps/* ../../fpsdk_apps/*/* ../../fpsdk_apps/*/*/* ../../fpsdk_apps/*/*/*/*))

$(info deps $(deps))

.PHONY: default
default: build/.make-build

build:
	mkdir -p $@

build/.make-build: $(deps) | build
	cmake -B build -S .
	cmake --build build --parallel 8
	touch $@

.PHONY: clean
clean:
	rm -rf build
