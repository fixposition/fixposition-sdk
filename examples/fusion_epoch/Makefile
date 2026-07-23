.SUFFIXES:

deps = $(sort $(wildcard CMakeList.txt Makefile ../../fpsdk_doc/* *.cpp \
    ../../fpsdk_common/* ../../fpsdk_common/*/* ../../fpsdk_common/*/*/* ../../fpsdk_common/*/*/*/* \
    ../../fpsdk_ros1/* ../../fpsdk_ros1/*/* ../../fpsdk_ros1/*/*/* ../../fpsdk_ros1/*/*/*/* \
    ../../fpsdk_ros2/* ../../fpsdk_ros2/*/* ../../fpsdk_ros2/*/*/* ../../fpsdk_ros2/*/*/*/* \
    ../../fpsdk_apps/* ../../fpsdk_apps/*/* ../../fpsdk_apps/*/*/* ../../fpsdk_apps/*/*/*/*))

.PHONY: default
default: build/.make-build

.PHONY: build
build: build/.make-build

build/.make-build: $(deps)
	mkdir -p build
	cmake -B build -S .
	cmake --build build --parallel 8
	touch $@

.PHONY: clean
clean:
	rm -rf build
