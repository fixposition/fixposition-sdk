cmake_minimum_required(VERSION 3.16)
include(fpsdk_common/cmake/setup.cmake)

project(fpsdk
    LANGUAGES CXX
    VERSION ${FPSDK_VERSION_NUMBER}
    DESCRIPTION "Fixposition SDK"
)

add_subdirectory(fpsdk_common)
add_subdirectory(fpsdk_apps)
