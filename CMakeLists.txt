cmake_minimum_required(VERSION 3.16)
include(fpsdk_common/cmake/setup.cmake)

project(fpsdk
    LANGUAGES CXX
    VERSION ${FPSDK_VERSION_NUMBER}
    DESCRIPTION "Fixposition SDK"
)

add_subdirectory(fpsdk_common)
if(FP_USE_ROS1)
    add_subdirectory(fpsdk_ros1)
endif()
if(FP_USE_ROS2)
    add_subdirectory(fpsdk_ros2)
endif()
add_subdirectory(fpsdk_apps)
