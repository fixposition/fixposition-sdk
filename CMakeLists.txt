cmake_minimum_required(VERSION 3.16)
include(fpcommon/cmake/setup.cmake)

project(fpsdk
    LANGUAGES CXX
    VERSION ${FP_VERSION_NUMBER}
    DESCRIPTION "Fixposition SDK"
)

add_subdirectory(fpcommon)
if(FP_USE_ROS1)
    add_subdirectory(fpros1)
endif()
add_subdirectory(fpapps)
