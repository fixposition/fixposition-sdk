# GENERAL ==============================================================================================================

cmake_minimum_required(VERSION 3.16)
include(../fpcommon/cmake/setup.cmake)

project(fpros2
    LANGUAGES CXX
    VERSION ${FP_VERSION_NUMBER}
    DESCRIPTION "Fixposition SDK: ROS2 library"
)


# COMPILER SETUP =======================================================================================================

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic\
    -Wshadow -Wunused-parameter -Wformat -Wpointer-arith\
    -Werror")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
if (CMAKE_BUILD_TYPE STREQUAL "Release")
    add_compile_definitions(NDEBUG)
endif()
if(NOT CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_compile_definitions(NDEBUG)
endif()


# DEPENDENCIES =========================================================================================================

# https://docs.ros.org/en/humble/How-To-Guides/Ament-CMake-Documentation.html#adding-files-and-headers
find_package(ament_cmake REQUIRED)

# Handle this being used as add_subdirectory() of ../CMakeLists.txt vs. building this standalone
if(NOT TARGET fpcommon)
    find_package(fpcommon REQUIRED)
endif()

find_package(rclcpp REQUIRED)


# SHARED LIBRARY =======================================================================================================

file(GLOB CPP_FILES src/*.cpp)
add_library(${PROJECT_NAME} SHARED ${CPP_FILES})

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
)

# target_link_libraries(${PROJECT_NAME}
#     PUBLIC
#         fpcommon
#         ${FP_ROS2_LIBRARIES}
# )

ament_target_dependencies(${PROJECT_NAME} fpcommon rclcpp)


# INSTALL ==============================================================================================================

# TODO


# TESTS ================================================================================================================

include(../fpcommon/cmake/testing.cmake)

add_gtest(TARGET utils_test     SOURCES test/utils_test.cpp     LINK_LIBS ${PROJECT_NAME})


# ======================================================================================================================

ament_package()
