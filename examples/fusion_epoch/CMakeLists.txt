# GENERAL ==============================================================================================================

cmake_minimum_required(VERSION 3.16)

project(fusion_epoch
    LANGUAGES CXX
    VERSION 0.0.0
    DESCRIPTION "Fixposition SDK example: collecting fusion epoch data from FP_A messages"
)

set(FPSDK_BUILD_TESTING OFF)
set(FPSDK_USE_PROJ ON)


# COMPILER SETUP =======================================================================================================

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Werror \
    -Wshadow -Wunused-parameter -Wformat -Wpointer-arith -Woverloaded-virtual")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()
if(NOT CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_compile_definitions(NDEBUG)
endif()


# DEPENDENCIES =========================================================================================================

add_subdirectory(../../fpsdk_common fpsdk_common)


# EXECUTABLES ==========================================================================================================

add_executable(${PROJECT_NAME} fusion_epoch.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} fpsdk_common ${EIGEN3_LIBRARIES})


# ======================================================================================================================
