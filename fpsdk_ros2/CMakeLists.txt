# GENERAL ==============================================================================================================

cmake_minimum_required(VERSION 3.16)
include(../fpsdk_common/cmake/setup.cmake)

project(fpsdk_ros2
    LANGUAGES CXX
    VERSION ${FPSDK_VERSION_NUMBER}
    DESCRIPTION "Fixposition SDK: ROS2 library"
)


# COMPILER SETUP =======================================================================================================

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Werror \
    -Wshadow -Wunused-parameter -Wformat -Wpointer-arith -Woverloaded-virtual")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
if(NOT CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_compile_definitions(NDEBUG)
endif()


# DEPENDENCIES =========================================================================================================

# Handle this being used as add_subdirectory() of ../CMakeLists.txt vs. building this standalone
if(NOT TARGET fpsdk_common)
    find_package(fpsdk_common REQUIRED)
endif()

set(FP_ROS2_INCLUDE_DIRS "")
set(FP_ROS2_LIBRARIES "")

find_package(rclcpp REQUIRED)
list(APPEND FP_ROS2_INCLUDE_DIRS ${rclcpp_INCLUDE_DIRS})
list(APPEND FP_ROS2_LIBRARIES ${rclcpp_LIBRARIES})


list(REMOVE_DUPLICATES FP_ROS2_INCLUDE_DIRS)
list(REMOVE_DUPLICATES FP_ROS2_LIBRARIES)

message(STATUS "fpsdk: FP_ROS2_INCLUDE_DIRS=${FP_ROS2_INCLUDE_DIRS}")
message(STATUS "fpsdk: FP_ROS2_LIBRARIES=${FP_ROS2_LIBRARIES}")


# SHARED LIBRARY =======================================================================================================

file(GLOB CPP_FILES src/*.cpp)
add_library(${PROJECT_NAME} SHARED ${CPP_FILES})

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        ${FP_ROS2_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC
        fpsdk_common
        ${FP_ROS2_LIBRARIES}
)

set_target_properties(${PROJECT_NAME}
    PROPERTIES
        VERSION ${PROJECT_VERSION}
)


# INSTALL ==============================================================================================================

include(GNUInstallDirs) # Provides nice relative paths wrt CMAKE_INSTALL_PREFIX
set(PROJECT_RUNTIME_DIR ${CMAKE_INSTALL_FULL_BINDIR})
set(PROJECT_LIBRARY_DIR ${CMAKE_INSTALL_FULL_LIBDIR})
set(PROJECT_INCLUDE_DIR ${CMAKE_INSTALL_FULL_INCLUDEDIR}/${PROJECT_NAME})
set(PROJECT_DATA_DIR    ${CMAKE_INSTALL_FULL_DATAROOTDIR}/${PROJECT_NAME})

# Headers
install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${PROJECT_INCLUDE_DIR}
)

# Library, tools
install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}-targets
    LIBRARY DESTINATION ${PROJECT_LIBRARY_DIR}
    RUNTIME DESTINATION ${PROJECT_RUNTIME_DIR}
)

# CMake target config
install(EXPORT ${PROJECT_NAME}-targets
    NAMESPACE ${PROJECT_NAMESPACE_PREFIX}
    FILE ${PROJECT_NAME}-targets.cmake
    DESTINATION lib/cmake/${PROJECT_NAME}
)

# CMake config
include(CMakePackageConfigHelpers)
set(TARGET1 ${PROJECT_NAME})
configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake
    INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
    NO_SET_AND_CHECK_MACRO
    NO_CHECK_REQUIRED_COMPONENTS_MACRO
)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake
    VERSION "${CMAKE_PROJECT_VERSION_MAJOR}.${CMAKE_PROJECT_VERSION_MINOR}"
    COMPATIBILITY AnyNewerVersion
)
install(
    FILES
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake
    DESTINATION lib/cmake/${PROJECT_NAME}
)

# pkg-config config
include(CMakePackageConfigHelpers)
set(TARGET1 ${PROJECT_NAME})
configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/config.pc.in
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.pc
    INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
    NO_SET_AND_CHECK_MACRO
    NO_CHECK_REQUIRED_COMPONENTS_MACRO
)
install(
    FILES
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.pc
    DESTINATION lib/pkgconfig
)


# TESTS ================================================================================================================

include(../fpsdk_common/cmake/testing.cmake)

add_gtest(TARGET utils_test     SOURCES test/utils_test.cpp     LINK_LIBS ${PROJECT_NAME})


# ======================================================================================================================
