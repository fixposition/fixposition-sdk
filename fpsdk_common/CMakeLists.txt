# GENERAL ==============================================================================================================
message(STATUS "fpsdk: ----- ${CMAKE_CURRENT_SOURCE_DIR} -----")

cmake_minimum_required(VERSION 3.16)
include(cmake/setup.cmake)
include(cmake/utils.cmake)

project(fpsdk_common
    LANGUAGES CXX C
    VERSION ${FPSDK_VERSION_NUMBER}
    DESCRIPTION "Fixposition SDK: Common Library"
)

set(_unused "${CMAKE_C_COMPILER}") # suppress warning in some setups

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
if(NOT CMAKE_BUILD_TYPE STREQUAL "Release")
    #add_compile_definitions(BOOST_STACKTRACE_USE_BACKTRACE)
    #add_compile_definitions(BOOST_STACKTRACE_USE_ADDR2LINE)
    add_compile_definitions(BOOST_STACKTRACE_DYN_LINK)
    set(FPSDK_BOOST_STACKTRACE 1)
endif()


# DEPENDENCIES =========================================================================================================

if(FPSDK_BOOST_STACKTRACE)
    if(FPSDK_LEGACY_BOOST_CMAKE) # e.g. Buildroot 2022.08
        find_package(Boost REQUIRED COMPONENTS stacktrace_backtrace)
    else()
        find_package(Boost CONFIG REQUIRED COMPONENTS stacktrace_backtrace)
    endif()
    set(Boost_TARGETS "Boost::headers;Boost::stacktrace_backtrace")
else()
    if(FPSDK_LEGACY_BOOST_CMAKE)
        find_package(Boost REQUIRED)
    else()
        find_package(Boost CONFIG REQUIRED)
    endif()
    set(Boost_TARGETS "Boost::headers")
endif()
find_package(yaml-cpp REQUIRED)
find_package(ZLIB REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_search_module(OpenSSL REQUIRED IMPORTED_TARGET openssl)
fpsdk_find_package_bz2()
fpsdk_find_package_proj()
fpsdk_find_package_ffmpeg()

if(FPSDK_USE_ROS1)
    fpsdk_cmake_find_ros1_package(roscpp)
    fpsdk_cmake_find_ros1_package(rosconsole)
    fpsdk_cmake_find_ros1_package(std_msgs)
    fpsdk_cmake_find_ros1_package(sensor_msgs)
    fpsdk_cmake_find_ros1_package(geometry_msgs)
    fpsdk_cmake_find_ros1_package(tf2_msgs)
    fpsdk_cmake_find_ros1_package(nav_msgs)
else()
    if(FPSDK_USE_ROS2)
        find_package(rclcpp REQUIRED)
        find_package(rosbag2_cpp REQUIRED)
        find_package(std_msgs REQUIRED)
        find_package(sensor_msgs REQUIRED)
        find_package(geometry_msgs REQUIRED)
        find_package(tf2_msgs REQUIRED)
        find_package(nav_msgs REQUIRED)
    endif()

    # Minimal ROS1 stuff needed for fpltool
    fpsdk_cmake_local_ros1_package(std_msgs)
    fpsdk_cmake_local_ros1_package(sensor_msgs)
    fpsdk_cmake_local_ros1_package(geometry_msgs)
    fpsdk_cmake_local_ros1_package(tf2_msgs)
    fpsdk_cmake_local_ros1_package(nav_msgs)
endif()

fpsdk_save_versions(FILE ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_versions.txt
    PACKAGES CMAKE Boost yaml-cpp ZLIB BZIP2 Eigen3 nlohmann_json OpenSSL PROJ libavcodec libavutil libavfilter libswscale)

# SHARED LIBRARY =======================================================================================================

file(GLOB CPP_FILES src/*.cpp src/parser/*.cpp src/stream/*.cpp)

# Unless building on a real ROS1 system we have to build some minimal implementation. Most things are header-only,
# though. See the fpsdk_cmake_local_ros1_package() and the setup for the rosnoros/ headers below.
if(NOT FPSDK_USE_ROS1)
    file(GLOB ROSNOROS_CPP_FILES rosnoros/ros/*.cpp)
    list(APPEND CPP_FILES ${ROSNOROS_CPP_FILES})
endif()

add_library(${PROJECT_NAME} SHARED ${CPP_FILES})

if(FPSDK_USE_ROS1)
    target_compile_definitions(${PROJECT_NAME} PUBLIC FPSDK_USE_ROS1=1)
else()
    target_compile_definitions(${PROJECT_NAME} PUBLIC FPSDK_USE_ROS1=0)
endif()

if(FPSDK_USE_ROS2)
    target_compile_definitions(${PROJECT_NAME} PUBLIC FPSDK_USE_ROS2=1)
else()
    target_compile_definitions(${PROJECT_NAME} PUBLIC FPSDK_USE_ROS2=0)
endif()

if(FPSDK_USE_BZ2)
    target_compile_definitions(${PROJECT_NAME} PUBLIC FPSDK_USE_BZ2=1)
else()
    target_compile_definitions(${PROJECT_NAME} PUBLIC FPSDK_USE_BZ2=0)
endif()

if(FPSDK_USE_PROJ)
    target_compile_definitions(${PROJECT_NAME} PUBLIC FPSDK_USE_PROJ=1)
else()
    target_compile_definitions(${PROJECT_NAME} PUBLIC FPSDK_USE_PROJ=0)
endif()

if(FPSDK_USE_FFMPEG)
    target_compile_definitions(${PROJECT_NAME} PUBLIC FPSDK_USE_FFMPEG=1)
else()
    target_compile_definitions(${PROJECT_NAME} PUBLIC FPSDK_USE_FFMPEG=0)
endif()

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
)

if(NOT FPSDK_USE_ROS1)
    target_include_directories(${PROJECT_NAME}
        PUBLIC
            $<INSTALL_INTERFACE:include/${PROJECT_NAME}/rosnoros>
            $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/rosnoros>
    )
endif()

set(FPSDK_COMMON_TRANS_DEPS "Eigen3;yaml-cpp")

target_link_libraries(${PROJECT_NAME}
    PUBLIC
        Eigen3::Eigen
        yaml-cpp
        Threads::Threads
    PRIVATE
        $<TARGET_NAME_IF_EXISTS:ros1::rosconsole>
        $<TARGET_NAME_IF_EXISTS:ros1::roscpp>
        $<TARGET_NAME_IF_EXISTS:ros1::std_msgs>
        $<TARGET_NAME_IF_EXISTS:ros1::geometry_msgs>
        $<TARGET_NAME_IF_EXISTS:ros1::sensor_msgs>
        $<TARGET_NAME_IF_EXISTS:ros1::tf2_msgs>
        $<TARGET_NAME_IF_EXISTS:ros1::nav_msgs>
        ${Boost_TARGETS}
        ${CMAKE_DL_LIBS}
        ZLIB::ZLIB
        PkgConfig::OpenSSL
        nlohmann_json::nlohmann_json
        $<$<BOOL:${FPSDK_USE_PROJ}>:PROJ::proj>
        $<$<BOOL:${FPSDK_USE_FFMPEG}>:PkgConfig::libavcodec>
        $<$<BOOL:${FPSDK_USE_FFMPEG}>:PkgConfig::libavutil>
        $<$<BOOL:${FPSDK_USE_FFMPEG}>:PkgConfig::libavfilter>
        $<$<BOOL:${FPSDK_USE_FFMPEG}>:PkgConfig::libswscale>
        $<$<BOOL:${FPSDK_USE_BZ2}>:BZip2::BZip2>
        ${rclcpp_TARGETS}
        ${rosbag2_cpp_TARGETS}
        ${std_msgs_TARGETS}
        ${sensor_msgs_TARGETS}
        ${geometry_msgs_TARGETS}
        ${tf2_msgs_TARGETS}
        ${nav_msgs_TARGETS}
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
set(PROJECT_DOC_DIR     ${CMAKE_INSTALL_FULL_DOCDIR})

# Headers
install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${PROJECT_INCLUDE_DIR}
)

install(DIRECTORY rosnoros
    DESTINATION ${PROJECT_INCLUDE_DIR}
    FILES_MATCHING PATTERN "*.h"
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

# Documentation
install(
    FILES
        ${PROJECT_SOURCE_DIR}/README.md
        ${PROJECT_SOURCE_DIR}/LICENSE
    DESTINATION ${PROJECT_DOC_DIR}
)


# TESTS ================================================================================================================

include(cmake/testing.cmake)

add_gtest(TARGET app_test             SOURCES test/app_test.cpp             LINK_LIBS ${PROJECT_NAME})
add_gtest(TARGET can_test             SOURCES test/can_test.cpp             LINK_LIBS ${PROJECT_NAME})
add_gtest(TARGET cam_test             SOURCES test/cam_test.cpp             LINK_LIBS ${PROJECT_NAME})
add_gtest(TARGET fpl_test             SOURCES test/fpl_test.cpp             LINK_LIBS ${PROJECT_NAME})
add_gtest(TARGET logging_test         SOURCES test/logging_test.cpp         LINK_LIBS ${PROJECT_NAME})
add_gtest(TARGET gnss_test            SOURCES test/gnss_test.cpp            LINK_LIBS ${PROJECT_NAME})
add_gtest(TARGET math_test            SOURCES test/math_test.cpp            LINK_LIBS ${PROJECT_NAME})
add_gtest(TARGET parser_test          SOURCES test/parser_test.cpp          LINK_LIBS ${PROJECT_NAME})
add_gtest(TARGET parser_crc_test      SOURCES test/parser_crc_test.cpp      LINK_LIBS ${PROJECT_NAME})
add_gtest(TARGET parser_fpa_test      SOURCES test/parser_fpa_test.cpp      LINK_LIBS ${PROJECT_NAME})
add_gtest(TARGET parser_fpb_test      SOURCES test/parser_fpb_test.cpp      LINK_LIBS ${PROJECT_NAME})
add_gtest(TARGET parser_nmea_test     SOURCES test/parser_nmea_test.cpp     LINK_LIBS ${PROJECT_NAME})
add_gtest(TARGET parser_novb_test     SOURCES test/parser_novb_test.cpp     LINK_LIBS ${PROJECT_NAME})
add_gtest(TARGET parser_qgc_test      SOURCES test/parser_qgc_test.cpp      LINK_LIBS ${PROJECT_NAME})
add_gtest(TARGET parser_rtcm3_test    SOURCES test/parser_rtcm3_test.cpp    LINK_LIBS ${PROJECT_NAME})
add_gtest(TARGET parser_sbf_test      SOURCES test/parser_sbf_test.cpp      LINK_LIBS ${PROJECT_NAME})
add_gtest(TARGET parser_spartn_test   SOURCES test/parser_spartn_test.cpp   LINK_LIBS ${PROJECT_NAME})
add_gtest(TARGET parser_types_test    SOURCES test/parser_types_test.cpp    LINK_LIBS ${PROJECT_NAME})
add_gtest(TARGET parser_ubx_test      SOURCES test/parser_ubx_test.cpp      LINK_LIBS ${PROJECT_NAME})
add_gtest(TARGET parser_unib_test     SOURCES test/parser_unib_test.cpp     LINK_LIBS ${PROJECT_NAME})
add_gtest(TARGET path_test            SOURCES test/path_test.cpp            LINK_LIBS ${PROJECT_NAME})
add_gtest(TARGET ros1_test            SOURCES test/ros1_test.cpp            LINK_LIBS ${PROJECT_NAME} $<TARGET_NAME_IF_EXISTS:ros1::roscpp> $<TARGET_NAME_IF_EXISTS:ros1::rosconsole> $<TARGET_NAME_IF_EXISTS:ros1::std_msgs> $<TARGET_NAME_IF_EXISTS:ros1::geometry_msgs> $<TARGET_NAME_IF_EXISTS:ros1::sensor_msgs> $<TARGET_NAME_IF_EXISTS:ros1::tf2_msgs> $<TARGET_NAME_IF_EXISTS:ros1::nav_msgs> $<$<BOOL:${FPSDK_USE_BZ2}>:BZip2::BZip2>)
add_gtest(TARGET ros2_test            SOURCES test/ros2_test.cpp            LINK_LIBS ${PROJECT_NAME} ${rclcpp_TARGETS} ${rosbag2_cpp_TARGETS} ${std_msgs_TARGETS} ${sensor_msgs_TARGETS} ${geometry_msgs_TARGETS} ${tf2_msgs_TARGETS} ${nav_msgs_TARGETS})
add_gtest(TARGET string_test          SOURCES test/string_test.cpp          LINK_LIBS ${PROJECT_NAME})
add_gtest(TARGET time_test            SOURCES test/time_test.cpp            LINK_LIBS ${PROJECT_NAME})
add_gtest(TARGET thread_test          SOURCES test/thread_test.cpp          LINK_LIBS ${PROJECT_NAME})
add_gtest(TARGET trafo_test           SOURCES test/trafo_test.cpp           LINK_LIBS ${PROJECT_NAME})
add_gtest(TARGET types_test           SOURCES test/types_test.cpp           LINK_LIBS ${PROJECT_NAME})
add_gtest(TARGET utils_test           SOURCES test/utils_test.cpp           LINK_LIBS ${PROJECT_NAME})
add_gtest(TARGET video_test           SOURCES test/video_test.cpp           LINK_LIBS ${PROJECT_NAME})
add_gtest(TARGET yaml_test            SOURCES test/yaml_test.cpp            LINK_LIBS ${PROJECT_NAME})

# ======================================================================================================================
