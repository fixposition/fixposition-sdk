# GENERAL ==============================================================================================================

cmake_minimum_required(VERSION 3.16)
include(cmake/setup.cmake)

project(fpcommon
    LANGUAGES CXX
    VERSION ${FP_VERSION_NUMBER}
    DESCRIPTION "Fixposition SDK: Common Library"
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
else()
    add_compile_definitions(BOOST_STACKTRACE_USE_BACKTRACE)
    add_compile_definitions(BOOST_STACKTRACE_USE_ADDR2LINE)
    set(FP_BOOST_STACKTRACE 1)
endif()
if(NOT CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_compile_definitions(NDEBUG)
endif()


# DEPENDENCIES =========================================================================================================

find_package(yaml-cpp REQUIRED)
if(FP_BOOST_STACKTRACE)
    find_package(Boost REQUIRED COMPONENTS REQUIRED stacktrace_basic stacktrace_backtrace stacktrace_addr2line)
else()
    find_package(Boost REQUIRED)
endif()
find_package(ZLIB REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(CapnProto 1.0.2 REQUIRED)
find_package(PROJ 9.4.1 REQUIRED CONFIG)

include_directories(include
    ${YAML_CPP_INCLUDE_DIR}
    ${Boost_INCLUDE_DIRS}
    ${ZLIB_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${CAPNP_INCLUDE_DIR}
)


# SHARED LIBRARY =======================================================================================================

file(GLOB CPP_FILES src/*.cpp)
add_library(${PROJECT_NAME} SHARED ${CPP_FILES})

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC
        ${YAML_CPP_LIBRARIES}
        ${EIGEN3_LIBRARIES}
    PRIVATE
        ${Boost_LIBRARIES} ${CMAKE_DL_LIBS}
        ${ZLIB_LIBRARIES}
        ${CAPNP_LIBRARIES}
        PROJ::proj
)


# INSTALL ==============================================================================================================

include(GNUInstallDirs) # Provides nice relative paths wrt CMAKE_INSTALL_PREFIX
set(PROJECT_LIBRARY_DIR ${CMAKE_INSTALL_FULL_LIBDIR})
set(PROJECT_INCLUDE_DIR ${CMAKE_INSTALL_FULL_INCLUDEDIR}/${PROJECT_NAME})

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

include(cmake/testing.cmake)

add_gtest(TARGET app_test       SOURCES test/app_test.cpp       LINK_LIBS ${PROJECT_NAME})
add_gtest(TARGET fpl_test       SOURCES test/fpl_test.cpp       LINK_LIBS ${PROJECT_NAME})
add_gtest(TARGET logging_test   SOURCES test/logging_test.cpp   LINK_LIBS ${PROJECT_NAME})
add_gtest(TARGET math_test      SOURCES test/math_test.cpp      LINK_LIBS ${PROJECT_NAME})
add_gtest(TARGET path_test      SOURCES test/path_test.cpp      LINK_LIBS ${PROJECT_NAME})
add_gtest(TARGET string_test    SOURCES test/string_test.cpp    LINK_LIBS ${PROJECT_NAME})
add_gtest(TARGET time_test      SOURCES test/time_test.cpp      LINK_LIBS ${PROJECT_NAME})
add_gtest(TARGET trafo_test     SOURCES test/trafo_test.cpp     LINK_LIBS ${PROJECT_NAME})
add_gtest(TARGET yaml_test      SOURCES test/yaml_test.cpp      LINK_LIBS ${PROJECT_NAME})

# ======================================================================================================================
