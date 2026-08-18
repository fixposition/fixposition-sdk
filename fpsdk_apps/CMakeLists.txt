# GENERAL ==============================================================================================================
message(STATUS "fpsdk: ----- ${CMAKE_CURRENT_SOURCE_DIR} -----")

cmake_minimum_required(VERSION 3.16)
include(../fpsdk_common/cmake/setup.cmake)
include(../fpsdk_common/cmake/utils.cmake)

project(fpsdk_apps
    LANGUAGES CXX C
    VERSION ${FPSDK_VERSION_NUMBER}
    DESCRIPTION "Fixposition SDK: Apps"
)

set(_unused "${FPSDK_BUILD_TESTING}") # suppress warning in some setups
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


# DEPENDENCIES =========================================================================================================

# Handle this being used as add_subdirectory() of ../CMakeLists.txt vs. building this standalone
if(NOT TARGET fpsdk_common)
    find_package(fpsdk_common REQUIRED)
endif()

if(FPSDK_USE_ROS1)
    fpsdk_cmake_find_ros1_package(roscpp)
    fpsdk_cmake_find_ros1_package(rosconsole)
    fpsdk_cmake_find_ros1_package(std_msgs)
    fpsdk_cmake_find_ros1_package(sensor_msgs)
    fpsdk_cmake_find_ros1_package(geometry_msgs)
    fpsdk_cmake_find_ros1_package(tf2_msgs)
    fpsdk_cmake_find_ros1_package(nav_msgs)
elseif(FPSDK_USE_ROS2)
    find_package(rclcpp REQUIRED)
    find_package(rosbag2_cpp REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(geometry_msgs REQUIRED)
    find_package(tf2_msgs REQUIRED)
    find_package(nav_msgs REQUIRED)
else()
    message(STATUS "fpsdk_apps: No ROS available")
endif()

find_package(Threads REQUIRED)
find_package(nlohmann_json REQUIRED)


# SHARED LIBRARY =======================================================================================================

# https://gitlab.kitware.com/cmake/community/-/wikis/doc/cmake/RPATH-handling#recommendations
list(APPEND CMAKE_INSTALL_RPATH $ORIGIN)
list(APPEND CMAKE_INSTALL_RPATH $ORIGIN/../lib)


# EXECUTABLES ==========================================================================================================

file(GLOB COMMON_CPP_FILES common/*.cpp)

file(GLOB FPLTOOL_CPP_FILES fpltool/*.cpp)
add_executable(fpltool ${FPLTOOL_CPP_FILES})
target_link_libraries(fpltool
    PRIVATE
        fpsdk_common
        $<TARGET_NAME_IF_EXISTS:ros1::rosconsole>
        $<TARGET_NAME_IF_EXISTS:ros1::roscpp>
        $<TARGET_NAME_IF_EXISTS:ros1::std_msgs>
        $<TARGET_NAME_IF_EXISTS:ros1::geometry_msgs>
        $<TARGET_NAME_IF_EXISTS:ros1::sensor_msgs>
        $<TARGET_NAME_IF_EXISTS:ros1::tf2_msgs>
        $<TARGET_NAME_IF_EXISTS:ros1::nav_msgs>
        ${rclcpp_TARGETS}
        ${rosbag2_cpp_TARGETS}
        ${std_msgs_TARGETS}
        ${sensor_msgs_TARGETS}
        ${geometry_msgs_TARGETS}
        ${tf2_msgs_TARGETS}
        ${nav_msgs_TARGETS}
)

file(GLOB PARSERTOOL_CPP_FILES parsertool/*.cpp)
add_executable(parsertool ${PARSERTOOL_CPP_FILES} ${COMMON_CPP_FILES})
target_link_libraries(parsertool
    PRIVATE
        fpsdk_common
)

file(GLOB TIMECONV_CPP_FILES timeconv/*.cpp)
add_executable(timeconv ${TIMECONV_CPP_FILES} ${COMMON_CPP_FILES})
target_link_libraries(timeconv
    PRIVATE
        fpsdk_common
)

file(GLOB TIMETOOL_CPP_FILES timetool/*.cpp)
add_executable(timetool ${TIMETOOL_CPP_FILES} ${COMMON_CPP_FILES})
target_link_libraries(timetool
    PRIVATE
        fpsdk_common
)

file(GLOB YAML2SHELL_CPP_FILES yaml2shell/*.cpp)
add_executable(yaml2shell ${YAML2SHELL_CPP_FILES} ${COMMON_CPP_FILES})
target_link_libraries(yaml2shell
    PRIVATE
        fpsdk_common
)

file(GLOB FINDSENSOR_CPP_FILES findsensor/*.cpp)
add_executable(findsensor ${FINDSENSOR_CPP_FILES} ${COMMON_CPP_FILES})
target_link_libraries(findsensor
    PRIVATE
        fpsdk_common
        Threads::Threads
        nlohmann_json::nlohmann_json
)

# INSTALL ==============================================================================================================

include(GNUInstallDirs) # Provides nice relative paths wrt CMAKE_INSTALL_PREFIX
set(PROJECT_RUNTIME_DIR ${CMAKE_INSTALL_FULL_BINDIR})
set(PROJECT_LIBRARY_DIR ${CMAKE_INSTALL_FULL_LIBDIR})
set(PROJECT_INCLUDE_DIR ${CMAKE_INSTALL_FULL_INCLUDEDIR}/${PROJECT_NAME})
set(PROJECT_DATA_DIR    ${CMAKE_INSTALL_FULL_DATAROOTDIR}/${PROJECT_NAME})
set(PROJECT_DOC_DIR     ${CMAKE_INSTALL_FULL_DOCDIR})

install(TARGETS fpltool parsertool timeconv timetool yaml2shell findsensor
    EXPORT ${PROJECT_NAME}-targets
    LIBRARY DESTINATION ${PROJECT_LIBRARY_DIR}
    RUNTIME DESTINATION ${PROJECT_RUNTIME_DIR}
)

# Documentation
install(
    FILES
        ${PROJECT_SOURCE_DIR}/README.md
        ${PROJECT_SOURCE_DIR}/LICENSE
    DESTINATION ${PROJECT_DOC_DIR}
)


# ======================================================================================================================
