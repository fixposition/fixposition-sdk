# GENERAL ==============================================================================================================
message(STATUS "fpsdk: ----- ${CMAKE_CURRENT_SOURCE_DIR} -----")

cmake_minimum_required(VERSION 3.16)

project(ros1_fpsdk_demo
    LANGUAGES CXX
    VERSION 1.0.0
    DESCRIPTION "Fixposition SDK: ROS1 Demo"
)


# COMPILER SETUP =======================================================================================================

set(CMAKE_CXX_STANDARD 17)
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

find_package(fpsdk_common REQUIRED)
find_package(fpsdk_ros1 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rosbag
    std_msgs
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}_lib
    CATKIN_DEPENDS
    roscpp
    std_msgs
)


# SHARED LIBRARY =======================================================================================================

add_library(${PROJECT_NAME}_lib SHARED
    src/node.cpp
)

target_link_libraries(${PROJECT_NAME}_lib ${catkin_LIBRARIES} fpsdk_common fpsdk_ros1)

target_include_directories(${PROJECT_NAME}_lib
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        ${catkin_INCLUDE_DIRS}
)


# EXECUTABLES ==========================================================================================================

add_executable(${PROJECT_NAME}_node
    src/node_main.cpp
)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_lib ${catkin_LIBRARIES} fpsdk_common fpsdk_ros1)


# INSTALL ==============================================================================================================

install(TARGETS ${PROJECT_NAME}_lib ${PROJECT_NAME}_node
    EXPORT ${PROJECT_NAME}_Targets
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY
    "launch"
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


# TESTS ================================================================================================================

if(CATKIN_ENABLE_TESTING)

# ...

endif()

# ======================================================================================================================
