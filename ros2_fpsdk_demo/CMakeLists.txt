# GENERAL ==============================================================================================================

cmake_minimum_required(VERSION 3.16)

project(ros2_fpsdk_demo
    LANGUAGES CXX
    VERSION 1.0.0
    DESCRIPTION "Fixposition SDK: ROS2 Demo"
)


# COMPILER SETUP =======================================================================================================

# TODO: The ament stuff overrides some of these...?
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic\
    -Wshadow -Wunused-parameter -Wformat -Wpointer-arith\
    -Werror")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
if(NOT CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_compile_definitions(NDEBUG)
endif()


# DEPENDENCIES =========================================================================================================

find_package(ament_cmake REQUIRED) # https://docs.ros.org/en/humble/How-To-Guides/Ament-CMake-Documentation.html
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(fpsdk_common REQUIRED)
find_package(fpsdk_ros2 REQUIRED)

include_directories(include ${rclcpp_INCLUDE_DIRS} ${rmw_INCLUDE_DIRS} ${fpsdk_common_INCLUDE_DIRS} ${fpsdk_ros2_INCLUDE_DIRS})


# SHARED LIBRARY =======================================================================================================

add_library(${PROJECT_NAME}_lib SHARED
    src/node.cpp
)

#target_link_libraries(${PROJECT_NAME}_lib fpsdk_common fpsdk_ros2)
ament_target_dependencies(${PROJECT_NAME}_lib rclcpp fpsdk_common fpsdk_ros2 std_msgs)


# EXECUTABLES ==========================================================================================================

add_executable(${PROJECT_NAME}_node
    src/node_main.cpp
)

target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_lib fpsdk_common fpsdk_ros2)
ament_target_dependencies(${PROJECT_NAME}_node rclcpp fpsdk_common fpsdk_ros2)

install(TARGETS
    ${PROJECT_NAME}_lib ${PROJECT_NAME}_node
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY
    "launch"
    DESTINATION share/${PROJECT_NAME}/
)

# TESTS ================================================================================================================

# ...

# ======================================================================================================================
ament_package()
