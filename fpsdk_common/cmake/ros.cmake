
# Find ROS1 package and create a target for it
macro(fpsdk_cmake_find_ros1_package ROS_PACKAGE)
    if(NOT TARGET ros1::${ROS_PACKAGE})
        # Create a target for the ROS library
        add_library(ros1_${ROS_PACKAGE} INTERFACE)
        add_library(ros1::${ROS_PACKAGE} ALIAS ros1_${ROS_PACKAGE})
        find_package(${ROS_PACKAGE} REQUIRED)
        target_include_directories(ros1_${ROS_PACKAGE} INTERFACE ${${ROS_PACKAGE}_INCLUDE_DIRS})
        target_link_libraries(ros1_${ROS_PACKAGE} INTERFACE ${${ROS_PACKAGE}_LIBRARIES})
    endif()
endmacro()

# Use shipped ROS1 "packages" (only for fpsdk_common!)
macro(fpsdk_cmake_local_ros1_package ROS_PACKAGE)
    if(NOT TARGET ros1::${ROS_PACKAGE})
        add_library(ros1_${ROS_PACKAGE} INTERFACE)
        add_library(ros1::${ROS_PACKAGE} ALIAS ros1_${ROS_PACKAGE})
        target_include_directories(ros1_${ROS_PACKAGE} INTERFACE ${PROJECT_SOURCE_DIR}/rosnoros)
    endif()
endmacro()
