include(CheckCSourceRuns)
include(CMakePushCheckState)

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

########################################################################################################################

# PROJ is optional, unless explicitly requested with -DFPSDK_USE_PROJ=ON
macro(fpsdk_find_package_proj)
    # message(STATUS "fpsdk: FPSDK_USE_PROJ=${FPSDK_USE_PROJ}")

    # Explicitly requested by user to use proj
    if(FPSDK_USE_PROJ STREQUAL "ON")
        find_package(PROJ 9.4 REQUIRED CONFIG)

    # Explicitly requested by user to not use proj
    elseif(FPSDK_USE_PROJ STREQUAL "OFF")
        message(STATUS "fpsdk: Not using PROJ")

    # Automatic, use proj if a suitable version is available
    else()
       find_package(PROJ 9.4 CONFIG QUIET)
        if(${PROJ_FOUND})
            set(FPSDK_USE_PROJ ON)
        else()
            message(STATUS "fpsdk: No PROJ found")
            set(FPSDK_USE_PROJ OFF)
        endif()
    endif()

    if(FPSDK_USE_PROJ)
        message(STATUS "fpsdk: Using PROJ (${PROJ_VERSION}, ${PROJ_INCLUDE_DIRS})")
    endif()

endmacro()

########################################################################################################################

# Check that FFmpeg is not built with --enable-gpl or --enable-nonfree, that is, it is built with
# --disable-gpl and --disable-nonfree. See docker/scripts/install_ffmpeg.sh
macro(fpsdk_check_ffmpeg_is_lgpl)
    cmake_push_check_state(RESET)

    set(CMAKE_REQUIRED_LIBRARIES        ${libavutil_LIBRARIES})
    set(CMAKE_REQUIRED_INCLUDES         ${libavutil_INCLUDEDIR})
    set(CMAKE_REQUIRED_LINK_DIRECTORIES ${libavutil_LIBRARY_DIRS})
    message(STATUS "fpsdk: Checking FFmpeg libraries (${libavutil_LIBRARY_DIRS})")
    check_c_source_runs("
        #include <libavutil/avutil.h>
        #include <string.h>
        #include <stdio.h>
        int main() {
            const char *config = avutil_configuration();
            printf(\"avutil_configuration: %s%c\", config, 0x0a);
            if (strstr(config, \"--enable-gpl\")     != NULL) { return 1; }
            if (strstr(config, \"--enable-nonfree\") != NULL) { return 1; }
            return 0;
        }
    " FFMPEG_IS_LGPL)

    if (NOT FFMPEG_IS_LGPL)
        message(STATUS "fpsdk: The FFmpeg libraries in ${libavutil_LIBRARY_DIRS} are not LGPL.")
    else()
        message(STATUS "fpsdk: The FFmpeg libraries in ${libavutil_LIBRARY_DIRS} are LGPL.")
    endif()

    cmake_pop_check_state()
endmacro()

# FFmpeg is optional, unless explicitly requested with -DFPSDK_USE_FFMPEG=ON
macro(fpsdk_find_package_ffmpeg)
    # message(STATUS "fpsdk: FPSDK_USE_FFMPEG=${FPSDK_USE_FFMPEG}")

    # Explicitly requested by user to use FFmpeg libraries
    if(FPSDK_USE_FFMPEG STREQUAL "ON")
        find_package(PkgConfig REQUIRED)
        pkg_search_module(libavcodec  REQUIRED IMPORTED_TARGET libavcodec)
        pkg_search_module(libavutil   REQUIRED IMPORTED_TARGET libavutil)
        pkg_search_module(libavfilter REQUIRED IMPORTED_TARGET libavfilter)
        pkg_search_module(libswscale  REQUIRED IMPORTED_TARGET libswscale)
        fpsdk_check_ffmpeg_is_lgpl()

    # Explicitly requested by user to not use FFmpeg libraries
    elseif(FPSDK_USE_FFMPEG STREQUAL "OFF")
        message(STATUS "fpsdk: Not using FFmpeg libraries")
        set(FPSDK_USE_FFMPEG OFF)
    # Automatic, use FFmpeg if available (and free))
    else()
        find_package(PkgConfig REQUIRED)
        pkg_search_module(libavcodec  QUIET IMPORTED_TARGET libavcodec)
        pkg_search_module(libavutil   QUIET IMPORTED_TARGET libavutil)
        pkg_search_module(libavfilter QUIET IMPORTED_TARGET libavfilter)
        pkg_search_module(libswscale  QUIET IMPORTED_TARGET libswscale)
        if(libavcodec_FOUND AND libavutil_FOUND AND libavfilter_FOUND AND libswscale_FOUND)
            fpsdk_check_ffmpeg_is_lgpl()
            if(NOT FFMPEG_IS_LGPL)
                set(FPSDK_USE_FFMPEG OFF)
            else()
                set(FPSDK_USE_FFMPEG ON)
            endif()
        else()
            set(FPSDK_USE_FFMPEG OFF)
        endif()

        if(NOT FPSDK_USE_FFMPEG)
            message(STATUS "fpsdk: No (suitable) FFmpeg libraries found")
        endif()

    endif()

    # Refuse to continue if FFmpeg is enabled but it's a non-free version
    if(FPSDK_USE_FFMPEG STREQUAL "ON")
        if(NOT FFMPEG_IS_LGPL)
            message(FATAL_ERROR "fpsdk: The FFmpeg libraries in ${libavutil_LIBRARY_DIRS} \
                are not LGPL. Please either disable the use of FFmpeg libraries (-DFPSDK_USE_FFMPEG=OFF) \
                or provide suitably configured and compiled FFmpeg libraries (-DCMAKE_PREFIX_PATH=/path/to/ffmpeg).")
        endif()
    endif()

    if(FPSDK_USE_FFMPEG)
        message(STATUS "fpsdk: Using FFmpeg (libavcodec ${libavcodec_VERSION}, ${libavcodec_INCLUDEDIR})")
        message(STATUS "fpsdk: Using FFmpeg (libavutil ${libavutil_VERSION}, ${libavutil_INCLUDEDIR})")
        message(STATUS "fpsdk: Using FFmpeg (libavfilter ${libavfilter_VERSION}, ${libavfilter_INCLUDEDIR})")
        message(STATUS "fpsdk: Using FFmpeg (libswscale ${libswscale_VERSION}, ${libswscale_INCLUDEDIR})")
    endif()

endmacro()
