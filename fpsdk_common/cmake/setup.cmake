########################################################################################################################
# Some variables
set(FP_CMAKE_CMD_HELP "cmake -B build -DCMAKE_INSTALL_PREFIX=~/fpsdk")


########################################################################################################################
# (Somewhat) prevent in-source builds
get_filename_component(srcdir "${CMAKE_SOURCE_DIR}" REALPATH)
get_filename_component(bindir "${CMAKE_BINARY_DIR}" REALPATH)
message(STATUS "fpsdk: srcdir=${srcdir}")
message(STATUS "fpsdk: bindir=${bindir}")
if("${srcdir}" STREQUAL "${bindir}")
    message(FATAL_ERROR "\n\n ==> Aborting attempt to do a in-source build. Use '${FP_CMAKE_CMD_HELP}'\n\n")
endif()


########################################################################################################################
# Add install prefix to cmake path. Other fpsdk libs may be available there.
if(NOT "${CMAKE_INSTALL_PREFIX}" STREQUAL "")
    list(APPEND CMAKE_PREFIX_PATH ${CMAKE_INSTALL_PREFIX})
endif()


########################################################################################################################
# Check if we have ROS, and add to cmake path to that require(rosbag) works. User can either specify it as a cmake
# argument or we can detect it from the environment
# - ROS2 environment loaded
if("$ENV{ROS_VERSION}" STREQUAL "2")
    message(STATUS "fpsdk: Using ROS2")
    set(FPSDK_USE_ROS2 ON)
    # TODO: Maybe this also needs some stuff added to CMAKE_PREFIX_PATH?
# - ROS1 package path explicitly given on cmake command line. Not recommended. Enough to build, but not enough to run.
elseif(NOT "${ROS_PACKAGE_PATH}" STREQUAL "")
    message(STATUS "fpsdk: Using ROS1 (cmake arg ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH})")
    string(REPLACE ":" ";" ROS_PACKAGE_PATH_LIST ${ROS_PACKAGE_PATH})
    list(APPEND CMAKE_PREFIX_PATH ${ROS_PACKAGE_PATH_LIST})
    list(APPEND CMAKE_PREFIX_PATH /opt/ros/noetic)
    set(FPSDK_USE_ROS1 ON)
# - ROS1 environment loaded. Recommended.
elseif(NOT "$ENV{ROS_PACKAGE_PATH}" STREQUAL "")
    message(STATUS "fpsdk: Using ROS1 (environment ROS_PACKAGE_PATH=$ENV{ROS_PACKAGE_PATH})")
    # Environment (catkin) should already set CMAKE_PREFIX_PATH correctly. However, we seem to need this:
    list(APPEND CMAKE_PREFIX_PATH /opt/ros/noetic)
    set(FPSDK_USE_ROS1 ON)
else()
    message(STATUS "fpsdk: No ROS environment detected")
endif()


########################################################################################################################
# Get version info from GIT, unless explicitly given by the user (cmake command line)
# TODO: This is executed only once on configuration of the project. That's of course wrong. We'd have to execute this
# always, write/update some.hpp that can be included and properly dependency-managed. There are things like
# https://github.com/andrew-hardin/cmake-git-version-tracking/tree/master, but they're quite involved, too.

if (NOT FPSDK_VERSION_IS_SET) # Do this only once. E.g. when running the top-level CMakeLists.txt
    # - Version supplied on cmake command line (-DVERSION_STRING=x.x.x, -DVERSION_STRING=x.x.x-gggggg)
    if (FPSDK_VERSION_STRING)
        string(REGEX MATCH "^([0-9]+\.[0-9]+\.[0-9]+).*$" _ ${FPSDK_VERSION_STRING})
        if ("${CMAKE_MATCH_1}" STREQUAL "")
            message(FATAL_ERROR "fpsdk: bad FPSDK_VERSION_STRING=${FPSDK_VERSION_STRING}, must be x.x.x or x.x.x-ggggggg")
        endif()
        set(FPSDK_VERSION_NUMBER "${CMAKE_MATCH_1}")
        set(FPSDK_VERSION_STRING "${FPSDK_VERSION_STRING}")
    # - Version supplied from VERSION file (release tarballs), but ignore file if fpsdk is a git repo
    elseif(EXISTS ${CMAKE_CURRENT_LIST_DIR}/../../FPSDK_VERSION_NUMBER AND NOT EXISTS ${CMAKE_CURRENT_LIST_DIR}/../../.git)
        file(STRINGS ${CMAKE_CURRENT_LIST_DIR}/../../FPSDK_VERSION_NUMBER FPSDK_VERSION_NUMBER LIMIT_COUNT 1)
        set(FPSDK_VERSION_NUMBER "${FPSDK_VERSION_NUMBER}")
        file(STRINGS ${CMAKE_CURRENT_LIST_DIR}/../../FPSDK_VERSION_STRING FPSDK_VERSION_STRING LIMIT_COUNT 1)
        set(FPSDK_VERSION_STRING "${FPSDK_VERSION_STRING}")
    # - Version from git
    else()
        execute_process(
            COMMAND git -C ${CMAKE_CURRENT_LIST_DIR} describe --dirty --tags --always --exact-match --all --long
            OUTPUT_VARIABLE CMD_OUTPUT
            OUTPUT_STRIP_TRAILING_WHITESPACE
        )
        # tags/fp_9.16.0-0-g9db8f03                 at tag, clean
        # tags/fp_9.16.0-0-g9db8f03-dirty           at tag, dirty
        # heads/integ/master-0-geec9255             at commit, clean
        # heads/integ/master-0-geec9255-dirty       at commit, dirty

        # Anything dirty: version number 0.0.0 and string "0.0.0-dirty"
        if("${CMD_OUTPUT}" MATCHES "-dirty")
            set(FPSDK_VERSION_NUMBER 0.0.0)
            set(FPSDK_VERSION_STRING "${FPSDK_VERSION_NUMBER}-${CMD_OUTPUT}")
        # Tags that look like a version: version number x.y.z and string "x.y.z"
        elseif("${CMD_OUTPUT}" MATCHES "^tags/([a-z0-9]+_|v|)(0|[1-9][0-9]*).(0|[1-9][0-9]*).(0|[1-9][0-9]*)")
            set(FPSDK_VERSION_NUMBER "${CMAKE_MATCH_2}.${CMAKE_MATCH_3}.${CMAKE_MATCH_4}")
            set(FPSDK_VERSION_STRING "${FPSDK_VERSION_NUMBER}")
        # Anything else: version number 0.0.0 and string "0.0.0-whatevergitsaid"
        elseif(NOT "${CMD_OUTPUT}" STREQUAL "")
            set(FPSDK_VERSION_NUMBER 0.0.0)
            set(FPSDK_VERSION_STRING "${FPSDK_VERSION_NUMBER}-${CMD_OUTPUT}")
        # Git failed: version number 0.0.0 and string "0.0.0-dev"
        else()
            set(FPSDK_VERSION_NUMBER 0.0.0)
            set(FPSDK_VERSION_STRING "${FPSDK_VERSION_NUMBER}-dev")
        endif()
    endif()

    #add_compile_definitions(FPSDK_VERSION_NUMBER="${FPSDK_VERSION_NUMBER}")
    add_compile_definitions(FPSDK_VERSION_STRING="${FPSDK_VERSION_STRING}")

    # write version info to file
    file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/FPSDK_VERSION_NUMBER "${FPSDK_VERSION_NUMBER}")
    file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/FPSDK_VERSION_STRING "${FPSDK_VERSION_STRING}")

    set(FPSDK_VERSION_IS_SET ON)
endif()


########################################################################################################################
# Some debugging
message(STATUS "fpsdk: CMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}")
message(STATUS "fpsdk: CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}")
message(STATUS "fpsdk: CMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}")
message(STATUS "fpsdk: CMAKE_VERSION=${CMAKE_VERSION}")
message(STATUS "fpsdk: FPSDK_VERSION_NUMBER=${FPSDK_VERSION_NUMBER}")
message(STATUS "fpsdk: FPSDK_VERSION_STRING=${FPSDK_VERSION_STRING}")


########################################################################################################################

macro(fpsdk_save_versions)
    set(options)
    set(one_value_args FILE)
    set(multi_value_args PACKAGES)
    cmake_parse_arguments(FPSDK_SAVE_VERSIONS "${options}" "${one_value_args}" "${multi_value_args}" ${ARGN})

    list(SORT FPSDK_SAVE_VERSIONS_PACKAGES)
    list(REMOVE_DUPLICATES FPSDK_SAVE_VERSIONS_PACKAGES)

    message(STATUS "fpsdk: Saving versions to ${FPSDK_SAVE_VERSIONS_FILE}")
    write_file(${FPSDK_SAVE_VERSIONS_FILE} "# Versions for ${PROJECT_NAME}")

    foreach(_pkg ${FPSDK_SAVE_VERSIONS_PACKAGES})
        string(SUBSTRING "${_pkg}                    " 0 20 _name)
        write_file(${FPSDK_SAVE_VERSIONS_FILE} "${_name} ${${_pkg}_VERSION}" APPEND)
    endforeach()

endmacro()


########################################################################################################################
