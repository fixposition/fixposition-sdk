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
if(NOT "${ROS_PACKAGE_PATH}" STREQUAL "")
    message(STATUS "fpsdk: Using ROS1 (cmake arg ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH})")
    list(APPEND CMAKE_PREFIX_PATH ${ROS_PACKAGE_PATH})
    set(FP_USE_ROS1 ON)
elseif(NOT "$ENV{ROS_PACKAGE_PATH}" STREQUAL "")
    message(STATUS "fpsdk: Using ROS1 (environment ROS_PACKAGE_PATH=$ENV{ROS_PACKAGE_PATH})")
    list(APPEND CMAKE_PREFIX_PATH $ENV{ROS_PACKAGE_PATH})
    set(FP_USE_ROS1 ON)
else()
    message(STATUS "fpsdk: No ROS environment detected")
endif()


########################################################################################################################
# Get version info from GIT, unless explicitly given by the user (cmake command line)
# TODO: This is executed only once on configuration of the project. That's of course wrong. We'd have to execute this
# always, write/update some.hpp that can be included and properly dependency-managed. There are things like
# https://github.com/andrew-hardin/cmake-git-version-tracking/tree/master, but they're quite involved, too. So perhaps
# we should just supply -DFP_VERSION_NUMBER it by the release CI and otherwise just use 0.0.0

if (NOT FP_VERSION_IS_SET) # Do this only once. E.g. when running the top-level CMakeLists.txt
    if (FP_VERSION_NUMBER)
        set(FP_VERSION_STRING "${FP_VERSION_NUMBER}")
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
            set(FP_VERSION_NUMBER 0.0.0)
            set(FP_VERSION_STRING "${FP_VERSION_NUMBER}-${CMD_OUTPUT}")
        # Tags that look like a version: version number x.y.z and string "x.y.z"
        elseif("${CMD_OUTPUT}" MATCHES "^tags/([a-z0-9]+_|v|)(0|[1-9][0-9]*).(0|[1-9][0-9]*).(0|[1-9][0-9]*)")
            set(FP_VERSION_NUMBER "${CMAKE_MATCH_2}.${CMAKE_MATCH_3}.${CMAKE_MATCH_4}")
            set(FP_VERSION_STRING "${FP_VERSION_NUMBER}")
        # Anything else: version number 0.0.0 and string "0.0.0-whatevergitsaid"
        elseif(NOT "${CMD_OUTPUT}" STREQUAL "")
            set(FP_VERSION_NUMBER 0.0.0)
            set(FP_VERSION_STRING "${FP_VERSION_NUMBER}-${CMD_OUTPUT}")
        # Git failed: version number 0.0.0 and string "0.0.0-dev"
        else()
            set(FP_VERSION_NUMBER 0.0.0)
            set(FP_VERSION_STRING "${FP_VERSION_NUMBER}-dev")
        endif()
    endif()

    #add_compile_definitions(FP_VERSION_NUMBER="${FP_VERSION_NUMBER}")
    add_compile_definitions(FP_VERSION_STRING="${FP_VERSION_STRING}")

    # write version info to file
    file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/FP_VERSION_STRING "${FP_VERSION_STRING}")

    set(FP_VERSION_IS_SET ON)
endif()


########################################################################################################################
# Some debugging
message(STATUS "fpsdk: CMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}")
message(STATUS "fpsdk: CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}")
message(STATUS "fpsdk: FP_VERSION_NUMBER=${FP_VERSION_NUMBER}")
message(STATUS "fpsdk: FP_VERSION_STRING=${FP_VERSION_STRING}")


########################################################################################################################
