# Override the standard BUILD_TESTING variable
if (DEFINED FPSDK_BUILD_TESTING)
    message(STATUS "fpsdk: FPSDK_BUILD_TESTING=${FPSDK_BUILD_TESTING}")
    set(BUILD_TESTING ${FPSDK_BUILD_TESTING})
endif()

# User requested to build testing (-DBUILD_TESTING=ON), abort if no suitable version available
if (BUILD_TESTING STREQUAL "ON")

    find_package(GTest 1.12.0 REQUIRED)
    # GTest doesn't seem to have a normal cmake config file, so we have to check and abort ourselves.. :-/
    if ("${GTest_VERSION}" STREQUAL "")
        message(FATAL_ERROR "Unsupported GTest version")
    endif()
    message(STATUS "fpsdk: Using GTest (${GTest_VERSION})")
    set(BUILD_TESTING ON)

# User requested no testing (-DBUILD_TESTING=OFF)
elseif(BUILD_TESTING STREQUAL "OFF")

    message(STATUS "fpsdk: testing disabled")
    set(BUILD_TESTING OFF)

# Automatically detect if a suitable GTest library is available
else()

    find_package(GTest 1.12.0 QUIET)
    if ("${GTest_VERSION}" STREQUAL "")
        message(STATUS "fpsdk: No GTest found, disable testing")
        set(BUILD_TESTING OFF)
    else()
        message(STATUS "fpsdk: Using GTest (${GTest_VERSION})")
        set(BUILD_TESTING ON)
    endif()

endif()


if(BUILD_TESTING)
    # Make unique test executable names across projects
    set(GTEST_PREFIX "${PROJECT_NAME}")
    enable_testing()
    include(GoogleTest)
    include(CTest)

    macro(add_gtest)
        # Arg parse
        set(options)
        set(one_value_args TARGET WORKING_DIRECTORY)
        set(multi_value_args SOURCES INCLUDE_DIRS LINK_DIRS LINK_LIBS)
        cmake_parse_arguments(ADD_GTEST "${options}" "${one_value_args}" "${multi_value_args}" ${ARGN})

        # Make targets
        add_executable(${GTEST_PREFIX}_${ADD_GTEST_TARGET}
            ${ADD_GTEST_SOURCES}
        )

        target_link_libraries(${GTEST_PREFIX}_${ADD_GTEST_TARGET}
            PRIVATE
                GTest::gtest_main
                ${ADD_GTEST_LINK_LIBS}
        )

        # Discover tests
        gtest_add_tests(
            TARGET ${GTEST_PREFIX}_${ADD_GTEST_TARGET}
            SOURCES ${ADD_GTEST_SOURCES}
            # WORKING_DIRECTORY ${ADD_GTEST_WORKING_DIRECTORY}
        )

        endmacro()

else()

    macro(add_gtest)
        # nothing...
    endmacro()

endif()
