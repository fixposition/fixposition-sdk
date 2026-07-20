# Explicitly requested by user to build tests (-DFPSDK_BUILD_TESTING=ON)
if(FPSDK_BUILD_TESTING STREQUAL "ON")
    find_package(GTest 1.11.0 REQUIRED)
    # GTest doesn't seem to have a normal cmake config file, so we have to check and abort ourselves.. :-/
    if ("${GTest_VERSION}" STREQUAL "")
        message(FATAL_ERROR "Unsupported GTest version")
    endif()

    # Explicitly requested by user to not build tests (-DFPSDK_BUILD_TESTING=OFF)
elseif(FPSDK_BUILD_TESTING STREQUAL "OFF")
    message(STATUS "fpsdk: Not using gtest")

    # Automatic, build tests if a suitable version of gtest is available (-DFPSDK_BUILD_TESTING=)
else()
    find_package(GTest 1.11.0 QUIET)
    if ("${GTest_VERSION}" STREQUAL "")
        set(FPSDK_BUILD_TESTING OFF)
        message(STATUS "fpsdk: No gtest found")
    else()
        set(FPSDK_BUILD_TESTING ON)
    endif()
endif()

# Setup testing stuff
if(FPSDK_BUILD_TESTING)
    message(STATUS "fpsdk: Using gtest (${GTest_VERSION})")

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
    set(BUILD_TESTING OFF)

    macro(add_gtest)
        # nothing...
    endmacro()

endif()
