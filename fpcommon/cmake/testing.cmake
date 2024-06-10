find_package(GTest)

# Disable tests if GTest is not installed (or not installed properly)
if ("${GTest_VERSION}" STREQUAL "")
    message(STATUS "fpsdk: No GTest found, disable testing")
    set(BUILD_TESTING OFF)
# Enable tests, unless use gave -DBUILD_TESTING=... on the command-line
else()
    if("${BUILD_TESTING}" STREQUAL "")
        set(BUILD_TESTING ON)
    endif()
endif()

if(BUILD_TESTING)

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
        add_executable(${PROJECT_NAME}_${ADD_GTEST_TARGET}
            ${ADD_GTEST_SOURCES}
        )

        target_include_directories(${GTEST_PREFIX}_${ADD_GTEST_TARGET}
            PRIVATE ${ADD_GTEST_INCLUDE_DIRS}
        )
        target_link_directories(${GTEST_PREFIX}_${ADD_GTEST_TARGET}
            PRIVATE ${ADD_GTEST_LINK_DIRS}
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
            WORKING_DIRECTORY ${ADD_GTEST_WORKING_DIRECTORY}
        )

        endmacro()

else()

macro(add_gtest)
        # nothing...
    endmacro()

endif()
