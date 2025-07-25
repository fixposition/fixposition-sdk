# GENERAL ==============================================================================================================

cmake_minimum_required(VERSION 3.16)
include(../fpsdk_common/cmake/setup.cmake)

project(fpsdk_apps
    LANGUAGES CXX
    VERSION ${FPSDK_VERSION_NUMBER}
    DESCRIPTION "Fixposition SDK: Apps"
)


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

if(FP_USE_ROS1)
    if(NOT TARGET fpsdk_ros1)
        find_package(fpsdk_ros1 REQUIRED)
        message(STATUS "fpsdk_apps: fpsdk_ros1_INCLUDE_DIRS=${fpsdk_ros1_INCLUDE_DIRS}")
        message(STATUS "fpsdk_apps: fpsdk_ros1_LIBRARIES=${fpsdk_ros1_LIBRARIES}")
    endif()
    add_compile_definitions(FP_USE_ROS1)
elseif(FP_USE_ROS2)
    add_compile_definitions(FP_USE_ROS2)
else()
    message(STATUS "fpsdk_apps: No ROS available")
endif()

include_directories(include ${fpsdk_common_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIR} ${fpsdk_ros1_INCLUDE_DIRS})


# SHARED LIBRARY =======================================================================================================

# https://gitlab.kitware.com/cmake/community/-/wikis/doc/cmake/RPATH-handling#recommendations
list(APPEND CMAKE_INSTALL_RPATH $ORIGIN)
list(APPEND CMAKE_INSTALL_RPATH $ORIGIN/../lib)
if(FP_USE_ROS1)
    list(APPEND CMAKE_INSTALL_RPATH ${fpsdk_ros1_ROS_LIBRARIES_DIRS})
    # Note that user still has to load the ROS environment (setup.bash) as the RUNPATH (mistakenly called RPATH by
    # cmake) a) does not work for indirect loading (app -> libfpsdk_ros1 -> someroslib) and b) ROS does funny things
    # run-time (such as loading more libs)
endif()


# EXECUTABLES ==========================================================================================================

file(GLOB COMMON_CPP_FILES common/*.cpp)

file(GLOB FPLTOOL_CPP_FILES fpltool/*.cpp)
add_executable(fpltool ${FPLTOOL_CPP_FILES})
target_link_libraries(fpltool
    fpsdk_common
    $<$<BOOL:${FP_USE_ROS1}>:fpsdk_ros1>
)

file(GLOB PARSERTOOL_CPP_FILES parsertool/*.cpp)
add_executable(parsertool ${PARSERTOOL_CPP_FILES} ${COMMON_CPP_FILES})
target_link_libraries(parsertool
    fpsdk_common
)

file(GLOB TIMECONV_CPP_FILES timeconv/*.cpp)
add_executable(timeconv ${TIMECONV_CPP_FILES} ${COMMON_CPP_FILES})
target_link_libraries(timeconv
    fpsdk_common
)

file(GLOB YAML2SHELL_CPP_FILES yaml2shell/*.cpp)
add_executable(yaml2shell ${YAML2SHELL_CPP_FILES} ${COMMON_CPP_FILES})
target_link_libraries(yaml2shell
    fpsdk_common
)


# INSTALL ==============================================================================================================

include(GNUInstallDirs) # Provides nice relative paths wrt CMAKE_INSTALL_PREFIX
set(PROJECT_RUNTIME_DIR ${CMAKE_INSTALL_FULL_BINDIR})
set(PROJECT_LIBRARY_DIR ${CMAKE_INSTALL_FULL_LIBDIR})
set(PROJECT_INCLUDE_DIR ${CMAKE_INSTALL_FULL_INCLUDEDIR}/${PROJECT_NAME})
set(PROJECT_DATA_DIR    ${CMAKE_INSTALL_FULL_DATAROOTDIR}/${PROJECT_NAME})

install(TARGETS fpltool parsertool timeconv yaml2shell
    EXPORT ${PROJECT_NAME}-targets
    LIBRARY DESTINATION ${PROJECT_LIBRARY_DIR}
    RUNTIME DESTINATION ${PROJECT_RUNTIME_DIR}
)


# ======================================================================================================================
