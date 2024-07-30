# GENERAL ==============================================================================================================

cmake_minimum_required(VERSION 3.16)
include(../fpcommon/cmake/setup.cmake)

project(fpapps
    LANGUAGES CXX
    VERSION ${FP_VERSION_NUMBER}
    DESCRIPTION "Fixposition SDK: Apps"
)


# COMPILER SETUP =======================================================================================================

set(CMAKE_CXX_STANDARD 14)
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

# Handle this being used as add_subdirectory() of ../CMakeLists.txt vs. building this standalone
if(NOT TARGET fpcommon)
    find_package(fpcommon REQUIRED)
endif()

find_package(CapnProto 1.0.2 REQUIRED) # TODO: why needed?

if(FP_USE_ROS1)
    if(NOT TARGET fpros1)
        find_package(fpros1 REQUIRED)
        message(STATUS "fpapps: fpros1_INCLUDE_DIRS=${fpros1_INCLUDE_DIRS}")
        message(STATUS "fpapps: fpros1_LIBRARIES=${fpros1_LIBRARIES}")
    endif()
    add_compile_definitions(FP_USE_ROS1)
elseif(FP_USE_ROS2)
    add_compile_definitions(FP_USE_ROS2)
else()
    message(STATUS "fpapps: No ROS available")
endif()

include_directories(include ${fpcommon_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIR} ${fpros1_INCLUDE_DIRS})


# SHARED LIBRARY =======================================================================================================

# https://gitlab.kitware.com/cmake/community/-/wikis/doc/cmake/RPATH-handling#recommendations
list(APPEND CMAKE_INSTALL_RPATH $ORIGIN)
list(APPEND CMAKE_INSTALL_RPATH $ORIGIN/../lib)
if(FP_USE_ROS1)
    list(APPEND CMAKE_INSTALL_RPATH ${fpros1_ROS_LIBRARIES_DIRS})
    # Note that user still has to load the ROS environment (setup.bash) as the RUNPATH (mistakenly called RPATH by
    # cmake) a) does not work for indirect loading (app -> libfpros1 -> someroslib) and b) ROS does funny things
    # run-time (such as loading more libs)
endif()


# EXECUTABLES ==========================================================================================================

file(GLOB FPL_TOOL_CPP_FILES fpltool/*.cpp)
add_executable(fpltool ${FPL_TOOL_CPP_FILES})
target_link_libraries(fpltool
    fpcommon
    $<$<BOOL:${FP_USE_ROS1}>:fpros1>
)


# INSTALL ==============================================================================================================

include(GNUInstallDirs) # Provides nice relative paths wrt CMAKE_INSTALL_PREFIX
set(PROJECT_RUNTIME_DIR ${CMAKE_INSTALL_FULL_BINDIR})
set(PROJECT_LIBRARY_DIR ${CMAKE_INSTALL_FULL_LIBDIR})
set(PROJECT_INCLUDE_DIR ${CMAKE_INSTALL_FULL_INCLUDEDIR}/${PROJECT_NAME})
set(PROJECT_DATA_DIR    ${CMAKE_INSTALL_FULL_DATAROOTDIR}/${PROJECT_NAME})

install(TARGETS fpltool
    EXPORT ${PROJECT_NAME}-targets
    LIBRARY DESTINATION ${PROJECT_LIBRARY_DIR}
    RUNTIME DESTINATION ${PROJECT_RUNTIME_DIR}
)


# ======================================================================================================================
