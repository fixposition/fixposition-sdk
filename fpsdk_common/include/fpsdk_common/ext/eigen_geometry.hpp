// Wrapper to suppress warning from Eigen headers
#ifndef __FPSDK_COMMON_EXT_EIGEN_GEOMETRY_HPP__
#define __FPSDK_COMMON_EXT_EIGEN_GEOMETRY_HPP__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wall"
#pragma GCC diagnostic ignored "-Wextra"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wshadow"

#if !defined(__clang__)
#  pragma GCC diagnostic ignored "-Wclass-memaccess"  // NOLINT
#  if defined(__GNUC__) && (__GNUC__ >= 9)
#    pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#    pragma GCC diagnostic ignored "-Wdeprecated-copy"
#  endif
#endif

#include <Eigen/Geometry>
#pragma GCC diagnostic pop
// See exmplanation in eigen_core.hpp
#if !EIGEN_VERSION_AT_LEAST(3, 4, 0) && defined(__GNUC__) && (__GNUC__ >= 9)
#  pragma GCC diagnostic ignored "-Wdeprecated-copy"
#endif
#endif  // __FPSDK_COMMON_EXT_EIGEN_GEOMETRY_HPP__
