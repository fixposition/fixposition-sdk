// Wrapper to suppress warning from Eigen headers
#ifndef __FPSDK_COMMON_EXT_EIGEN_CORE_HPP__
#define __FPSDK_COMMON_EXT_EIGEN_CORE_HPP__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wall"
#pragma GCC diagnostic ignored "-Wextra"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wshadow"

#if !defined(__clang__)
#  pragma GCC diagnostic ignored "-Wmaybe-uninitialized"  // NOLINT
#  if defined(__GNUC__) && (__GNUC__ >= 9)
#    pragma GCC diagnostic ignored "-Wdeprecated-copy"
#  endif
#  pragma GCC diagnostic ignored "-Wclass-memaccess"  // NOLINT
#endif

#include <Eigen/Core>
#pragma GCC diagnostic pop
// For older Eigen (e.g. 3.3.7, which we have in fusion-dev-env and on the sensor), we unfortunately have to disable
// this warning globally (templates instantiation can be anywhere...). :-/ GCC 9 (in Yocto) doesn't like this.
// See https://gitlab.com/libeigen/eigen/-/issues/1788, https://gitlab.com/libeigen/eigen/-/merge_requests/29
#if !EIGEN_VERSION_AT_LEAST(3, 4, 0) && defined(__GNUC__) && (__GNUC__ >= 9)
#  pragma GCC diagnostic ignored "-Wdeprecated-copy"
#endif
#endif  // __FPSDK_COMMON_EXT_EIGEN_CORE_HPP__
