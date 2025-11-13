/*
 * Copyright (C) 2009, Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 *  @file
 *  @brief ROS utils polyfill
 *
 *  @details Files impoorted:
 *              - ros/exception.h
 *              - ros/platform.h
 *              - ros/datatypes.h
 *
 *  ___    ___
 *  \  \  /  /
 *   \  \/  /   Fixposition AG
 *   /  /\  \   All right reserved.
 *  /__/  \__\
 *
 */

#ifndef __ROSNOROS__UTILS_H__
#define __ROSNOROS__UTILS_H__

/**********************************************************************************************************************/
// ros/exception.h
/**********************************************************************************************************************/
#include <stdexcept>

namespace ros {
/**
 * \brief Base class for all exceptions thrown by ROS
 */
class Exception : public std::runtime_error {
   public:
    Exception(const std::string& what) : std::runtime_error(what) {}
};
}  // namespace ros

/**********************************************************************************************************************/
// ros/platform.h
/**********************************************************************************************************************/
#include <cstdlib>  // getenv
#include <string>

namespace ros {

/**
 * Convenient cross platform function for returning a std::string of an
 * environment variable.
 */
inline bool get_environment_variable(std::string& str, const char* environment_variable) {
    char* env_var_cstr = NULL;
    env_var_cstr = getenv(environment_variable);
    if (env_var_cstr) {
        str = std::string(env_var_cstr);
        return true;
    } else {
        str = std::string("");
        return false;
    }
}

}  // namespace ros

/**********************************************************************************************************************/
// ros/datatypes.h
/**********************************************************************************************************************/
#include <boost/shared_ptr.hpp>
#include <map>
#include <set>

namespace ros {

typedef std::vector<std::pair<std::string, std::string> > VP_string;
typedef std::vector<std::string> V_string;
typedef std::set<std::string> S_string;
typedef std::map<std::string, std::string> M_string;
typedef std::pair<std::string, std::string> StringPair;

typedef boost::shared_ptr<M_string> M_stringPtr;

}  // namespace ros

#endif  // __ROSNOROS__UTILS_H__
