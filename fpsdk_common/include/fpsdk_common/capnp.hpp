/**
 * \verbatim
 * ___    ___
 * \  \  /  /
 *  \  \/  /   Copyright (c) Fixposition AG (www.fixposition.com) and contributors
 *  /  /\  \   License: see the LICENSE file
 * /__/  \__\
 * \endverbatim
 *
 * @file
 * @brief Fixposition SDK: Cap'n Proto utilities
 *
 * @page FPSDK_COMMON_CAPNP Cap'n Proto utilities
 *
 * **API**: fpsdk_common/capnp.hpp and fpsdk::common::capnp
 *
 */
#ifndef __FPSDK_COMMON_CAPNP_HPP__
#define __FPSDK_COMMON_CAPNP_HPP__

/* LIBC/STL */
#include <cinttypes>
#include <string>
#include <vector>

/* EXTERNAL */
#include <capnp/compat/json.h>
#include <capnp/message.h>
#include <capnp/serialize-packed.h>
#include <capnp/serialize.h>

/* PACKAGE */

namespace fpsdk {
namespace common {
/**
 * @brief Cap'n Proto utilities
 */
namespace capnp {
/* ****************************************************************************************************************** */

/**
 * @brief Serialise capnp message (unpacked)
 *
 * @param[in]  capnp_msg  The message (builder) to serialise
 *
 * @returns the serialised data
 */
std::vector<uint8_t> MessageToUnpacked(::capnp::MessageBuilder& capnp_msg);

/**
 * @brief Serialise capnp message (packed)
 *
 * @param[in]  capnp_msg  The message (builder) to serialise
 *
 * @returns the serialised data
 */
std::vector<uint8_t> MessageToPacked(::capnp::MessageBuilder& capnp_msg);

/**
 * @brief Deserialise message (unpacked)
 *
 * @param[in]  data  The serialised data (no alignement required)
 * @param[in]  size  The size of the serialised data
 * @tparam     T     Capnp message type to instantiate
 *
 * @returns some funky pointer thingy (something like a unique_ptr) with the message (and a copy of the data), throws on
 *          error (bad data)
 */
template <typename T>
kj::Own<typename T::Reader> UnpackedToMessage(const uint8_t* data, const std::size_t size)
{
    kj::ArrayPtr<kj::byte> buffer((kj::byte*)data, size);
    kj::ArrayInputStream in(buffer);
    ::capnp::InputStreamMessageReader capnp_msg(in);
    // The clone copies the necessary input data, into a buffer attached to the reader object.
    // "Unfortunately" this is now a funky kj::Own<T> thingy..
    return ::capnp::clone<typename T::Reader>(capnp_msg.getRoot<T>());
}

/**
 * @brief Deserialise message (packed)
 *
 * @param[in]  data  The serialised data (no alignement required)
 * @param[in]  size  The size of the serialised data
 * @tparam     T     Capnp message type to instantiate
 *
 * @returns some funky pointer thingy (something like a unique_ptr) with the message (and a copy of the data), throws on
 *          error (bad data)
 */
template <typename T>
kj::Own<typename T::Reader> PackedToMessage(const uint8_t* data, const std::size_t size)
{
    kj::ArrayPtr<kj::byte> buffer((kj::byte*)data, size);
    kj::ArrayInputStream in(buffer);
    ::capnp::PackedMessageReader capnp_msg(in);
    // The clone copies the necessary input data, into a buffer attached to the reader object.
    // "Unfortunately" this is now a funky kj::Own<T> thingy..
    return ::capnp::clone<typename T::Reader>(capnp_msg.getRoot<T>());
}

/**
 * @brief Serialise to JSON
 *
 * @param[in]  message  The message (reader) to serialise
 * @param[in]  pretty   Add whitespace and (sometimes) linefeeds to generated JSON, otherwise use compact JSON
 * @tparam     T        Capnp message type
 */
template <typename T>
std::string MessageToJson(const T& message, const bool pretty = false)
{
    ::capnp::JsonCodec json;
    json.setPrettyPrint(pretty);
    kj::String json_str = json.encode(message);
    return json_str.cStr();
}

/* ****************************************************************************************************************** */
}  // namespace capnp
}  // namespace common
}  // namespace fpsdk
#endif  // __FPSDK_COMMON_CAPNP_HPP__
