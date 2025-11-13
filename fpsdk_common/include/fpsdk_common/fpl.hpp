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
 * @brief Fixposition SDK: .fpl utilities
 *
 * @page FPSDK_COMMON_FPL .fpl utilities
 *
 * **API**: fpsdk_common/fpl.hpp and fpsdk::common::fpl
 *
 */
#ifndef __FPSDK_COMMON_FPL_HPP__
#define __FPSDK_COMMON_FPL_HPP__

/* LIBC/STL */
#include <cstdint>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

/* EXTERNAL */

/* PACKAGE */
#include "time.hpp"

namespace fpsdk {
namespace common {
/**
 * @brief .fpl utilities
 */
namespace fpl {
/* ****************************************************************************************************************** */

/**
 * @brief FplMessage types
 */
enum class FplType : uint16_t
{  // clang-format off
    UNSPECIFIED  = 0x0000,  //!< Invalid
    ROSMSGDEF    = 0x0101,  //!< Message definition for a topic (a.k.a. "connection header" in rosbag speak)
    ROSMSGBIN    = 0x0102,  //!< Serialised ROS message with timestamp
    LOGMETA      = 0x0105,  //!< Logfile meta data
    STREAMMSG    = 0x0106,  //!< Stream message raw data with timestamp
    LOGSTATUS    = 0x0107,  //!< Logging status
    FILEDUMP     = 0x0108,  //!< Dump of an entire (small) file
    BLOB         = 0xbaad,  //!< Arbitrary data, not FplMessage framing
    INT_D        = 0xffaa,  //!< Fixposition internal use only
    INT_F        = 0xffbb,  //!< Fixposition internal use only
    INT_X        = 0xffcc,  //!< Fixposition internal use only
};  // clang-format on

/**
 * @brief Stringify type
 *
 * @param[in]  type  The type
 *
 * @returns a unique string identifying the type
 */
const char* FplTypeStr(const FplType type);

// ---------------------------------------------------------------------------------------------------------------------

/**
 * @brief One message in the log
 */
class FplMessage
{
   public:
    FplMessage();

    /**
     * @brief Parse message from buffer
     *
     * @param[in]  data  Pointer to start of message data
     * @param[in]  size  Size of message data
     *
     * @returns 0 if the data is not a valid message,
     *          -1 if the size is too small to decide, or
     *          the message size (>0) if a valid message was found and loaded
     */
    int Parse(const uint8_t* data, const uint32_t size);

    /**
     * @brief Get message
     *
     * @returns a reference to the message data
     */
    const std::vector<uint8_t>& Raw() const;
    /**
     * @brief Get message data
     *
     * @returns a pointer to the message data (size = Size())
     */
    const uint8_t* RawData() const;

    /**
     * @brief Get message size
     *
     * @returns the size of the message (data = Data())
     */
    uint32_t RawSize() const;

    /**
     * @brief Get payload (log) type
     *
     * @returns type of the payload
     */
    FplType PayloadType() const;

    /**
     * @brief Get payload data
     *
     * @returns a pointer to the payload data
     */
    const uint8_t* PayloadData() const;

    /**
     * @brief Get payload size
     *
     * @returns size of the payload data
     */
    uint32_t PayloadSize() const;

    // Debugging stuff, only with FplParser. For debugging.
    uint64_t file_pos_;   //!< Offset in logfile of the log message
    uint64_t file_size_;  //!< Size in the logfile of the log message
    uint64_t file_seq_;   //!< Message sequence counter in logfile

    static constexpr std::uint32_t MAX_SIZE = 2000000;  //!< Maximum message size
    static constexpr std::size_t MAX_NAME_LEN = 100;    //!< Maximum length of (file, topic, ...) names

    /**
     * @brief Make message a BLOB
     *
     * @param[in]  data  The data
     * @param[in]  size  The size of the data
     */
    void SetBlob(const uint8_t* data, const uint32_t size);

   private:
    static constexpr uint16_t MAGIC = 0x55aa;   //!< Start of message marker
    static constexpr uint32_t HEADER_SIZE = 8;  //!< Message header size
    FplType payload_type_;                      //!< Payload type
    uint32_t payload_size_;                     //!< Payload size
    std::vector<uint8_t> message_;              //!< Entire message
};

// ---------------------------------------------------------------------------------------------------------------------

/**
 * @brief Logfile parser
 */
class FplParser
{
   public:
    FplParser();

    /**
     * @brief Add data to parser
     *
     * @param[in]  data  Pointer to data, can be NULL, in which case \c size is ignored
     * @param[in]  size  Size of data, can be <= 0, in which case \c data is ignored
     *
     * The suggested chunk size to add to the parser is FplMessage::MAX_SIZE
     *
     * @returns true if data was added to the parser, false if there was not enough space left (parser overflow)
     */
    bool Add(const uint8_t* data, const std::size_t size);

    /**
     * @brief Reset parser
     *
     * Resets the parser state and discards all collected data.
     */
    void Reset();

    /**
     * @brief Process data in parser, return message
     *
     * @param[out]  log_msg  The detected message
     *
     * @returns true if a message was detected, false otherwise (meaning: not enough data in parser). Note that also in
     *          the false case the \c log_msg may be modified.
     */
    bool Process(FplMessage& log_msg);

   private:
    std::vector<uint8_t> buf_;           //!< Working buffer
    std::size_t offs_;                   //!< Current offset into buf_
    std::size_t size_;                   //!< Current size of useful data in buf_ (counted from offs_)
    std::size_t file_pos_;               //!< Offset into "file", i.e. total number of bytes processed so far
    std::size_t file_seq_;               //!< Count of messages from "file"
    void EmitBlob(FplMessage& log_msg);  //!< Helper for Process()
};

// ---------------------------------------------------------------------------------------------------------------------

/**
 * @brief Read log messages from logfile
 */
class FplFileReader
{
   public:
    FplFileReader();

    /**
     * @brief Open logfile
     *
     * @param[in]  path  The logfile
     *
     * @returns true on success, false otherwise
     */
    bool Open(const std::string& path);

    /**
     * @brief Get next log message
     *
     * @param[out]  log_msg  The log message
     *
     * @returns true if a next log message was found, false at end of file (or other problems)
     */
    bool Next(FplMessage& log_msg);

    /**
     * @brief Get progress report
     *
     * @param[out]  progress   Progress in [%] (approximate with gzipped files)
     * @param[out]  rate       Rate in [MiB/s]
     *
     * @returns true if is a good time to print the progress report, false otherwise
     */
    bool GetProgress(double& progress, double& rate);

   private:
    std::string path_;                  //!< Logfile path
    std::unique_ptr<std::istream> fh_;  //!< File handle
    FplParser parser_;                  //!< Parser
    std::vector<uint8_t> buf_;          //!< Read buffer
    std::size_t pos_;                   //!< Current position in logfile
    std::size_t count_;                 //!< Number of messages
    std::size_t size_;                  //!< Size of logfile
    bool is_gz_;                        //!< File is gzipped
    double last_progress_;              //!< GetProgress() state
    uint64_t last_pos_;                 //!< GetProgress() state
    double last_secs_;                  //!< GetProgress() state
};

// ---------------------------------------------------------------------------------------------------------------------

/**
 * @brief Helper for extracting meta data
 */
struct LogMeta
{
    LogMeta() = default;  //!< Default ctor

    /**
     * @brief Constructor
     *
     * @param[in]  log_msg  .fpl log message
     */
    LogMeta(const FplMessage& log_msg);
    bool valid_ = false;                 //!< Data valid, successfully extracted from message
    std::string info_;                   //!< Stringification of (some of the) data, for debugging
    std::string hw_uid_;                 //!< Hardware UID
    std::string product_model_;          //!< Product model
    std::string sw_version_;             //!< Software version
    uint32_t log_start_time_posix_ = 0;  //!< Start time of logging
    std::string log_start_time_iso_;     //!< Start time of logging
    std::string log_profile_;            //!< Logging configuration profile name
    std::string log_target_;             //!< Loging target name
    std::string log_filename_;           //!< Logfile name
    std::string yaml_;                   //!< Raw meta data YAML
};

// ---------------------------------------------------------------------------------------------------------------------

/**
 * @brief Helper for extracting recording status data
 */
struct LogStatus
{
    LogStatus() = default;  //!< Default ctor

    /**
     * @brief Constructor
     *
     * @param[in]  log_msg  .fpl log message
     */
    LogStatus(const FplMessage& log_msg);
    bool valid_;  //!< Data valid, successfully extracted from message

    // Version 1 and later
    std::string info_;          //!< Stringification of (some of the) data, for debugging
    std::string state_;         //!< Logging state: "stopped", "logging", "stopping"
    uint32_t queue_size_ = 0;   //!< Queue size
    uint32_t queue_peak_ = 0;   //!< Queue peak size
    uint32_t queue_skip_ = 0;   //!< Number of skipped messages (queue full)
    uint32_t queue_bsize_ = 0;  //!< Output queue size [bytes]
    uint32_t queue_bpeak_ = 0;  //!< Output queue peak size [bytes]

    uint32_t log_count_ = 0;     //!< Number of logged messages
    uint32_t log_errors_ = 0;    //!< Number of messages failed to log (failed to write/send)
    uint64_t log_size_ = 0;      //!< Total size of logged messages [bytes]
    uint32_t log_duration_ = 0;  //!< Duration of logging [s]

    // Version 2 and later
    uint32_t log_time_posix_ = 0;  //!< Approximate time
    std::string log_time_iso_;     //!< Approximate time
    int8_t pos_source_ = 0;        //!< Approximate sensor position source (see POS_SOURCE_... below)
    int8_t pos_fix_type_ = 0;      //!< Approximate sensor position fix type (see fpsdk::common::gnss::FixType)
    double pos_lat_ = 0.0;         //!< Approximate sensor position latitude [deg]
    double pos_lon_ = 0.0;         //!< Approximate sensor position longitude [deg]
    double pos_height_ = 0.0;      //!< Approximate sensor position height [m]

    static constexpr int8_t POS_SOURCE_UNKNOWN = 0;  //!< Position source unknown resp. not available
    static constexpr int8_t POS_SOURCE_GNSS = 1;     //!< Position source is GNSS
    static constexpr int8_t POS_SOURCE_FUSION = 2;   //!< Position source is Fusion

    std::string yaml_;  //!< Raw status data YAML
};

// ---------------------------------------------------------------------------------------------------------------------

/**
 * @brief Helper for extracting ROS message definition (the relevant fields from the "connection header")
 */
struct RosMsgDef
{
    RosMsgDef() = default;  //!< Default ctor

    /**
     * @brief Constructor
     *
     * @param[in]  log_msg  .fpl log message
     */
    RosMsgDef(const FplMessage& log_msg);
    bool valid_ = false;      //!< Data valid, successfully extracted from message
    std::string info_;        //!< Stringification of (some of the) data, for debugging
    std::string topic_name_;  //!< The topic name
    std::string msg_name_;    //!< The message name (a.k.a. data type)
    std::string msg_md5_;     //!< The message definition MD5 sum
    std::string msg_def_;     //!< The message definition (the .msg file and also the defs for all the types)
};

// ---------------------------------------------------------------------------------------------------------------------

/**
 * @brief Helper for extracting a serialised ROS message
 */
struct RosMsgBin
{
    RosMsgBin() = default;  //!< Default ctor

    /**
     * @brief Constructor
     *
     * @param[in]  log_msg  .fpl log message
     */
    RosMsgBin(const FplMessage& log_msg);
    bool valid_ = false;             //!< Data valid, successfully extracted from message
    std::string info_;               //!< Stringification of (some of the) data, for debugging
    std::string topic_name_;         //!< The topic name
    time::RosTime rec_time_;         //!< Recording timestamp
    std::vector<uint8_t> msg_data_;  //!< Serialised ROS message data
};

// ---------------------------------------------------------------------------------------------------------------------

/**
 * @brief Helper for extracting data of a stream message (NMEA, RTCM3, etc.)
 */
struct StreamMsg
{
    StreamMsg() = default;  //!< Default ctor

    /**
     * @brief Constructor
     *
     * @param[in]  log_msg  .fpl log message
     */
    StreamMsg(const FplMessage& log_msg);
    bool valid_ = false;             //!< Data valid, successfully extracted from message
    std::string info_;               //!< Stringification of (some of the) data, for debugging
    time::RosTime rec_time_;         //!< Recording timestamp
    std::string stream_name_;        //!< Stream name
    std::vector<uint8_t> msg_data_;  //!< Message data
};

// ---------------------------------------------------------------------------------------------------------------------

/**
 * @brief Helper for extracting data of a stream message (NMEA, RTCM3, etc.)
 */
struct FileDump
{
    /**
     * @brief Constructor
     *
     * @param[in]  log_msg  .fpl log message
     */
    FileDump(const FplMessage& log_msg);
    bool valid_;                 //!< Data valid, successfully extracted from message
    std::string info_;           //!< Stringification of (some of the) data, for debugging
    std::string filename_;       //!< Filename
    time::Time mtime_;           //!< File modification time
    std::vector<uint8_t> data_;  //!< File contents
};

/* ****************************************************************************************************************** */
}  // namespace fpl
}  // namespace common
}  // namespace fpsdk
#endif  // __FPSDK_COMMON_FPL_HPP__
