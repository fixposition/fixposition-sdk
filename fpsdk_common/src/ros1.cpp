/**
 * \verbatim
 * ___    ___
 * \  \  /  /
 *  \  \/  /   Copyright (c) Fixposition AG (www.fixposition.com) and contributors
 *  /  /\  \   License: see the LICENSE file
 * /__/  \__\
 *
 * Parts copyright (c) 2008, Willow Garage, Inc.
 * \endverbatim
 *
 * @file
 * @brief Fixposition SDK: ROS1 types and utils
 */

/* LIBC/STL */
#include <algorithm>
#include <cstring>
#include <fstream>
#include <map>
#include <string>
#include <vector>

/* EXTERNAL */
#if FPSDK_USE_BZ2
#  include <bzlib.h>
#endif

/* PACKAGE */
#include "fpsdk_common/logging.hpp"
#include "fpsdk_common/ros1.hpp"

namespace fpsdk {
namespace common {
namespace ros1 {
/* ****************************************************************************************************************** */

ros::Time ConvTime(const time::Time& time)
{
    const auto rt = time.GetRosTime();
    return ros::Time(rt.sec_, rt.nsec_);
}

time::Time ConvTime(const ros::Time& time)
{
    return time::Time::FromRosTime({ time.sec, time.nsec });
}

/* ****************************************************************************************************************** */

// Rosbag format version 2.0. See http://wiki.ros.org/Bags/Format/2.0 and, for the reference implementation,
// ros_comm/rosbag_storage in the ROS1 (Noetic) sources.
namespace {

using HeaderFields = std::map<std::string, std::string>;

// Record "op" field values
static constexpr uint8_t OP_MSG_DATA = 0x02;
static constexpr uint8_t OP_FILE_HEADER = 0x03;
static constexpr uint8_t OP_INDEX_DATA = 0x04;
static constexpr uint8_t OP_CHUNK = 0x05;
static constexpr uint8_t OP_CHUNK_INFO = 0x06;
static constexpr uint8_t OP_CONNECTION = 0x07;

static constexpr char VERSION_LINE[] = "#ROSBAG V2.0\n";
static constexpr uint32_t FILE_HEADER_LENGTH = 4 * 1024;  // Bytes reserved for the file header record
static constexpr uint32_t INDEX_VERSION = 1;
static constexpr uint32_t CHUNK_INFO_VERSION = 1;
static constexpr uint32_t CHUNK_THRESHOLD = 768 * 1024;  // Same as rosbag's default
static constexpr char COMPRESSION_NONE[] = "none";
static constexpr char COMPRESSION_BZ2[] = "bz2";
// Same parameters as rosbag (see ros_comm/rosbag_storage/src/bz2_stream.cpp)
#if FPSDK_USE_BZ2
static constexpr int BZ2_BLOCK_SIZE_100K = 9;
static constexpr int BZ2_WORK_FACTOR = 30;
static constexpr int BZ2_VERBOSITY = 0;
#endif

// Header field values are raw little-endian data, not strings
template <typename T>
static std::string HeaderVal(const T value)
{
    std::string str(sizeof(T), '\0');
    std::memcpy(&str[0], &value, sizeof(T));
    return str;
}

static std::string HeaderTime(const time::RosTime& time)
{
    std::string str(2 * sizeof(uint32_t), '\0');
    std::memcpy(&str[0], &time.sec_, sizeof(uint32_t));
    std::memcpy(&str[sizeof(uint32_t)], &time.nsec_, sizeof(uint32_t));
    return str;
}

static void AppendU32(std::vector<uint8_t>& buf, const uint32_t value)
{
    const uint8_t* data = (const uint8_t*)&value;
    buf.insert(buf.end(), data, data + sizeof(value));
}

static void AppendStr(std::vector<uint8_t>& buf, const std::string& str)
{
    buf.insert(buf.end(), str.cbegin(), str.cend());
}

// A record header: length of all fields, followed by the fields ("<len><name>=<value>")
static void AppendHeader(std::vector<uint8_t>& buf, const HeaderFields& fields)
{
    uint32_t total_len = 0;
    for (const auto& field : fields) {
        total_len += sizeof(uint32_t) + field.first.size() + 1 + field.second.size();
    }
    AppendU32(buf, total_len);
    for (const auto& field : fields) {
        AppendU32(buf, field.first.size() + 1 + field.second.size());
        AppendStr(buf, field.first);
        buf.push_back('=');
        AppendStr(buf, field.second);
    }
}

// One entry in a chunk's per-connection index
struct IndexEntry
{
    IndexEntry(const time::RosTime& time, const uint32_t offset)
        : sec_{ time.sec_ }, nsec_{ time.nsec_ }, offset_{ offset }
    {
    }
    uint32_t sec_;
    uint32_t nsec_;
    uint32_t offset_;  //!< Offset of the message record within the (uncompressed) chunk data
    bool operator<(const IndexEntry& rhs) const
    {
        return (sec_ < rhs.sec_) || ((sec_ == rhs.sec_) && (nsec_ < rhs.nsec_));
    }
};

struct ChunkInfo
{
    uint64_t pos_ = 0;  //!< Offset of the chunk record in the file
    time::RosTime start_time_;
    time::RosTime end_time_;
    std::map<uint32_t, uint32_t> conn_counts_;  //!< Number of messages per connection in this chunk
};

struct Connection
{
    uint32_t id_ = 0;
    HeaderFields header_;  //!< The connection header (type, md5sum, message_definition, ...)
};

static bool TimeLess(const time::RosTime& lhs, const time::RosTime& rhs)
{
    return (lhs.sec_ < rhs.sec_) || ((lhs.sec_ == rhs.sec_) && (lhs.nsec_ < rhs.nsec_));
}

}  // namespace

// ---------------------------------------------------------------------------------------------------------------------

struct BagWriter::Impl
{
    // Message definitions from .fpl (AddMsgDef()), by topic. These survive Open()/Close().
    std::map<std::string, HeaderFields> msg_defs_;

    std::string path_;
    std::fstream file_;
    bool error_ = false;
    uint64_t file_header_pos_ = 0;
    uint64_t index_data_pos_ = 0;
    std::map<std::string, Connection> conns_;  // Connections, by topic
    std::vector<ChunkInfo> chunks_;
    // The compression used for all chunks of this bag. Fixed at Open(), so that the placeholder chunk header and the
    // final one have the same size (see WriteChunkHeaderRecord()).
    std::string compression_ = COMPRESSION_NONE;

    // Current chunk
    bool chunk_open_ = false;
    ChunkInfo chunk_info_;
    std::vector<uint8_t> chunk_buf_;  //!< The chunk data (records) is assembled in memory
    std::vector<uint8_t> comp_buf_;   //!< The compressed chunk data (only used if compression_ != none)
    std::map<uint32_t, std::vector<IndexEntry>> chunk_index_;

    // -----------------------------------------------------------------------------------------------------------------

    bool Write(const void* data, const std::size_t size)
    {
        if (!error_) {
            file_.write((const char*)data, size);
            if (!file_) {
                WARNING("BagWriter: write fail %s", path_.c_str());
                error_ = true;
            }
        }
        return !error_;
    }

    bool Write(const std::vector<uint8_t>& buf)
    {
        return Write(buf.data(), buf.size());
    }

    bool Seek(const uint64_t pos)
    {
        if (!error_) {
            file_.seekp(pos, std::ios::beg);
            if (!file_) {
                WARNING("BagWriter: seek fail %s", path_.c_str());
                error_ = true;
            }
        }
        return !error_;
    }

    bool SeekEnd()
    {
        if (!error_) {
            file_.seekp(0, std::ios::end);
            if (!file_) {
                WARNING("BagWriter: seek fail %s", path_.c_str());
                error_ = true;
            }
        }
        return !error_;
    }

    uint64_t Tell()
    {
        return (error_ ? 0 : (uint64_t)file_.tellp());
    }

    // -----------------------------------------------------------------------------------------------------------------

    bool Open(const std::string& path, const std::string& compression)
    {
        Close();
        path_ = path;
        compression_ = compression;
        error_ = false;
        conns_.clear();
        chunks_.clear();
        chunk_open_ = false;
        chunk_buf_.clear();
        comp_buf_.clear();
        chunk_index_.clear();
        chunk_info_ = ChunkInfo();
        index_data_pos_ = 0;

        file_.open(path, std::ios::in | std::ios::out | std::ios::binary | std::ios::trunc);
        if (!file_.is_open()) {
            WARNING("BagWriter: open fail %s", path.c_str());
            error_ = true;
            return false;
        }

        // Version line, followed by the (for now empty) file header record
        Write(VERSION_LINE, sizeof(VERSION_LINE) - 1);
        file_header_pos_ = Tell();
        WriteFileHeaderRecord();

        if (error_) {
            file_.close();
            return false;
        }
        return true;
    }

    void Close()
    {
        if (file_.is_open()) {
            if (chunk_open_) {
                StopWritingChunk();
            }
            // The connection and chunk info records ("the index") go at the end of the file...
            SeekEnd();
            index_data_pos_ = Tell();
            WriteConnectionRecords();
            WriteChunkInfoRecords();
            // ...and now we know what to put into the file header
            Seek(file_header_pos_);
            WriteFileHeaderRecord();
            file_.close();
        }
        path_.clear();
    }

    // -----------------------------------------------------------------------------------------------------------------

    void WriteFileHeaderRecord()
    {
        const uint32_t conn_count = conns_.size();
        const uint32_t chunk_count = chunks_.size();
        const HeaderFields fields = { // clang-format off
            { "op",          HeaderVal(OP_FILE_HEADER) },
            { "index_pos",   HeaderVal(index_data_pos_) },
            { "conn_count",  HeaderVal(conn_count) },
            { "chunk_count", HeaderVal(chunk_count) } };  // clang-format on

        // Note: the size of this record must not change, so that it can be rewritten in place on Close(). It is padded
        // to a fixed size. All fields above have a fixed-size (binary) value, so the header length is constant, too.
        std::vector<uint8_t> buf;
        AppendHeader(buf, fields);
        const uint32_t header_len = buf.size() - sizeof(uint32_t);
        const uint32_t data_len = (header_len < FILE_HEADER_LENGTH ? FILE_HEADER_LENGTH - header_len : 0);
        AppendU32(buf, data_len);
        buf.resize(buf.size() + data_len, ' ');
        Write(buf);
    }

    void WriteChunkHeaderRecord(const uint32_t compressed_size, const uint32_t uncompressed_size)
    {
        const HeaderFields fields = { // clang-format off
            { "op",          HeaderVal(OP_CHUNK) },
            { "compression", compression_ },
            { "size",        HeaderVal(uncompressed_size) } };  // clang-format on
        std::vector<uint8_t> buf;
        AppendHeader(buf, fields);
        AppendU32(buf, compressed_size);
        Write(buf);
    }

    void StartWritingChunk(const time::RosTime& time)
    {
        chunk_info_ = ChunkInfo();
        chunk_info_.pos_ = Tell();
        chunk_info_.start_time_ = time;
        chunk_info_.end_time_ = time;
        chunk_buf_.clear();
        chunk_index_.clear();
        // Placeholder header, rewritten in StopWritingChunk() once the size is known
        WriteChunkHeaderRecord(0, 0);
        chunk_open_ = true;
    }

    void StopWritingChunk()
    {
        const uint32_t uncompressed_size = chunk_buf_.size();
        uint32_t compressed_size = uncompressed_size;
        if (compression_ == COMPRESSION_BZ2) {
            if (CompressChunkBz2(compressed_size)) {
                Write(comp_buf_.data(), compressed_size);
            }
        } else {
            Write(chunk_buf_);
        }
        const uint64_t end_pos = Tell();

        // Rewrite the chunk header, now that we know the sizes
        Seek(chunk_info_.pos_);
        WriteChunkHeaderRecord(compressed_size, uncompressed_size);
        Seek(end_pos);

        WriteIndexRecords();

        chunks_.push_back(chunk_info_);
        chunk_open_ = false;
        chunk_buf_.clear();
        chunk_index_.clear();
    }

    // Compress chunk_buf_ into comp_buf_. Returns false (and flags an error) on failure. Note that the compressed data
    // can be larger than the uncompressed data (for incompressible data), same as in rosbag.
    bool CompressChunkBz2(uint32_t& compressed_size)
    {
#if FPSDK_USE_BZ2
        // Worst case output size for bzip2 is 1% + 600 bytes more than the input
        unsigned int dest_len = chunk_buf_.size() + (chunk_buf_.size() / 100) + 600;
        comp_buf_.resize(dest_len);
        const int res = BZ2_bzBuffToBuffCompress((char*)comp_buf_.data(), &dest_len, (char*)chunk_buf_.data(),
            chunk_buf_.size(), BZ2_BLOCK_SIZE_100K, BZ2_VERBOSITY, BZ2_WORK_FACTOR);
        if (res != BZ_OK) {
            WARNING("BagWriter: bz2 compress fail (%d) %s", res, path_.c_str());
            error_ = true;
            return false;
        }
        compressed_size = dest_len;
        return true;
#else
        (void)compressed_size;
        WARNING("BagWriter: no bz2 support");
        error_ = true;
        return false;
#endif
    }

    void WriteIndexRecords()
    {
        for (auto& entry : chunk_index_) {
            const uint32_t conn_id = entry.first;
            std::vector<IndexEntry>& index = entry.second;
            // The index must be sorted by time (the messages are normally, but not necessarily, in time order)
            std::stable_sort(index.begin(), index.end());

            const uint32_t count = index.size();
            const HeaderFields fields = { // clang-format off
                { "op",    HeaderVal(OP_INDEX_DATA) },
                { "conn",  HeaderVal(conn_id) },
                { "ver",   HeaderVal(INDEX_VERSION) },
                { "count", HeaderVal(count) } };  // clang-format on
            std::vector<uint8_t> buf;
            AppendHeader(buf, fields);
            AppendU32(buf, count * 3 * sizeof(uint32_t));
            for (const auto& ix : index) {
                AppendU32(buf, ix.sec_);
                AppendU32(buf, ix.nsec_);
                AppendU32(buf, ix.offset_);
            }
            Write(buf);
        }
    }

    void WriteConnectionRecords()
    {
        // In connection id order (conns_ is by topic), same as rosbag does it
        std::vector<const std::map<std::string, Connection>::value_type*> conns;
        conns.reserve(conns_.size());
        for (const auto& entry : conns_) {
            conns.push_back(&entry);
        }
        std::sort(conns.begin(), conns.end(),
            [](const auto* lhs, const auto* rhs) { return lhs->second.id_ < rhs->second.id_; });
        for (const auto* entry : conns) {
            std::vector<uint8_t> buf;
            AppendConnectionRecord(buf, entry->first, entry->second);
            Write(buf);
        }
    }

    static void AppendConnectionRecord(std::vector<uint8_t>& buf, const std::string& topic, const Connection& conn)
    {
        const HeaderFields fields = { // clang-format off
            { "op",    HeaderVal(OP_CONNECTION) },
            { "topic", topic },
            { "conn",  HeaderVal(conn.id_) } };  // clang-format on
        AppendHeader(buf, fields);
        AppendHeader(buf, conn.header_);
    }

    void WriteChunkInfoRecords()
    {
        for (const auto& chunk : chunks_) {
            const uint32_t count = chunk.conn_counts_.size();
            const HeaderFields fields = { // clang-format off
                { "op",         HeaderVal(OP_CHUNK_INFO) },
                { "ver",        HeaderVal(CHUNK_INFO_VERSION) },
                { "chunk_pos",  HeaderVal(chunk.pos_) },
                { "start_time", HeaderTime(chunk.start_time_) },
                { "end_time",   HeaderTime(chunk.end_time_) },
                { "count",      HeaderVal(count) } };  // clang-format on
            std::vector<uint8_t> buf;
            AppendHeader(buf, fields);
            AppendU32(buf, count * 2 * sizeof(uint32_t));
            for (const auto& entry : chunk.conn_counts_) {
                AppendU32(buf, entry.first);
                AppendU32(buf, entry.second);
            }
            Write(buf);
        }
    }

    // -----------------------------------------------------------------------------------------------------------------

    bool WriteMessage(const std::string& topic, const time::RosTime& time, const std::string& msg_name,
        const std::string& msg_md5, const std::string& msg_def, const uint8_t* data, const std::size_t size)
    {
        if (!file_.is_open()) {
            WARNING("BagWriter: no bag open");
            return false;
        }
        if (error_) {
            return false;
        }

        // Find, or create, the connection for this topic
        auto conn_entry = conns_.find(topic);
        const bool new_conn = (conn_entry == conns_.end());
        if (new_conn) {
            Connection conn;
            conn.id_ = conns_.size();
            // Prefer the message definition from the .fpl (AddMsgDef()), fall back to the message type's traits
            const auto msg_def_entry = msg_defs_.find(topic);
            if (msg_def_entry != msg_defs_.end()) {
                conn.header_ = msg_def_entry->second;
            } else {
                conn.header_.emplace("type", msg_name);
                conn.header_.emplace("md5sum", msg_md5);
                conn.header_.emplace("message_definition", msg_def);
            }
            conn_entry = conns_.emplace(topic, std::move(conn)).first;
        }
        const Connection& conn = conn_entry->second;

        if (!chunk_open_) {
            StartWritingChunk(time);
        }

        // A new connection is announced in the chunk (and again at the end of the file, see WriteConnectionRecords())
        if (new_conn) {
            AppendConnectionRecord(chunk_buf_, topic, conn);
        }

        // Index the message record that we're about to add to the chunk
        chunk_index_[conn.id_].emplace_back(time, (uint32_t)chunk_buf_.size());
        chunk_info_.conn_counts_[conn.id_]++;
        if (TimeLess(chunk_info_.end_time_, time)) {
            chunk_info_.end_time_ = time;
        } else if (TimeLess(time, chunk_info_.start_time_)) {
            chunk_info_.start_time_ = time;
        }

        // The message record
        const HeaderFields fields = { // clang-format off
            { "op",   HeaderVal(OP_MSG_DATA) },
            { "conn", HeaderVal(conn.id_) },
            { "time", HeaderTime(time) } };  // clang-format on
        AppendHeader(chunk_buf_, fields);
        AppendU32(chunk_buf_, size);
        chunk_buf_.insert(chunk_buf_.end(), data, data + size);

        if (chunk_buf_.size() > CHUNK_THRESHOLD) {
            StopWritingChunk();
        }

        return !error_;
    }
};

/* ****************************************************************************************************************** */

BagWriter::BagWriter() /* clang-format off */ :
    impl_   { std::make_unique<Impl>() }
{
}

BagWriter::~BagWriter()
{
    Close();
}

// ---------------------------------------------------------------------------------------------------------------------

bool BagWriter::Open(const std::string& path, const int compress)
{
    // Note: unlike rosbag we have no LZ4. So compress = 1 gives bz2, too (and not lz4).
    std::string compression = COMPRESSION_NONE;
    if (compress > 0) {
#if FPSDK_USE_BZ2
        compression = COMPRESSION_BZ2;
#else
        WARNING("BagWriter: cannot create compressed %s, bz2 support not compiled in", path.c_str());
        return false;
#endif
    }
    if (!impl_->Open(path, compression)) {
        return false;
    }
    DEBUG("BagWriter: %s (%s)", path.c_str(), compression.c_str());
    return true;
}

// ---------------------------------------------------------------------------------------------------------------------

void BagWriter::Close()
{
    impl_->Close();
}

// ---------------------------------------------------------------------------------------------------------------------

void BagWriter::AddMsgDef(const fpl::RosMsgDef& rosmsgdef)
{
    if (rosmsgdef.valid_ && (impl_->msg_defs_.find(rosmsgdef.topic_name_) == impl_->msg_defs_.end())) {
        DEBUG("BagWriter: %s", rosmsgdef.info_.c_str());
        impl_->msg_defs_.emplace(
            rosmsgdef.topic_name_, HeaderFields{ // clang-format off
            { "message_definition", rosmsgdef.msg_def_ },
            { "topic",              rosmsgdef.topic_name_ },
            { "md5sum",             rosmsgdef.msg_md5_ },
            { "type",               rosmsgdef.msg_name_ } });  // clang-format on
    }
}

// ---------------------------------------------------------------------------------------------------------------------

bool BagWriter::WriteMessage(const fpl::RosMsgBin& rosmsgbin)
{
    if (!rosmsgbin.valid_) {
        return false;
    }
    // For ROS1 we can directly write the serialised data. We should already know the message meta data (see
    // AddMsgDef() above).
    if (impl_->msg_defs_.find(rosmsgbin.topic_name_) == impl_->msg_defs_.end()) {
        WARNING("BagWriter: missing message definition for %s", rosmsgbin.topic_name_.c_str());
        return false;
    }
    return impl_->WriteMessage(
        rosmsgbin.topic_name_, rosmsgbin.rec_time_, "", "", "", rosmsgbin.msg_data_.data(), rosmsgbin.msg_data_.size());
}

// ---------------------------------------------------------------------------------------------------------------------

bool BagWriter::WriteSerialised(const std::string& topic, const time::RosTime& time, const std::string& msg_name,
    const std::string& msg_md5, const std::string& msg_def, const uint8_t* data, const std::size_t size)
{
    return impl_->WriteMessage(topic, time, msg_name, msg_md5, msg_def, data, size);
}

/* ****************************************************************************************************************** */

#if FPSDK_USE_ROS1

static char g_logger_name[100];

static void sLoggingFn(const logging::LoggingParams& /*params*/, const logging::LoggingLevel level, const char* str)
{
    switch (level) {  // clang-format off
        case logging::LoggingLevel::TRACE:   ROS_LOG(ros::console::levels::Debug, g_logger_name, "%s", str); break;
        case logging::LoggingLevel::DEBUG:   ROS_LOG(ros::console::levels::Debug, g_logger_name, "%s", str); break;
        case logging::LoggingLevel::INFO:    ROS_LOG(ros::console::levels::Info,  g_logger_name, "%s", str); break;
        case logging::LoggingLevel::NOTICE:  ROS_LOG(ros::console::levels::Info,  g_logger_name, "%s", str); break;
        case logging::LoggingLevel::WARNING: ROS_LOG(ros::console::levels::Warn,  g_logger_name, "%s", str); break;
        case logging::LoggingLevel::ERROR:   ROS_LOG(ros::console::levels::Error, g_logger_name, "%s", str); break;
        case logging::LoggingLevel::FATAL:   ROS_LOG(ros::console::levels::Fatal, g_logger_name, "%s", str); break;
    }  // clang-format on
}

void RedirectLoggingToRosConsole(const char* logger_name)
{
    logging::LoggingParams params = logging::LoggingGetParams();
    params.fn_ = sLoggingFn;
    params.level_ = logging::LoggingLevel::TRACE;  // We leave it up to ROS to decide what to print
    // Set logger name. Note that ROSCONSOLE_DEFAULT_NAME here is the "ros.fpsdk_common" package name. However, when
    // called from the app (node) then the default argument to logger_name is that package's ROSCONSOLE_DEFAULT_NAME,
    // e.g. "ros.my_package"
    std::snprintf(
        g_logger_name, sizeof(g_logger_name), "%s", logger_name != NULL ? logger_name : ROSCONSOLE_DEFAULT_NAME);
    LoggingSetParams(params);
}

#endif  // FPSDK_USE_ROS1

/* ****************************************************************************************************************** */
}  // namespace ros1
}  // namespace common
}  // namespace fpsdk
