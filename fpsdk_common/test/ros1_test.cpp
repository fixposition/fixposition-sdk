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
 * @brief Fixposition SDK: tests for fpsdk::common::ros1
 */

/* LIBC/STL */
#include <cstdio>
#include <cstring>
#include <fstream>
#include <map>
#include <string>
#include <unistd.h>
#include <vector>

/* EXTERNAL */
#include <gtest/gtest.h>
#if FPSDK_USE_BZ2
#  include <bzlib.h>
#endif

/* PACKAGE */
#include <fpsdk_common/logging.hpp>
#include <fpsdk_common/ros1.hpp>

namespace {
/* ****************************************************************************************************************** */
using namespace fpsdk::common::ros1;
using namespace fpsdk::common::time;

// ---------------------------------------------------------------------------------------------------------------------

TEST(Ros1Test, ConvTime)
{
    const ros::Time ros_time(1723650283, 125000000);
    DEBUG("ros::Time %" PRIu32 " %" PRIu32, ros_time.sec, ros_time.nsec);

    const fpsdk::common::time::Time sdk_time = ConvTime(ros_time);
    DEBUG("SDK Time  %" PRIu32 " %" PRIu32, sdk_time.sec_, sdk_time.nsec_);
    EXPECT_EQ(sdk_time.sec_, ros_time.sec + 27);  // 1723650310
    EXPECT_EQ(sdk_time.nsec_, ros_time.nsec);

    const ros::Time ros_time_again = ConvTime(sdk_time);
    DEBUG("ros::Time %" PRIu32 " %" PRIu32, ros_time_again.sec, ros_time_again.nsec);

    EXPECT_EQ(ros_time_again.sec, ros_time.sec);
    EXPECT_EQ(ros_time_again.nsec, ros_time.nsec);
}

// ---------------------------------------------------------------------------------------------------------------------

// The ros::Time implementation from rosnoros/ros/time.cpp (or from the real ROS1)

TEST(Ros1Test, RosTime)
{
    ros::Time t1(1, 999999999);
    EXPECT_EQ(t1.sec, 1u);
    EXPECT_EQ(t1.nsec, 999999999u);
    EXPECT_EQ(t1.toNSec(), 1999999999u);

    // Normalisation of the nsec overflow
    ros::Time t2(1, 2500000000);
    EXPECT_EQ(t2.sec, 3u);
    EXPECT_EQ(t2.nsec, 500000000u);

    ros::Time t3;
    EXPECT_TRUE(t3.isZero());
    t3.fromNSec(1234000005678u);
    EXPECT_EQ(t3.sec, 1234u);
    EXPECT_EQ(t3.nsec, 5678u);
    t3.fromSec(1234.5);
    EXPECT_EQ(t3.sec, 1234u);
    EXPECT_EQ(t3.nsec, 500000000u);

    // Comparison and arithmetic
    EXPECT_TRUE(t1 < t2);
    EXPECT_TRUE(t2 > t1);
    EXPECT_TRUE(t1 != t2);
    EXPECT_TRUE(t1 == ros::Time(1, 999999999));
    EXPECT_EQ((t2 - t1).toNSec(), 1500000001);
    EXPECT_TRUE((t1 + ros::Duration(0, 1)) == ros::Time(2, 0));

    // The constants
    EXPECT_TRUE(ros::TIME_MIN == ros::Time(0, 1));
    EXPECT_TRUE(ros::Time::MIN == ros::TIME_MIN);
    EXPECT_TRUE(ros::Time::ZERO.isZero());
    EXPECT_TRUE(ros::Duration::ZERO.isZero());
    EXPECT_EQ(ros::Duration::SECOND.toNSec(), 1000000000);

    // Now() should be a sensible wall clock time (> 2020-01-01, < 2100-01-01)
    const ros::Time now = ros::Time::now();
    EXPECT_GT(now.sec, 1577836800u);
    EXPECT_LT(now.sec, 4102444800u);
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(Ros1Test, SerialiseDeserialise)
{
    sensor_msgs::Imu msg;
    msg.header.seq = 42;
    msg.header.stamp.fromNSec(1234000005678u);
    msg.header.frame_id = "test_frame";
    msg.linear_acceleration.x = 1.25;
    msg.angular_velocity.z = -2.5;
    msg.orientation.w = 1.0;

    const uint32_t size = ros::serialization::serializationLength(msg);
    EXPECT_GT(size, 0u);
    std::vector<uint8_t> buf(size);
    ros::serialization::OStream stream(buf.data(), size);
    ros::serialization::serialize(stream, msg);

    sensor_msgs::Imu msg2;
    DeserializeMessage(buf, msg2);
    EXPECT_EQ(msg2.header.seq, 42u);
    EXPECT_EQ(msg2.header.stamp.sec, 1234u);
    EXPECT_EQ(msg2.header.stamp.nsec, 5678u);
    EXPECT_EQ(msg2.header.frame_id, "test_frame");
    EXPECT_EQ(msg2.linear_acceleration.x, 1.25);
    EXPECT_EQ(msg2.angular_velocity.z, -2.5);
    EXPECT_EQ(msg2.orientation.w, 1.0);
}

/* ****************************************************************************************************************** */
// A minimal, independent rosbag v2.0 reader, to check what BagWriter produces. This deliberately does not share any
// code with the writer.

struct BagRecord
{
    std::map<std::string, std::string> header_;
    std::string data_;
    uint8_t Op() const
    {
        const auto entry = header_.find("op");
        return (entry == header_.end() || entry->second.empty() ? 0xff : (uint8_t)entry->second[0]);
    }
    uint32_t U32(const std::string& field) const
    {
        uint32_t value = 0;
        const auto entry = header_.find(field);
        if ((entry != header_.end()) && (entry->second.size() == sizeof(value))) {
            std::memcpy(&value, entry->second.data(), sizeof(value));
        }
        return value;
    }
    uint64_t U64(const std::string& field) const
    {
        uint64_t value = 0;
        const auto entry = header_.find(field);
        if ((entry != header_.end()) && (entry->second.size() == sizeof(value))) {
            std::memcpy(&value, entry->second.data(), sizeof(value));
        }
        return value;
    }
    std::string Str(const std::string& field) const
    {
        const auto entry = header_.find(field);
        return (entry == header_.end() ? std::string() : entry->second);
    }
};

class BagReader
{
   public:
    bool Load(const std::string& path)
    {
        std::ifstream file(path, std::ios::binary);
        if (!file.is_open()) {
            return false;
        }
        buf_.assign(std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>());
        return true;
    }

    // Parse a "<len><name>=<value>..." header blob
    static bool ParseHeader(const std::string& blob, std::map<std::string, std::string>& fields)
    {
        std::size_t pos = 0;
        while (pos < blob.size()) {
            if ((blob.size() - pos) < 4) {
                return false;
            }
            uint32_t len = 0;
            std::memcpy(&len, &blob[pos], 4);
            pos += 4;
            if ((blob.size() - pos) < len) {
                return false;
            }
            const std::string field = blob.substr(pos, len);
            pos += len;
            const std::size_t eq = field.find('=');
            if (eq == std::string::npos) {
                return false;
            }
            fields[field.substr(0, eq)] = field.substr(eq + 1);
        }
        return pos == blob.size();
    }

    // Read one "<header_len><header><data_len><data>" record
    static bool ReadRecord(const std::string& buf, std::size_t& pos, BagRecord& rec)
    {
        if ((buf.size() - pos) < 4) {
            return false;
        }
        uint32_t header_len = 0;
        std::memcpy(&header_len, &buf[pos], 4);
        pos += 4;
        if ((buf.size() - pos) < header_len) {
            return false;
        }
        if (!ParseHeader(buf.substr(pos, header_len), rec.header_)) {
            return false;
        }
        pos += header_len;
        if ((buf.size() - pos) < 4) {
            return false;
        }
        uint32_t data_len = 0;
        std::memcpy(&data_len, &buf[pos], 4);
        pos += 4;
        if ((buf.size() - pos) < data_len) {
            return false;
        }
        rec.data_ = buf.substr(pos, data_len);
        pos += data_len;
        return true;
    }

    std::string buf_;
};

// The parsed contents of a bag
struct BagContents
{
    uint32_t conn_count_ = 0;
    uint32_t chunk_count_ = 0;
    uint32_t n_chunks_ = 0;
    uint32_t n_index_ = 0;
    std::string compression_;                                           //!< The compression of the last chunk seen
    uint64_t compressed_ = 0;                                           //!< Total compressed size of all chunks
    uint64_t uncompressed_ = 0;                                         //!< Total uncompressed size of all chunks
    std::map<uint32_t, std::string> topics_;                            //!< Topic by connection id
    std::map<uint32_t, std::map<std::string, std::string>> conn_hdrs_;  //!< Connection header by connection id
    std::map<uint32_t, std::map<std::string, std::string>> tail_hdrs_;  //!< Ditto, from the index section
    std::map<uint32_t, std::vector<std::string>> msgs_;                 //!< Message data by connection id
    std::map<uint32_t, std::vector<RosTime>> times_;                    //!< Message times by connection id
    std::map<uint32_t, uint32_t> info_counts_;                          //!< Counts from the CHUNK_INFO records
};

// Read and check a bag. Returns "" on success, or a description of the problem.
std::string ReadBag(const std::string& path, BagContents& bag)
{
#define BAGCHECK(cond, msg) \
    if (!(cond)) {          \
        return (msg);       \
    }
    BagReader reader;
    BAGCHECK(reader.Load(path), "cannot open");
    const std::string& buf = reader.buf_;
    BAGCHECK(buf.substr(0, 13) == "#ROSBAG V2.0\n", "bad version line");

    // File header record
    std::size_t pos = 13;
    BagRecord fh;
    BAGCHECK(BagReader::ReadRecord(buf, pos, fh), "no file header record");
    BAGCHECK(fh.Op() == 0x03, "no FILE_HEADER op");
    BAGCHECK(pos == (13 + 4096 + 8), "file header record not padded to 4096");
    const uint64_t index_pos = fh.U64("index_pos");
    bag.conn_count_ = fh.U32("conn_count");
    bag.chunk_count_ = fh.U32("chunk_count");
    BAGCHECK((index_pos >= pos) && (index_pos <= buf.size()), "bad index_pos");  // == pos for an empty bag

    // Chunk section
    std::vector<uint32_t> chunk_offsets;
    while (pos < index_pos) {
        BagRecord rec;
        BAGCHECK(BagReader::ReadRecord(buf, pos, rec), "bad record in chunk section");
        // Chunk
        if (rec.Op() == 0x05) {
            bag.n_chunks_++;
            bag.compression_ = rec.Str("compression");
            const uint32_t uncompressed_size = rec.U32("size");
            bag.compressed_ += rec.data_.size();
            bag.uncompressed_ += uncompressed_size;
            if (bag.compression_ == "none") {
                BAGCHECK(uncompressed_size == rec.data_.size(), "chunk size mismatch");
            } else if (bag.compression_ == "bz2") {
#if FPSDK_USE_BZ2
                // Decompress exactly the way rosbag's Bag::decompressBz2Chunk() does it
                std::string plain(uncompressed_size, '\0');
                unsigned int dest_len = uncompressed_size;
                const int res = BZ2_bzBuffToBuffDecompress(uncompressed_size > 0 ? &plain[0] : nullptr, &dest_len,
                    rec.data_.empty() ? nullptr : &rec.data_[0], rec.data_.size(), 0, 0);
                BAGCHECK(res == BZ_OK, "bz2 decompress fail");
                BAGCHECK(dest_len == uncompressed_size, "bz2 chunk size mismatch");
                rec.data_ = plain;
#else
                return "bz2 chunk but no bz2 support";
#endif
            } else {
                return "unexpected compression";
            }
            chunk_offsets.clear();
            std::size_t cpos = 0;
            while (cpos < rec.data_.size()) {
                chunk_offsets.push_back(cpos);
                BagRecord crec;
                BAGCHECK(BagReader::ReadRecord(rec.data_, cpos, crec), "bad record in chunk");
                // Connection
                if (crec.Op() == 0x07) {
                    const uint32_t id = crec.U32("conn");
                    BAGCHECK(bag.topics_.count(id) == 0, "duplicate connection id");
                    bag.topics_[id] = crec.Str("topic");
                    BAGCHECK(BagReader::ParseHeader(crec.data_, bag.conn_hdrs_[id]), "bad connection header");
                }
                // Message
                else if (crec.Op() == 0x02) {
                    const uint32_t id = crec.U32("conn");
                    const std::string time = crec.Str("time");
                    BAGCHECK(time.size() == 8, "bad message time field");
                    uint32_t sec = 0;
                    uint32_t nsec = 0;
                    std::memcpy(&sec, &time[0], 4);
                    std::memcpy(&nsec, &time[4], 4);
                    bag.msgs_[id].push_back(crec.data_);
                    bag.times_[id].push_back(RosTime(sec, nsec));
                } else {
                    return "unexpected op in chunk";
                }
            }
            BAGCHECK(cpos == rec.data_.size(), "chunk records overrun");
        }
        // Index
        else if (rec.Op() == 0x04) {
            bag.n_index_++;
            BAGCHECK(rec.U32("ver") == 1, "bad index version");
            const uint32_t count = rec.U32("count");
            BAGCHECK(rec.data_.size() == (count * 12), "index data size mismatch");
            uint64_t prev = 0;
            for (uint32_t ix = 0; ix < count; ix++) {
                uint32_t sec = 0;
                uint32_t nsec = 0;
                uint32_t offset = 0;
                std::memcpy(&sec, &rec.data_[(ix * 12) + 0], 4);
                std::memcpy(&nsec, &rec.data_[(ix * 12) + 4], 4);
                std::memcpy(&offset, &rec.data_[(ix * 12) + 8], 4);
                const uint64_t time = ((uint64_t)sec * 1000000000) + nsec;
                BAGCHECK(time >= prev, "index not sorted by time");
                prev = time;
                bool found = false;
                for (const auto cand : chunk_offsets) {
                    if (cand == offset) {
                        found = true;
                        break;
                    }
                }
                BAGCHECK(found, "index offset is not a record boundary");
            }
        } else {
            return "unexpected op in chunk section";
        }
    }
    BAGCHECK(pos == index_pos, "chunk section does not end at index_pos");
    BAGCHECK(bag.n_chunks_ == bag.chunk_count_, "chunk count mismatch");

    // Index section
    uint32_t n_conn = 0;
    uint32_t n_info = 0;
    uint32_t prev_conn_id = 0;
    while (pos < buf.size()) {
        BagRecord rec;
        BAGCHECK(BagReader::ReadRecord(buf, pos, rec), "bad record in index section");
        // Connection
        if (rec.Op() == 0x07) {
            const uint32_t id = rec.U32("conn");
            BAGCHECK((n_conn == 0) || (id > prev_conn_id), "connection records not in id order");
            prev_conn_id = id;
            n_conn++;
            BAGCHECK(BagReader::ParseHeader(rec.data_, bag.tail_hdrs_[id]), "bad connection header");
        }
        // Chunk info
        else if (rec.Op() == 0x06) {
            n_info++;
            BAGCHECK(rec.U32("ver") == 1, "bad chunk info version");
            BAGCHECK(rec.Str("start_time").size() == 8, "bad start_time");
            BAGCHECK(rec.Str("end_time").size() == 8, "bad end_time");
            const uint32_t count = rec.U32("count");
            BAGCHECK(rec.data_.size() == (count * 8), "chunk info data size mismatch");
            for (uint32_t ix = 0; ix < count; ix++) {
                uint32_t id = 0;
                uint32_t num = 0;
                std::memcpy(&id, &rec.data_[(ix * 8) + 0], 4);
                std::memcpy(&num, &rec.data_[(ix * 8) + 4], 4);
                bag.info_counts_[id] += num;
            }
        } else {
            return "unexpected op in index section";
        }
    }
    BAGCHECK(pos == buf.size(), "trailing garbage");
    BAGCHECK(n_conn == bag.conn_count_, "connection count mismatch");
    BAGCHECK(n_info == bag.chunk_count_, "chunk info count mismatch");
    BAGCHECK(bag.tail_hdrs_ == bag.conn_hdrs_, "index connection headers differ from the in-chunk ones");
    for (const auto& entry : bag.msgs_) {
        BAGCHECK(bag.info_counts_[entry.first] == entry.second.size(), "CHUNK_INFO counts != actual messages");
    }
#undef BAGCHECK
    return "";
}

// ---------------------------------------------------------------------------------------------------------------------

class BagWriterTest : public ::testing::Test
{
   protected:
    void SetUp() override
    {
        char tmpl[] = "/tmp/fpsdk_bagwriter_test_XXXXXX";
        const int fd = mkstemp(tmpl);
        ASSERT_GE(fd, 0);
        close(fd);
        path_ = tmpl;
    }
    void TearDown() override
    {
        std::remove(path_.c_str());
    }
    std::string path_;
};

// ---------------------------------------------------------------------------------------------------------------------

TEST_F(BagWriterTest, WriteAndReadBack)
{
    const RosTime t0(1750000000, 500000);

    {
        BagWriter bag;
        ASSERT_TRUE(bag.Open(path_));

        for (int ix = 0; ix < 10; ix++) {
            sensor_msgs::Imu imu;
            imu.header.seq = ix;
            imu.header.frame_id = "imu_frame";
            imu.header.stamp = ros::Time(t0.sec_ + ix, t0.nsec_);
            imu.linear_acceleration.x = ix;
            EXPECT_TRUE(bag.WriteMessage(imu, "/imu/data", RosTime(t0.sec_ + ix, t0.nsec_)));

            sensor_msgs::Temperature temp;
            temp.header.frame_id = "imu_frame";
            temp.temperature = 20.0 + ix;
            // The ros::Time flavour of WriteMessage()
            EXPECT_TRUE(bag.WriteMessage(temp, "/imu/temp", ros::Time(t0.sec_ + ix, t0.nsec_)));
        }
        bag.Close();
    }

    BagContents bag;
    ASSERT_EQ(ReadBag(path_, bag), "");
    EXPECT_EQ(bag.conn_count_, 2u);
    EXPECT_EQ(bag.chunk_count_, 1u);  // Way below the chunk threshold
    EXPECT_EQ(bag.n_index_, 2u);      // One index record per connection per chunk
    EXPECT_EQ(bag.topics_.at(0), "/imu/data");
    EXPECT_EQ(bag.topics_.at(1), "/imu/temp");
    EXPECT_EQ(bag.conn_hdrs_.at(0).at("type"), "sensor_msgs/Imu");
    EXPECT_EQ(bag.conn_hdrs_.at(0).at("md5sum"), ros::message_traits::md5sum<sensor_msgs::Imu>());
    EXPECT_EQ(bag.conn_hdrs_.at(0).at("message_definition"), ros::message_traits::definition<sensor_msgs::Imu>());
    EXPECT_EQ(bag.conn_hdrs_.at(1).at("type"), "sensor_msgs/Temperature");
    ASSERT_EQ(bag.msgs_.at(0).size(), 10u);
    ASSERT_EQ(bag.msgs_.at(1).size(), 10u);

    // The message payloads must deserialise to what we put in
    for (int ix = 0; ix < 10; ix++) {
        const auto& data = bag.msgs_.at(0)[ix];
        std::vector<uint8_t> raw(data.cbegin(), data.cend());
        sensor_msgs::Imu imu;
        DeserializeMessage(raw, imu);
        EXPECT_EQ(imu.header.seq, (uint32_t)ix);
        EXPECT_EQ(imu.header.frame_id, "imu_frame");
        EXPECT_EQ(imu.header.stamp.sec, t0.sec_ + ix);
        EXPECT_EQ(imu.header.stamp.nsec, t0.nsec_);
        EXPECT_EQ(imu.linear_acceleration.x, (double)ix);
        // ...and the bag record time must be right, too
        EXPECT_EQ(bag.times_.at(0)[ix].sec_, t0.sec_ + ix);
        EXPECT_EQ(bag.times_.at(0)[ix].nsec_, t0.nsec_);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

TEST_F(BagWriterTest, MultipleChunks)
{
    // Write enough data to trigger more than one chunk (the threshold is 768 KiB)
    const int n_msgs = 20;
    const std::size_t msg_size = 100 * 1024;
    {
        BagWriter bag;
        ASSERT_TRUE(bag.Open(path_));
        for (int ix = 0; ix < n_msgs; ix++) {
            std_msgs::ByteMultiArray msg;
            msg.data.assign(msg_size, (int8_t)ix);
            EXPECT_TRUE(bag.WriteMessage(msg, "/big/raw", RosTime(1750000000 + ix, 0)));
        }
        bag.Close();
    }

    BagContents bag;
    ASSERT_EQ(ReadBag(path_, bag), "");
    EXPECT_EQ(bag.conn_count_, 1u);
    EXPECT_GE(bag.chunk_count_, 2u);
    EXPECT_EQ(bag.chunk_count_, bag.n_index_);  // One connection -> one index record per chunk
    ASSERT_EQ(bag.msgs_.at(0).size(), (std::size_t)n_msgs);
    for (int ix = 0; ix < n_msgs; ix++) {
        const auto& data = bag.msgs_.at(0)[ix];
        std::vector<uint8_t> raw(data.cbegin(), data.cend());
        std_msgs::ByteMultiArray msg;
        DeserializeMessage(raw, msg);
        ASSERT_EQ(msg.data.size(), msg_size);
        EXPECT_EQ(msg.data[0], (int8_t)ix);
        EXPECT_EQ(msg.data[msg_size - 1], (int8_t)ix);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

TEST_F(BagWriterTest, MessagesFromFpl)
{
    // Serialise a message the way it is stored in a .fpl file
    sensor_msgs::Imu imu;
    imu.header.frame_id = "fpl_frame";
    imu.angular_velocity.y = 3.25;
    const uint32_t size = ros::serialization::serializationLength(imu);
    std::vector<uint8_t> data(size);
    ros::serialization::OStream stream(data.data(), size);
    ros::serialization::serialize(stream, imu);

    fpsdk::common::fpl::RosMsgDef def;
    def.valid_ = true;
    def.info_ = "test";
    def.topic_name_ = "/fpl/imu";
    def.msg_name_ = ros::message_traits::datatype<sensor_msgs::Imu>();
    def.msg_md5_ = ros::message_traits::md5sum<sensor_msgs::Imu>();
    def.msg_def_ = ros::message_traits::definition<sensor_msgs::Imu>();

    fpsdk::common::fpl::RosMsgBin bin;
    bin.valid_ = true;
    bin.topic_name_ = def.topic_name_;
    bin.rec_time_ = RosTime(1750000123, 456);
    bin.msg_data_ = data;

    {
        BagWriter bag;
        ASSERT_TRUE(bag.Open(path_));

        // Without a message definition this must fail -- noisy
        EXPECT_FALSE(bag.WriteMessage(bin));

        bag.AddMsgDef(def);
        EXPECT_TRUE(bag.WriteMessage(bin));
        EXPECT_TRUE(bag.WriteMessage(bin));

        // An invalid message is silently ignored
        fpsdk::common::fpl::RosMsgBin invalid;
        EXPECT_FALSE(bag.WriteMessage(invalid));

        bag.Close();
    }

    BagContents bag;
    ASSERT_EQ(ReadBag(path_, bag), "");
    EXPECT_EQ(bag.conn_count_, 1u);
    EXPECT_EQ(bag.topics_.at(0), "/fpl/imu");
    // The .fpl connection header is used as-is (and it also has the "topic" field)
    EXPECT_EQ(bag.conn_hdrs_.at(0).at("type"), def.msg_name_);
    EXPECT_EQ(bag.conn_hdrs_.at(0).at("md5sum"), def.msg_md5_);
    EXPECT_EQ(bag.conn_hdrs_.at(0).at("message_definition"), def.msg_def_);
    EXPECT_EQ(bag.conn_hdrs_.at(0).at("topic"), def.topic_name_);
    ASSERT_EQ(bag.msgs_.at(0).size(), 2u);
    // The serialised data is written verbatim
    EXPECT_TRUE(bag.msgs_.at(0)[0] == std::string(data.cbegin(), data.cend()));
    EXPECT_EQ(bag.times_.at(0)[1].sec_, 1750000123u);
    EXPECT_EQ(bag.times_.at(0)[1].nsec_, 456u);
}

// ---------------------------------------------------------------------------------------------------------------------

TEST_F(BagWriterTest, EmptyBag)
{
    {
        BagWriter bag;
        ASSERT_TRUE(bag.Open(path_));
        bag.Close();
    }
    BagContents bag;
    ASSERT_EQ(ReadBag(path_, bag), "");
    EXPECT_EQ(bag.conn_count_, 0u);
    EXPECT_EQ(bag.chunk_count_, 0u);
}

// ---------------------------------------------------------------------------------------------------------------------

TEST_F(BagWriterTest, ErrorCases)
{
    BagWriter bag;

    // Writing without a bag open fails, but doesn't blow up
    sensor_msgs::Imu imu;
    EXPECT_FALSE(bag.WriteMessage(imu, "/imu/data"));
    // Closing without a bag open is fine
    bag.Close();

    // Bad path -- noisy
    EXPECT_FALSE(bag.Open("/no/such/directory/nope.bag"));
    EXPECT_FALSE(bag.WriteMessage(imu, "/imu/data"));

#if FPSDK_USE_BZ2
    // Compression is not available
    EXPECT_TRUE(bag.Open(path_, 2));
    EXPECT_TRUE(bag.WriteMessage(imu, "/imu/data", RosTime(1750000000, 0)));
#endif

    // Re-opening implicitly closes the previous bag
    ASSERT_TRUE(bag.Open(path_));
    EXPECT_TRUE(bag.WriteMessage(imu, "/imu/data", RosTime(1750000000, 0)));
    bag.Close();

    BagContents contents;
    ASSERT_EQ(ReadBag(path_, contents), "");
    EXPECT_EQ(contents.conn_count_, 1u);
    EXPECT_EQ(contents.msgs_.at(0).size(), 1u);
}

// ---------------------------------------------------------------------------------------------------------------------

TEST_F(BagWriterTest, UnorderedTimes)
{
    // The index records must be sorted by time even if the messages aren't written in time order
    {
        BagWriter bag;
        ASSERT_TRUE(bag.Open(path_));
        sensor_msgs::Temperature msg;
        for (const uint32_t sec : { 1750000005u, 1750000001u, 1750000003u, 1750000002u }) {
            EXPECT_TRUE(bag.WriteMessage(msg, "/imu/temp", RosTime(sec, 0)));
        }
        bag.Close();
    }
    // ReadBag() checks that the index is sorted
    BagContents bag;
    ASSERT_EQ(ReadBag(path_, bag), "");
    ASSERT_EQ(bag.msgs_.at(0).size(), 4u);
    // ...but the messages themselves stay in the order they were written
    EXPECT_EQ(bag.times_.at(0)[0].sec_, 1750000005u);
    EXPECT_EQ(bag.times_.at(0)[3].sec_, 1750000002u);
}

// ---------------------------------------------------------------------------------------------------------------------

#if FPSDK_USE_BZ2
// Write the same data compressed and uncompressed, and compare
TEST_F(BagWriterTest, Bz2Compression)
{
    // Nicely compressible data
    const int n_msgs = 30;
    const std::size_t msg_size = 64 * 1024;
    const auto write_bag = [&](const std::string& path, const int compress) {
        BagWriter bag;
        EXPECT_TRUE(bag.Open(path, compress));
        for (int ix = 0; ix < n_msgs; ix++) {
            std_msgs::ByteMultiArray msg;
            msg.layout.dim.resize(1);
            msg.layout.dim[0].label = "test";
            msg.data.assign(msg_size, (int8_t)(ix % 7));
            EXPECT_TRUE(bag.WriteMessage(msg, "/big/raw", RosTime(1750000000 + ix, 0)));
        }
        bag.Close();
    };

    write_bag(path_, 0);
    BagContents plain;
    ASSERT_EQ(ReadBag(path_, plain), "");
    EXPECT_EQ(plain.compression_, "none");
    EXPECT_EQ(plain.compressed_, plain.uncompressed_);

    const std::string comp_path = path_ + ".bz2.bag";
    write_bag(comp_path, 1);  // Note: 1 means LZ4 in rosbag, but we don't have that, so it's bz2, too
    BagContents comp;
    const std::string res = ReadBag(comp_path, comp);
    std::remove(comp_path.c_str());
    ASSERT_EQ(res, "");

#  if FPSDK_USE_BZ2
    EXPECT_EQ(comp.compression_, "bz2");
    EXPECT_LT(comp.compressed_, comp.uncompressed_ / 10);  // This data compresses very well
#  else
    EXPECT_EQ(comp.compression_, "none");  // No bz2 available, it only warns
    EXPECT_EQ(comp.compressed_, comp.uncompressed_);
#  endif

    // Same content either way
    EXPECT_EQ(comp.uncompressed_, plain.uncompressed_);
    EXPECT_EQ(comp.chunk_count_, plain.chunk_count_);
    EXPECT_EQ(comp.conn_count_, plain.conn_count_);
    EXPECT_EQ(comp.conn_hdrs_, plain.conn_hdrs_);
    ASSERT_EQ(comp.msgs_.at(0).size(), (std::size_t)n_msgs);
    EXPECT_TRUE(comp.msgs_ == plain.msgs_);
    for (int ix = 0; ix < n_msgs; ix++) {
        const auto& data = comp.msgs_.at(0)[ix];
        std::vector<uint8_t> raw(data.cbegin(), data.cend());
        std_msgs::ByteMultiArray msg;
        DeserializeMessage(raw, msg);
        ASSERT_EQ(msg.data.size(), msg_size);
        EXPECT_EQ(msg.data[0], (int8_t)(ix % 7));
        ASSERT_EQ(msg.layout.dim.size(), 1u);
        EXPECT_EQ(msg.layout.dim[0].label, "test");
    }
}
#endif

// ---------------------------------------------------------------------------------------------------------------------

#if FPSDK_USE_ROS1
// This should be the last test as it messes with the console and we may want to use DEBUG() etc. in the tests
TEST(UtilsTest, RedirectLoggingToRosConsole)
{
    // Silence the ROS console
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Warn);

    // We're not actually testing much more than checking that this doesn't crash.
    // Run the test with -v -v -v and set ROS console level to Debug (above) to make it a bit more interesting in the
    // output.
    // clang-format off
    // $ clear; make INSTALL_PREFIX=fpsdk BUILD_TYPE=Debug build && ROSCONSOLE_FORMAT='${severity} ${time:%Y-%m-%d %H:%M:%S.%f} ${logger} - ${message}' build/Debug/fpsdk_common/fpsdk_common_ros1_test -v -v -v
    // clang-format on

    ROS_DEBUG("Hello, this is a ros debug before redirect...");
    ROS_INFO("Hello, this is a ros info before redirect...");
    // ROS_WARN("Hello, this is a ros warn before redirect...");
    DEBUG("This is fpsdk_common debug before redirect...");
    INFO("This is fpsdk_common info before redirect...");
    // WARNING("This is fpsdk_common warning before redirect...");

    RedirectLoggingToRosConsole();

    ROS_DEBUG("Hello, this is a ros debug after redirect...");
    ROS_INFO("Hello, this is a ros info after redirect...");
    // ROS_WARN("Hello, this is a ros warn after redirect...");
    DEBUG("This is fpsdk_common debug after redirect...");
    INFO("This is fpsdk_common info after redirect...");
    // WARNING("This is fpsdk_common warning after redirect...");
}
// This should be the last test as it messes with the console and we may want to use DEBUG() etc. in the tests
#endif

/* ****************************************************************************************************************** */
}  // namespace

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    auto level = fpsdk::common::logging::LoggingLevel::WARNING;
    for (int ix = 0; ix < argc; ix++) {
        if ((argv[ix][0] == '-') && argv[ix][1] == 'v') {
            level++;
        }
    }
#if FPSDK_USE_ROS1
    ros::Time::init();
#endif
    fpsdk::common::logging::LoggingSetParams(level);
    return RUN_ALL_TESTS();
}
