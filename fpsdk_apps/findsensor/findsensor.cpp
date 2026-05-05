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
 * @brief Fixposition SDK: findsensor app
 */

/* LIBC/STL */
#include <array>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <exception>
#include <map>
#include <memory>
#include <string>
#include <vector>

/* EXTERNAL */
#include <boost/asio/buffer.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/ip/multicast.hpp>
#include <boost/asio/ip/udp.hpp>
#include <boost/asio/ip/v6_only.hpp>
#include <boost/asio/post.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/system/error_code.hpp>
#include <ifaddrs.h>
#include <nlohmann/json.hpp>
#include <sys/types.h>

/* Fixposition SDK */
#include <fpsdk_common/app.hpp>
#include <fpsdk_common/logging.hpp>
#include <fpsdk_common/string.hpp>
#include <fpsdk_common/time.hpp>
#include <fpsdk_common/types.hpp>
#include <fpsdk_common/utils.hpp>

/* PACKAGE */

namespace fpsdk {
namespace apps {
namespace findsensor {
/* ****************************************************************************************************************** */

using namespace fpsdk::common::app;
using namespace fpsdk::common::logging;
using namespace fpsdk::common::string;
using namespace fpsdk::common::utils;
using namespace fpsdk::common::types;
using namespace nlohmann;
using namespace boost::asio;

// ---------------------------------------------------------------------------------------------------------------------

static std::string PropsToStr(const std::map<std::string, std::string>& props)
{
    std::string str;
    for (auto& [k, v] : props) {
        str += ", " + k + "=" + v;
    }
    return str.size() > 2 ? str.substr(2) : str;
}

// Program options
class FindSensorOptions : public ProgramOptions
{
   public:
    FindSensorOptions()  // clang-format off
        : ProgramOptions("findsensor", {
            { 'p', true,  "port"       },
            { 's', false, "server"     },
            { 'u', true,  "uid"        },
            { 'i', true,  "interface"  },
            { 'm', true,  "multi-addr" },
            { 't', true,  "timeout"    },
            { 'j', false, "json"       },
            { 'T', true,  "ttl"        },
            { 'P', true,  "prop"       },
         }) {};  // clang-format on

    uint16_t port_ = 8952;
    bool server_ = false;
    std::string uid_;
    std::vector<std::string> ifs_;  //  = { "eth0", "wlan0", "wlan1" };
    std::string multi_addr_ =
        "239.255.89.52";  // organization-local scope, https://en.wikipedia.org/wiki/Multicast_address
    int ttl_ = 32;        // same site/organization
    double timeout_ = 1.5;
    bool json_ = false;
    std::map<std::string, std::string> props_;

    void PrintHelp() override final
    {
        // clang-format off
        std::fputs(
            "\n"
            "Tool to find Fixposition sensors on the local (ethernet) network\n"
            "\n"
            "Usage:\n"
            "\n"
            "    findsensor [flags]\n"
            "\n"
            "Where:\n"
            "\n", stdout);
        std::fputs(COMMON_FLAGS_HELP, stdout);
        std::fputs(
            "    -p <port>, --port <port>  -- Port number to use (default: 8952)\n"
            "    -u <uid>, --uid <uid>     -- Look for a particular sensor (default: report all found sensors)\n"
            "    -t <dur>, --timeout <dur> -- Timeout waiting for response [s] (default: 1.5)\n"
            "    -j, --json                -- Output JSON object response (to stdout, one line per response)\n"
            "\n"
            "Notes:\n"
            "\n"
            "    - The sensor discovery functionality is not available for all Fixposition sensors and/or\n"
            "      software versions\n"
            "    - The sensor discovery uses IPv4 UDP multicast. As such the functionality is subject to\n"
            "      your local network setup and your system configuration (router and network configuratuion,\n"
            "      firewall configuration, etc.).\n"
            "\n"
            "Examples:\n"
            "\n"
            "    Look for any sensor in the network and print the information about the found sensors:\n"
            "\n"
            "        $ findsensor\n"
            "\n"
            "        Querying (timeout 1.5s) ...\n"
            "        Found fp-6d9d2c (VRTK2_STK)\n"
            "            - Interface eth0\n"
            "                - Address 172.22.1.60/20\n"
            "            - Interface wlan0\n"
            "                - Address 192.168.43.156/24\n"
            "            - Interface wlan1\n"
            "                - Address 10.0.1.1/24\n"
            "        Found 1 sensor\n"
            "\n"
            "    Quietly look for a particular sensor and print the information as JSON:\n"
            "\n"
            "        $ findsensor --quiet --uid xf-a0d2d8 --json\n"
            "\n"
            "        {\"ifs\":{\"eth0\":[\"172.22.1.60/20\"],\"wlan0\":[\"192.168.43.156/24\"],\"wlan1\":[\"10.0.1.1/24\"]},\\\n"
            "         \"props\":{\"product_model\":\"VRTK2_STK\"},\"uid\":\"fp-6d9d2c\"}\n"
            "\n"
            "\n", stdout);
        // clang-format on

        // And undocumented server code for testing:
        //
        //     findsensor -i eth0 -i wlan0 -p 8952 -s -u someuid -vv -P key=val
        //
        // Testing:
        //
        //     perl -e 'print(pack("NN", 0x66703f3f, 0x71756572))' | socat - udp4-sendto:239.255.89.52:8952
        //     http://www.dest-unreach.org/socat/doc/socat-multicast.html
        //     sudo tcpdump -nnvvvXX udp port 8952
        //
    }

    bool HandleOption(const Option& option, const std::string& argument) final
    {
        bool ok = true;
        switch (option.flag) {
            case 'p':
                if ((port_ != 0) || !StrToValue(argument, port_) || (port_ < 1024)) {
                    WARNING("Bad --port %s", argument.c_str());
                    ok = false;
                }
                break;
            case 'u':
                uid_ = argument;
                break;
            case 's':
                server_ = true;
                break;
            case 'i':
                ifs_.push_back(argument);
                break;
            case 'm':
                multi_addr_ = argument;
                break;
            case 't':
                if (!StrToValue(argument, timeout_) || (timeout_ < 0.1) || (timeout_ > 60.0)) {
                    WARNING("Bad --timeout %s", argument.c_str());
                    ok = false;
                }
                break;
            case 'j':
                json_ = true;
                break;
            case 'T':
                if (!StrToValue(argument, ttl_) || (ttl_ < 1) || (ttl_ > 255)) {
                    WARNING("Bad --ttl %s", argument.c_str());
                    ok = false;
                }
                break;
            case 'P': {
                const auto kv = StrSplit(argument, "=", 2);
                if (kv.size() == 2) {
                    props_[kv[0]] = kv[1];
                } else {
                    WARNING("Bad --prop %s", argument.c_str());
                    ok = false;
                }
                break;
            }
            default:
                ok = false;
                break;
        }
        return ok;
    }

    bool CheckOptions(const std::vector<std::string>& args) final
    {
        bool ok = true;

        if (!args.empty()) {
            WARNING("Spurious command line arguments: %s", StrJoin(args, ", ").c_str());
            return false;
        }

        if (server_) {
            if (uid_.empty()) {
                WARNING("Need -u");
                ok = false;
            }
            MakeUnique(ifs_);
        }

        boost::system::error_code ec;
        ip::make_address(multi_addr_, ec);
        if (ec) {
            WARNING("Bad multicast address %s: %s", multi_addr_.c_str(), ec.message().c_str());
            ok = false;
        }

        DEBUG("server     = %s", ToStr(server_));
        DEBUG("port       = %" PRIu16, port_);
        DEBUG("uid        = %s", uid_.c_str());
        DEBUG("interfaces = %s", StrJoin(ifs_, ", ").c_str());
        DEBUG("multi_addr = %s", multi_addr_.c_str());
        DEBUG("ttl        = %d", ttl_);
        DEBUG("timeout    = %.1f", timeout_);
        DEBUG("json       = %s", ToStr(json_));
        DEBUG("props      = %s", PropsToStr(props_).c_str());

        return ok;
    }
};

/* ****************************************************************************************************************** */

class FindSensor
{
   public:
    FindSensor(const FindSensorOptions& opts);

    bool Run();

    struct Data
    {
        Data() = default;
        std::string uid;
        std::map<std::string, std::vector<std::string>> ifs;
        std::map<std::string, std::string> props;
    };
    // Cannot use the helper macro, as the version in noetic does not have this feature yet. Instead, we have to
    // manually define to_json() and from_json(), see below, and for this the type has to be public. :-/
    // NLOHMANN_DEFINE_TYPE_INTRUSIVE_WITH_DEFAULT(Data, uid, ifs, props)

   private:
    FindSensorOptions opts_;  //!< Program options

    // clang-format off
    enum class OpCode { UNSPECIFIED, QUERY, IDENT };
    const char* OpCodeToStr(const OpCode opcode) const;
    static constexpr std::array<uint8_t, 4> MAGIC = { { 0x66, 0x70, 0x3f, 0x3f } };  // "fp??"
    static constexpr std::array<uint8_t, 4> QUERY = { { 0x71, 0x75, 0x65, 0x72 } };  // "quer"
    static constexpr std::array<uint8_t, 4> IDENT = { { 0x69, 0x64, 0x65, 0x6e } };  // "iden"
    static constexpr std::size_t  HEADER_SIZE = MAGIC.size() + QUERY.size();
    static constexpr std::size_t  MAX_SIZE    = 4 * 1024;
    static constexpr std::size_t  MAX_PAYLOAD = MAX_SIZE - HEADER_SIZE;
    // clang-format on

    struct Socket : private NoCopyNoMove
    {
        Socket(const ip::udp::endpoint& local_endpoint, const ip::udp::endpoint& multi_endpoint, io_context& ctx,
            const FindSensorOptions& opts);
        ~Socket();
        bool Start();
        void Stop();
        bool Send(const uint8_t* data, const std::size_t size);

        bool init_ = false;
        std::atomic<bool> ok_ = true;
        std::string name_;
        ip::udp::endpoint local_endpoint_;
        ip::udp::endpoint multi_endpoint_;
        ip::udp::socket socket_;
        ip::udp::endpoint sender_;
        uint8_t rx_buf_data_[MAX_SIZE];
        mutable_buffer rx_buf_;
        FindSensorOptions opts_;
    };

    io_context ctx_;
    std::vector<std::unique_ptr<Socket>> socks_;
    std::vector<Data> idents_;  // @todo check for duplicates (perhaps we want to retry the QUERY?)

    bool SendPacket(Socket* sock, const OpCode opcode, const json& payload = {});
    void HandlePacket(const uint8_t* data, const std::size_t size, Socket* sock);

    Data GetIdent() const;
    void PrintIdent(const Data& data) const;
    bool HaveIdent(const std::string& uid) const;

    void DoRead(Socket* sock);
    void OnRead(const boost::system::error_code& ec, std::size_t bytes_transferred, Socket* sock);
};

inline void from_json(const json& j, FindSensor::Data& d)
{
    j.at("uid").get_to(d.uid);
    j.at("props").get_to(d.props);
    for (auto& [k, v] : j.at("ifs").items()) {
        d.ifs[k] = {};
        for (auto& vv : v) {
            d.ifs[k].push_back(vv);
        }
    }
}

inline void to_json(json& j, const FindSensor::Data& d)
{
    auto ji = json::object();
    for (auto& [k, v] : d.ifs) {
        auto ja = json::array();
        for (auto& vv : v) {
            ja.push_back(vv);
        }
        ji[k] = ja;
    }
    j = json::object({ { "uid", d.uid }, { "ifs", ji }, { "props", d.props } });
}

// ---------------------------------------------------------------------------------------------------------------------

FindSensor::FindSensor(const FindSensorOptions& opts) /* clang-format off */ :
    opts_   { opts }  // clang-format on
{
}

// ---------------------------------------------------------------------------------------------------------------------

static std::string HostPortStr(const ip::udp::endpoint& endpoint)
{
    return (endpoint.address().is_v6() ? "[" : "") + endpoint.address().to_string() +
           (endpoint.address().is_v6() ? "]" : "") + ":" + std::to_string(endpoint.port());
}

// ---------------------------------------------------------------------------------------------------------------------

FindSensor::Socket::Socket(const ip::udp::endpoint& local_endpoint, const ip::udp::endpoint& multi_endpoint,
    io_context& ctx, const FindSensorOptions& opts) /* clang-format off */ :
    name_             { HostPortStr(local_endpoint) + "/" + HostPortStr(multi_endpoint) },
    local_endpoint_   { local_endpoint },
    multi_endpoint_   { multi_endpoint },
    socket_           { ctx },
    rx_buf_           { buffer(rx_buf_data_) },
    opts_             { opts }  // clang-format on
{
    TRACE("Socket(%s) ctor", name_.c_str());
}

FindSensor::Socket::~Socket()
{
    TRACE("Socket(%s) dtor", name_.c_str());
    Stop();
}

bool FindSensor::Socket::Start()
{
    if (init_) {
        return false;
    }
    TRACE("Socket(%s) start", name_.c_str());

    boost::system::error_code ec;

    // rx socket (listener)

    socket_.open(local_endpoint_.protocol(), ec);
    if (ec) {
        WARNING("Socket(%s) open fail: %s", name_.c_str(), ec.message().c_str());
        return false;
    }

    socket_.set_option(socket_base::reuse_address(true), ec);
    if (ec) {
        WARNING("Socket(%s) set_option(reuse_address) fail: %s", name_.c_str(), ec.message().c_str());
        return false;
    }

    socket_.set_option(socket_base::broadcast(true), ec);
    if (ec) {
        WARNING("Socket(%s) set_option(broadcast) fail: %s", name_.c_str(), ec.message().c_str());
        return false;
    }

    socket_.set_option(ip::multicast::hops(opts_.ttl_), ec);
    if (ec) {
        WARNING("Socket(%s) set_option(hops) fail: %s", name_.c_str(), ec.message().c_str());
        return false;
    }

    socket_.set_option(ip::multicast::enable_loopback(true), ec);
    if (ec) {
        WARNING("Socket(%s) set_option(enable_loopback) fail: %s", name_.c_str(), ec.message().c_str());
        return false;
    }

    socket_.bind(local_endpoint_, ec);
    if (ec) {
        WARNING("Socket(%s) bind fail: %s", name_.c_str(), ec.message().c_str());
        return false;
    }

    socket_.set_option(ip::multicast::outbound_interface(ip::address_v4::any()), ec);
    if (ec) {
        WARNING("Socket(%s) set_option(outbound_interface) fail: %s", name_.c_str(), ec.message().c_str());
        return false;
    }

    socket_.set_option(ip::multicast::join_group(multi_endpoint_.address()), ec);
    if (ec) {
        WARNING("Socket(%s) set_option(join_group) fail: %s", name_.c_str(), ec.message().c_str());
        return false;
    }

    init_ = true;
    ok_ = true;
    return true;
}

void FindSensor::Socket::Stop()
{
    if (!init_) {
        return;
    }
    TRACE("Socket(%s) stop", name_.c_str());

    boost::system::error_code ec;
    if (socket_.is_open()) {
        socket_.cancel(ec);
        if (ec) {
            WARNING("Socket(%s) cancel fail: %s", name_.c_str(), ec.message().c_str());
        }
    }
    if (socket_.is_open()) {
        socket_.close(ec);
        if (ec) {
            WARNING("Socket(%s) close fail: %s", name_.c_str(), ec.message().c_str());
        }
    }

    init_ = false;
}

bool FindSensor::Socket::Send(const uint8_t* data, const std::size_t size)
{
    TRACE_HEXDUMP(data, size, "    ", "Socket(%s) send", name_.c_str());
    boost::system::error_code ec;
    const std::size_t size_sent = socket_.send_to(buffer(data, size), multi_endpoint_, 0, ec);
    if (ec || (size_sent != size)) {
        WARNING("Socket(%s) send fail (%" PRIuMAX ", %" PRIuMAX "): %s", name_.c_str(), size, size_sent,
            ec.message().c_str());
        return false;
    }
    return true;
}

// ---------------------------------------------------------------------------------------------------------------------

const char* FindSensor::OpCodeToStr(const OpCode opcode) const
{
    switch (opcode) {  // clang-format off
        case OpCode::UNSPECIFIED: return "UNSPECIFIED";
        case OpCode::QUERY:       return "QUERY";
        case OpCode::IDENT:       return "IDENT";
    }  // clang-format on
    return "?";
}

// ---------------------------------------------------------------------------------------------------------------------

bool FindSensor::Run()
{
    opts_.LogVersion();

    socks_.push_back(std::make_unique<Socket>(ip::udp::endpoint(ip::address_v4::any(), opts_.port_),
        ip::udp::endpoint(ip::make_address(opts_.multi_addr_), opts_.port_), ctx_, opts_));

    for (auto& sock : socks_) {
        if (!sock || !sock->Start()) {
            socks_.clear();
            return false;
        }
    }

    bool ok = true;

    // Start reading
    for (auto& sock : socks_) {
        DoRead(sock.get());
    }

    // Client: send query
    steady_timer timer(ctx_);
    bool cancel = false;
    if (!opts_.server_) {
        auto query = json::object();
        if (!opts_.uid_.empty()) {
            query["uid"] = opts_.uid_;
            INFO("Querying for %s (timeout %.1fs) ...", opts_.uid_.c_str(), opts_.timeout_);
        } else {
            INFO("Querying (timeout %.1fs) ...", opts_.timeout_);
        }
        for (auto& sock : socks_) {
            if (!SendPacket(sock.get(), OpCode::QUERY, query)) {
                ok = false;
            }
        }
        // Setup timeout
        timer.expires_after(std::chrono::milliseconds((int)(opts_.timeout_ * 1e3)));
        timer.async_wait([&cancel](const boost::system::error_code& /*ec*/) { cancel = true; });
    }
    // Server
    else {
        INFO("Listening on port %" PRIu16, opts_.port_);
    }

    SigIntHelper sigint;    // CTRL-C
    SigTermHelper sigterm;  // daemon stop
    auto work = make_work_guard(ctx_);
    while (ok && !cancel && !sigint.ShouldAbort() && !sigterm.ShouldAbort()) {
        ctx_.run_for(std::chrono::milliseconds(123));
        for (auto& sock : socks_) {
            if (!sock->ok_) {
                ok = false;
            }
        }
        // We found what we were looking for
        if (!opts_.server_ && !opts_.uid_.empty() && HaveIdent(opts_.uid_)) {
            break;
        }
    }

    // Shutdown
    DEBUG("Stopping...");
    for (auto& sock : socks_) {
        sock->Stop();
    }
    socks_.clear();

    // We haven't found what we were looking for
    if (!opts_.server_) {
        if (opts_.uid_.empty()) {
            if (!idents_.empty()) {
                INFO("Found %" PRIuMAX " sensor%s", idents_.size(), idents_.size() > 1 ? "s" : "");
            } else {
                WARNING("Could not find any sensors");
                ok = false;
            }
        } else {
            if (HaveIdent(opts_.uid_)) {
                INFO("Found sensor %s", opts_.uid_.c_str());
            } else {
                WARNING("Could not find sensor %s", opts_.uid_.c_str());
                ok = false;
            }
        }
    }

    return ok;
}

// ---------------------------------------------------------------------------------------------------------------------

bool FindSensor::SendPacket(Socket* sock, const OpCode opcode, const json& payload)
{
    if (!sock) {
        return false;
    }
    const auto payload_str = (payload.empty() ? "" : payload.dump());
    if (payload_str.size() > MAX_PAYLOAD) {
        WARNING("SendPacket %s payload too long", sock->name_.c_str());
        return false;
    }

    uint8_t data[MAX_SIZE];
    std::memcpy(&data[0], MAGIC.data(), MAGIC.size());

    switch (opcode) {
        case OpCode::UNSPECIFIED:
            return false;
        case OpCode::QUERY:
            std::memcpy(&data[MAGIC.size()], QUERY.data(), QUERY.size());
            break;
        case OpCode::IDENT:
            std::memcpy(&data[MAGIC.size()], IDENT.data(), IDENT.size());
            break;
    }

    if (!payload_str.empty()) {
        const auto payload_buf = StrToBuf(payload_str);
        std::memcpy(&data[HEADER_SIZE], payload_buf.data(), payload_buf.size());
    }
    const std::size_t size = HEADER_SIZE + payload_str.size();

    DEBUG("SendPacket %s %s %s", sock->name_.c_str(), OpCodeToStr(opcode), payload_str.c_str());

    return sock->Send(data, size);
}

// ---------------------------------------------------------------------------------------------------------------------

// Packets are:
// - MAGIC QUERY                             wildcard query, always answer with IDENT
// - MAGIC QUERY { "uid": "" }               wildcard query, always answer with IDENT
// - MAGIC QUERY { "uid": "someuid" }        specific query, answer with IDENT if we're "someuid"
// - MAGIC IDENT { "uid": ..., "ifs": ... }  response to query

void FindSensor::HandlePacket(const uint8_t* data, const std::size_t size, Socket* sock)
{
    TRACE_HEXDUMP(data, size, "    ", "%s < %s %" PRIuMAX " bytes", sock->name_.c_str(),
        HostPortStr(sock->sender_).c_str(), size);

    if (size < HEADER_SIZE) {
        TRACE("%s < %s ignore (no header)", sock->name_.c_str(), HostPortStr(sock->sender_).c_str());
        return;
    }

    if (std::memcmp(data, MAGIC.data(), MAGIC.size()) != 0) {
        TRACE("%s < %s ignore (bad magic)", sock->name_.c_str(), HostPortStr(sock->sender_).c_str());
        return;
    }

    OpCode opcode = OpCode::UNSPECIFIED;
    if (std::memcmp(&data[MAGIC.size()], QUERY.data(), QUERY.size()) == 0) {
        opcode = OpCode::QUERY;
    } else if (std::memcmp(&data[MAGIC.size()], IDENT.data(), IDENT.size()) == 0) {
        opcode = OpCode::IDENT;
    }

    bool ok = true;
    if ((opcode == OpCode::QUERY) || (opcode == OpCode::IDENT)) {
        // Decode json payload
        Data payload;
        if (size > HEADER_SIZE) {
            try {
                payload = json::parse(BufToStr({ &data[HEADER_SIZE], &data[HEADER_SIZE] + size - HEADER_SIZE }))
                              .get<fpsdk::apps::findsensor::FindSensor::Data>();
            } catch (const std::exception& ex) {
                // WARNING("%s < %s bad %s: %s (got: %s)", sock->name_.c_str(), HostPortStr(sock->sender_).c_str(),
                //     OpCodeToStr(opcode), ex.what(), json(payload).dump().c_str());
                return;
            }
        }
        DEBUG("%s < %s %s uid=%s", sock->name_.c_str(), HostPortStr(sock->sender_).c_str(), OpCodeToStr(opcode),
            payload.uid.c_str());

        // Server
        if (opts_.server_) {
            // QUERY: respond to wildcard query (empty uid) and query for our uid
            if ((opcode == OpCode::QUERY) && (payload.uid.empty() || (payload.uid == opts_.uid_))) {
                if (!SendPacket(sock, OpCode::IDENT, GetIdent())) {
                    sock->ok_ = false;
                }
            }
            // IDENT: ignore
            else if (opcode != OpCode::IDENT) {
                ok = false;
            }
        }
        // Client
        else {
            // IDENT: print data
            if ((opcode == OpCode::IDENT) && (opts_.uid_.empty() || (payload.uid == opts_.uid_))) {
                idents_.push_back(payload);
                PrintIdent(payload);
            }
            // QUERY: ignore
            else if (opcode != OpCode::QUERY) {
                ok = false;
            }
        }
    }

    // Unknown
    if (!ok) {
        uint32_t op;
        std::memcpy(&op, &data[MAGIC.size()], sizeof(op));
        DEBUG("%s < %s unhandled opcode 0x%" PRIx32, sock->name_.c_str(), HostPortStr(sock->sender_).c_str(), op);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

static int CountBits(uint32_t mask)
{
    int count = 0;
    while (mask) {
        count += (mask & 0x1);
        mask >>= 1;
    }
    return count;
}

static int CountBits(const uint32_t masks[4])
{
    int count = 0;
    for (int ix = 0; ix < 4; ix++) {
        uint32_t mask = masks[ix];
        while (mask) {
            count += (mask & 0x1);
            mask >>= 1;
        }
    }
    return count;
}

FindSensor::Data FindSensor::GetIdent() const
{
    Data ident;
    ident.uid = opts_.uid_;
    ident.props = opts_.props_;

    // Get all addresses of all interfaces
    struct ifaddrs* ifaddr;
    if (getifaddrs(&ifaddr) == 0) {
        for (struct ifaddrs* ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
            if (ifa->ifa_addr == nullptr) {
                continue;
            }
            const auto sa_family = ifa->ifa_addr->sa_family;
            if (!((sa_family == AF_INET) || (sa_family == AF_INET6))) {
                continue;
            }
            const std::string ifa_name = ifa->ifa_name;
            if (!opts_.ifs_.empty() &&
                (std::find(opts_.ifs_.begin(), opts_.ifs_.end(), ifa_name) == opts_.ifs_.end())) {
                continue;
            }
            if (ident.ifs.count(ifa_name) == 0) {
                ident.ifs[ifa_name] = {};
            }

            if (ifa->ifa_addr->sa_family == AF_INET) {  // IPv4

                struct sockaddr_in* addr = reinterpret_cast<struct sockaddr_in*>(ifa->ifa_addr);
                struct sockaddr_in* netmask = reinterpret_cast<struct sockaddr_in*>(ifa->ifa_netmask);
                char addr_str[INET_ADDRSTRLEN];
                if (inet_ntop(ifa->ifa_addr->sa_family, &addr->sin_addr, addr_str, sizeof(addr_str))) {
                    ident.ifs[ifa_name].push_back(
                        std::string(addr_str) + "/" + std::to_string(CountBits(netmask->sin_addr.s_addr)));
                }
            } else if (ifa->ifa_addr->sa_family == AF_INET6) {  // IPv6

                struct sockaddr_in6* addr = reinterpret_cast<struct sockaddr_in6*>(ifa->ifa_addr);
                struct sockaddr_in6* netmask = reinterpret_cast<struct sockaddr_in6*>(ifa->ifa_netmask);
                char addr_str[INET6_ADDRSTRLEN];
                if (inet_ntop(ifa->ifa_addr->sa_family, &addr->sin6_addr, addr_str, sizeof(addr_str))) {
                    ident.ifs[ifa_name].push_back(std::string(addr_str) + "/" +
                                                  std::to_string(CountBits(netmask->sin6_addr.__in6_u.__u6_addr32)));
                }
            }
        }
        freeifaddrs(ifaddr);

    } else {
        WARNING("getifaddrs fail: %s", StrError(errno).c_str());
    }

    return ident;
}

// ---------------------------------------------------------------------------------------------------------------------

void FindSensor::PrintIdent(const Data& data) const
{
    if (opts_.json_) {
        std::printf("%s\n", json(data).dump().c_str());
        return;
    }

    const auto product_model_entry = data.props.find("product_model");

    NOTICE("Found %s (%s)", data.uid.c_str(),
        product_model_entry == data.props.end() ? "unknown model" : product_model_entry->second.c_str());

    for (auto& [name, addrs] : data.ifs) {
        INFO("    - Interface %s", name.c_str());
        for (auto& addr : addrs) {
            INFO("        - Address %s", addr.c_str());
        }
    }
}

// ---------------------------------------------------------------------------------------------------------------------

bool FindSensor::HaveIdent(const std::string& uid) const
{
    return std::find_if(idents_.begin(), idents_.end(), [&uid](const auto& cand) { return cand.uid == uid; }) !=
           idents_.end();
}

// ---------------------------------------------------------------------------------------------------------------------

void FindSensor::DoRead(Socket* sock)
{
    TRACE("DoRead %s", sock->name_.c_str());
    sock->socket_.async_receive_from(sock->rx_buf_, std::ref(sock->sender_),
        std::bind(&FindSensor::OnRead, this, std::placeholders::_1, std::placeholders::_2, sock));
}

void FindSensor::OnRead(const boost::system::error_code& ec, std::size_t bytes_transferred, Socket* sock)
{
    TRACE("OnRead %s %s %" PRIuMAX " %s", sock->name_.c_str(), HostPortStr(sock->sender_).c_str(), bytes_transferred,
        ec.message().c_str());

    // We got cancelled, do nothing
    if (ec == error::operation_aborted) {
    }
    // Error reading (connection lost)
    else if (ec) {
        WARNING("read fail: %s", ec.message().c_str());
        sock->ok_ = false;
    }
    // Process incoming data, and read again
    else {
        HandlePacket((const uint8_t*)sock->rx_buf_.data(), bytes_transferred, sock);
        DoRead(sock);
    }
}

/* ****************************************************************************************************************** */
}  // namespace findsensor
}  // namespace apps
}  // namespace fpsdk

/* ****************************************************************************************************************** */

int main(int argc, char** argv)
{
    using namespace fpsdk::apps::findsensor;
#ifndef NDEBUG
    fpsdk::common::app::StacktraceHelper stacktrace;
#endif
    bool ok = true;

    // Parse command line arguments
    FindSensorOptions opts;
    if (!opts.LoadFromArgv(argc, argv)) {
        ok = false;
    }

    if (ok) {
        FindSensor app(opts);
        ok = app.Run();
    }

    // Are we happy?
    if (ok) {
        return EXIT_SUCCESS;
    } else {
        ERROR("Failed");
        return EXIT_FAILURE;
    }
}

/* ****************************************************************************************************************** */
