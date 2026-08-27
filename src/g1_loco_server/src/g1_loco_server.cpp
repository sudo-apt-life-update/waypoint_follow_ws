/**
 * g1_loco_server -- drive the G1's locomotion controller from a ZMQ socket.
 *
 * The write-side counterpart to slam_ws's g1_state_server, and it exists for
 * the same reason: unitree_sdk2 and ROS 2 each link their own CycloneDDS, and
 * putting both in one address space corrupts the heap. So everything that
 * touches Unitree DDS lives in this process and the ROS side talks to a plain
 * socket. Its counterpart is loco_bridge.py in the g1_walk package.
 *
 * Wire format: one fixed-size packed struct per request, one per reply, over a
 * ZMQ REP socket. Same choice as G1StatePacket, for consistency and because the
 * peer is a single Python client we also own -- there is nobody to negotiate a
 * self-describing format with. REQ/REP rather than PUB/SUB because a command
 * that silently vanished is worse than one that reports an error code.
 *
 * SAFETY. This is the only process in the project that can make a 35 kg
 * humanoid walk, so the limits live here and not in the Python that calls it:
 *
 *   1. Velocities are clamped to --max-vx / --max-vy / --max-omega. A bug in
 *      the ROS layer cannot command full speed.
 *   2. Every SetVelocity carries a duration. The robot's own controller stops
 *      when it expires, so a crashed or disconnected client stops the robot
 *      even if this process dies with it. Durations are clamped to
 *      --max-duration; continuous mode (864000 s) is deliberately unreachable.
 *   3. A host-side watchdog stops the robot if no command arrives within
 *      --watchdog seconds while it was last told to move.
 *   4. SIGINT/SIGTERM issue StopMove before exiting.
 *
 * None of that substitutes for a hand on the remote. Keep the robot on the
 * gantry or keep the E-stop within reach.
 *
 * Build:  colcon build --packages-select g1_loco_server
 * Run:    ./install/g1_loco_server/lib/g1_loco_server/g1_loco_server [--iface=eno1]
 *                                                    [--bind=tcp://127.0.0.1:5558]
 */

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <csignal>
#include <iostream>
#include <map>
#include <memory>
#include <string>

#include <zmq.hpp>

#include <dds/ddsrt/log.h>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/robot/g1/loco/g1_loco_api.hpp>
#include <unitree/robot/g1/loco/g1_loco_client.hpp>

static constexpr uint32_t kReqMagic = 0x47314344;  // 'G1CD'
static constexpr uint32_t kRepMagic = 0x47315250;  // 'G1RP'
static constexpr uint32_t kOdomMagic = 0x47314F44;  // 'G1OD'
static constexpr uint32_t kVersion = 1;

/** The robot's own state estimator, found by DDS discovery on 2026-08-13.
 *
 *  Not documented for the G1 and not reachable through the loco RPC (ids
 *  7201-7203 return 3203, "api not implemented"). It is published as go2's
 *  SportModeState_ at ~500 Hz, and it is NOT LiDAR-derived -- no LiDAR SLAM
 *  emits pose at 500 Hz, position.z tracks pelvis height, and the identical
 *  values appear on rt/dog_odom. This is the locomotion controller's own leg
 *  odometry fused with the IMU, which is exactly what we were about to write
 *  by hand. Using it keeps the project LiDAR-free.
 *
 *  Validated against a tape measure: over one 7.5 s walk it reported 1.37 m
 *  forward and 0.52 m right (-21 deg), matching the observed drift, and it
 *  captured the backwards retreat at the end of the segment. It is odometry,
 *  so it drifts and has no loop closure -- fine for per-segment feedback,
 *  not a map. */
static const std::string kOdomTopic = "rt/odommodestate";

/** Command codes. Mirrored by Command in loco_bridge.py. */
enum Command : uint32_t
{
    CMD_PING = 0,
    CMD_VELOCITY = 1,       // vx, vy, omega, duration
    CMD_STOP = 2,           // SetVelocity(0,0,0)
    CMD_FSM = 3,            // ivalue = FSM id (see kFsmNames)
    CMD_BALANCE_MODE = 4,   // ivalue: 0 = balance stand, 1 = continuous gait
    CMD_STAND_HEIGHT = 5,   // fvalue [m]
    CMD_SWING_HEIGHT = 6,   // fvalue [m]
    CMD_SPEED_MODE = 7,     // ivalue
    CMD_STATUS = 8,         // read back fsm id / mode / balance mode
    CMD_HALT = 9,           // stop AND leave continuous gait -- see doHalt
};

/** FSM ids. Named here so the log says "stand_up" rather than "4", and so an
 *  unknown id is rejected rather than posted blindly to a controller that
 *  decides what to do with it.
 *
 *  [THE 500 vs 200 TRAP] LocoClient::Start() hardcodes SetFsmId(500), but on
 *  this robot (2026-08-10) sending 500 returns success and leaves the FSM at 4:
 *  the id is accepted and nothing happens, so every later SetVelocity is
 *  silently discarded because the robot is not in main operation. 200 is the
 *  G1's "Main Operation Control" state. Both are listed; `main_operation` (200)
 *  is the one to reach for, and `start` (500) is kept only because the SDK
 *  calls it that.
 *
 *  Verify with `loco_cli status` after any transition -- a return code of 0
 *  from SetFsmId does NOT mean the state changed. That is the whole lesson
 *  here. */
static const std::map<int32_t, std::string> kFsmNames = {
    {0, "zero_torque"},
    {1, "damp"},
    {2, "squat"},
    {3, "sit"},
    {4, "stand_up"},
    {200, "main_operation"},
    {500, "start"},
    {702, "lie2standup"},
    {706, "squat2standup"},
};

#pragma pack(push, 1)
struct G1CmdRequest
{
    uint32_t magic;     // kReqMagic
    uint32_t version;   // kVersion
    uint32_t seq;       // echoed back, so a reply can be matched to a request
    uint32_t command;   // one of Command
    float vx;           // [m/s]  forward
    float vy;           // [m/s]  left
    float omega;        // [rad/s] yaw, left-positive
    float duration;     // [s] how long the robot honours this velocity
    float fvalue;       // stand/swing height
    int32_t ivalue;     // fsm id, balance mode, speed mode
};

struct G1CmdReply
{
    uint32_t magic;     // kRepMagic
    uint32_t version;   // kVersion
    uint32_t seq;       // copied from the request
    int32_t ret;        // 0 = ok, otherwise the SDK's error code
    int32_t fsm_id;         // CMD_STATUS only, else -1
    int32_t fsm_mode;       // CMD_STATUS only, else -1
    int32_t balance_mode;   // CMD_STATUS only, else -1
    float clamped_vx;   // what was actually sent, after clamping
    float clamped_vy;
    float clamped_omega;
    float clamped_duration;
};
/** Streamed on a PUB socket, separate from the REQ/REP command channel.
 *
 *  Odometry is state, and state wants a stream: a 500 Hz signal polled over
 *  request/reply would cost a round trip per sample and go stale between
 *  them. Same split slam_ws uses -- PUB for state, and here REQ/REP stays for
 *  commands, where a silently dropped message would be unacceptable.
 *
 *  It lives in this process rather than a third one because the DDS
 *  participant already exists here, and because every walk already requires
 *  this server to be running. One fewer terminal to forget. */
struct G1OdomPacket
{
    uint32_t magic;     // kOdomMagic
    uint32_t version;   // kVersion
    uint64_t stamp_ns;  // CLOCK_REALTIME, directly usable by ROS
    float x;            // [m] odom frame, cumulative, drifts
    float y;
    float z;            // tracks pelvis height (~0.70 m standing)
    float qw;           // attitude, Unitree order (w, x, y, z)
    float qx;
    float qy;
    float qz;
    float vx;           // [m/s] as published by the controller
    float vy;
    float vz;
    float yaw_speed;    // [rad/s]
    uint32_t seq;       // increments per message received from the robot
    uint32_t pad;
};
#pragma pack(pop)

static_assert(sizeof(G1OdomPacket) == 68,
              "G1OdomPacket layout changed -- update ODOM_FMT in "
              "loco_protocol.py to match");

static_assert(sizeof(G1CmdRequest) == 40,
              "G1CmdRequest layout changed -- update REQUEST_FMT in "
              "loco_bridge.py to match");
static_assert(sizeof(G1CmdReply) == 44,
              "G1CmdReply layout changed -- update REPLY_FMT in "
              "loco_bridge.py to match");

/** Error codes we invent, kept clear of the SDK's own range. */
static constexpr int32_t kErrBadCommand = -1001;

/** Odometry republish rate. The robot sends ~500 Hz; a walking control loop
 *  runs at 20 Hz, so 100 Hz is generous and keeps the socket quiet. */
static constexpr double kOdomPublishHz = 100.0;

static std::atomic<bool> g_running{true};

/**
 * Drop CycloneDDS's per-retry transmit chatter, keep everything else.
 *
 * When the link to the robot goes down, CycloneDDS logs
 *   "ddsi_udp_conn_write to udp/192.168.123.1:47097 failed with retcode -1"
 * on every retry -- several lines a second, indefinitely. During a real fault
 * that buried the one line that mattered (the FSM result), and during a walk it
 * would bury everything.
 *
 * [WHY NOT CYCLONEDDS_URI] The obvious fix does not work here. unitree_sdk2
 * passes its own inline XML to dds_create_domain (it substitutes the interface
 * name into a built-in template), and a programmatic config overrides the
 * environment variable -- verified: the config below changes nothing about
 * this process's output. Worse, ROS 2 on this machine runs
 * RMW_IMPLEMENTATION=rmw_cyclonedds_cpp, so exporting CYCLONEDDS_URI would
 * quietly reconfigure every ROS node while still not touching this one.
 *
 * So filter at the log sink instead, which is process-local and precise.
 * The first occurrence is kept -- a silent link failure would be worse than a
 * noisy one -- and repeats are counted and reported periodically.
 */
static std::atomic<bool> g_filter_dds_chatter{true};
static std::atomic<uint64_t> g_suppressed{0};

/** Substrings whose repeats are noise once the first one has been seen. */
static bool isRetryChatter(const char* message)
{
    if (message == nullptr) return false;
    static const char* kPatterns[] = {
        "ddsi_udp_conn_write",   // one per retry while the link is down
        "failed with retcode",
    };
    for (const char* pattern : kPatterns)
    {
        if (std::strstr(message, pattern) != nullptr) return true;
    }
    return false;
}

static void ddsLogSink(void* /*arg*/, const dds_log_data_t* data)
{
    if (data == nullptr || data->message == nullptr) return;

    if (g_filter_dds_chatter.load() && isRetryChatter(data->message))
    {
        static std::atomic<bool> warned_once{false};
        if (!warned_once.exchange(true))
        {
            std::cerr << data->message
                      << "[g1_loco_server] ^ DDS cannot transmit. Usually the "
                         "link to the robot is down:\n"
                         "  ip addr show <iface>   -- expect UP with an "
                         "address on the robot's subnet\n"
                         "  ping 192.168.123.164\n"
                         "  Further identical messages are suppressed; pass "
                         "--verbose-dds to see them all."
                      << std::endl;
        }
        g_suppressed++;
        return;
    }

    // Everything else through untouched. The message already ends in a newline.
    std::cerr << data->message;
}

static void onSignal(int)
{
    // Async-signal-safe: just flip the flag. The stop-and-exit happens on the
    // main thread, which owns the LocoClient.
    g_running.store(false);
}

/** Wall clock in nanoseconds, so the odometry stamp is directly usable as a
 *  ROS timestamp on the other side of the socket. nowSeconds() below is
 *  steady-clock and deliberately separate: watchdogs must not be affected by
 *  clock steps, timestamps must be comparable with other machines. */
static uint64_t now_ns()
{
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch())
            .count());
}

static double nowSeconds()
{
    return std::chrono::duration<double>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

static float clamp(float value, float limit)
{
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

struct Options
{
    std::string iface = "eno1";
    std::string bind = "tcp://127.0.0.1:5558";
    std::string odom_bind = "tcp://127.0.0.1:5560";
    float max_vx = 0.6f;
    float max_vy = 0.3f;
    float max_omega = 0.6f;
    // A velocity command should outlive the gap between commands but not much
    // more, so a stalled client stops the robot within a stride.
    float max_duration = 2.0f;
    double watchdog = 0.5;
    float sdk_timeout = 10.0f;
    bool verbose_dds = false;
    // Off by default: an arbitrary FSM id posted to a humanoid's controller is
    // not something a typo should be able to do. On when the id mapping needs
    // exploring, which the 500-vs-200 confusion made necessary.
    bool allow_any_fsm = false;
};

static void printUsage()
{
    std::cout
        << "g1_loco_server -- ZMQ -> Unitree G1 locomotion bridge\n\n"
           "  --iface=eno1                  network interface to the robot\n"
           "  --bind=tcp://127.0.0.1:5558   ZMQ REP endpoint (commands)\n"
           "  --odom-bind=tcp://127.0.0.1:5560  ZMQ PUB endpoint "
           "(odometry)\n"
           "  --max-vx=0.6                  forward speed limit  [m/s]\n"
           "  --max-vy=0.3                  lateral speed limit  [m/s]\n"
           "  --max-omega=0.6               yaw rate limit       [rad/s]\n"
           "  --max-duration=2.0            per-command validity [s]\n"
           "  --watchdog=0.5                stop if idle this long while "
           "moving [s]\n"
           "  --verbose-dds                 do not suppress repeated "
           "CycloneDDS transmit errors\n"
           "  --allow-any-fsm               accept FSM ids outside the known "
           "table (exploration)\n"
           "  --help\n"
        << std::endl;
}

/** Parse --key=value. Anything unrecognised is fatal rather than ignored: a
 *  typo in a speed limit must not silently leave the default in place. */
static bool parseArgs(int argc, char** argv, Options& opt)
{
    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        if (arg == "--help" || arg == "-h")
        {
            printUsage();
            std::exit(0);
        }

        if (arg == "--verbose-dds")
        {
            opt.verbose_dds = true;
            continue;
        }

        if (arg == "--allow-any-fsm")
        {
            opt.allow_any_fsm = true;
            continue;
        }

        const size_t eq = arg.find('=');
        if (arg.rfind("--", 0) != 0 || eq == std::string::npos)
        {
            std::cerr << "unrecognised argument: " << arg << "\n";
            printUsage();
            return false;
        }

        const std::string key = arg.substr(2, eq - 2);
        const std::string value = arg.substr(eq + 1);

        if (key == "iface") opt.iface = value;
        else if (key == "bind") opt.bind = value;
        else if (key == "odom-bind") opt.odom_bind = value;
        else if (key == "max-vx") opt.max_vx = std::stof(value);
        else if (key == "max-vy") opt.max_vy = std::stof(value);
        else if (key == "max-omega") opt.max_omega = std::stof(value);
        else if (key == "max-duration") opt.max_duration = std::stof(value);
        else if (key == "watchdog") opt.watchdog = std::stod(value);
        else
        {
            std::cerr << "unrecognised option: --" << key << "\n";
            printUsage();
            return false;
        }
    }
    return true;
}

class G1LocoServer
{
public:
    explicit G1LocoServer(const Options& opt) : opt_(opt)
    {
        // [ORDER MATTERS] Unitree DDS must be initialised before the ZMQ
        // context exists. g1_state_server documents the same constraint: if
        // zmq allocates first, ChannelFactory::Init aborts with "corrupted
        // size vs. prev_size". Do not hoist the socket setup above this point.
        std::cout << "Initializing Unitree DDS on interface: " << opt_.iface
                  << std::endl;

        // Before Init, so the very first transmit failure is already filtered.
        g_filter_dds_chatter.store(!opt_.verbose_dds);
        dds_set_log_sink(&ddsLogSink, nullptr);

        unitree::robot::ChannelFactory::Instance()->Init(0, opt_.iface);

        // [ORDER MATTERS] Construct the client here, not as a value member.
        // LocoClient's base Client reaches into the ChannelFactory singleton
        // while constructing, so a value member -- built in the initialiser
        // list, before the line above runs -- segfaults before main() has
        // printed anything useful. Every SDK example constructs its client
        // after Init for this reason.
        client_ = std::make_unique<unitree::robot::g1::LocoClient>();

        client_->Init();
        client_->SetTimeout(opt_.sdk_timeout);

        // Explicitly single-shot: every Move() we issue carries its own
        // duration, so the robot stops on its own if we stop talking. The
        // continuous mode this flag would enable (864000 s) is exactly the
        // failure we are protecting against.
        client_->SwitchMoveMode(false);

        ctx_ = std::make_unique<zmq::context_t>(1);
        rep_ = std::make_unique<zmq::socket_t>(*ctx_, zmq::socket_type::rep);
        rep_->set(zmq::sockopt::linger, 0);

        odom_pub_ = std::make_unique<zmq::socket_t>(*ctx_,
                                                    zmq::socket_type::pub);
        // Drop rather than queue: odometry is only useful fresh, and an
        // unbounded send queue would turn a slow consumer into latency.
        odom_pub_->set(zmq::sockopt::sndhwm, 1);
        odom_pub_->set(zmq::sockopt::linger, 0);
        try
        {
            odom_pub_->bind(opt_.odom_bind);
        }
        catch (const zmq::error_t& e)
        {
            std::cerr << "[g1_loco_server] cannot bind " << opt_.odom_bind
                      << ": " << e.what()
                      << "\n  Another instance is running, or pick another "
                         "port with --odom-bind.\n";
            std::exit(1);
        }

        std::memset(&odom_, 0, sizeof(odom_));
        odom_.magic = kOdomMagic;
        odom_.version = kVersion;
        odom_.qw = 1.0f;

        try
        {
            rep_->bind(opt_.bind);
        }
        catch (const zmq::error_t& e)
        {
            std::cerr << "\n[g1_loco_server] cannot bind " << opt_.bind << ": "
                      << e.what() << "\n"
                      << "  Another g1_loco_server is probably already "
                         "running.\n"
                      << "  Check with : ss -ltnp | grep 5558\n"
                      << "  Stop it    : pkill -x g1_loco_server\n"
                      << "  Or use a different port:\n"
                      << "               g1_loco_server "
                         "--bind=tcp://127.0.0.1:5559\n"
                      << "  (then start the ROS side with port:=5559)\n"
                      << std::endl;
            std::exit(1);
        }

        odom_sub_ = std::make_shared<unitree::robot::ChannelSubscriber<
            unitree_go::msg::dds_::SportModeState_>>(kOdomTopic);
        odom_sub_->InitChannel(
            std::bind(&G1LocoServer::onOdom, this, std::placeholders::_1), 10);

        std::cout << "Odometry: " << kOdomTopic << " -> " << opt_.odom_bind
                  << "\n";
        std::cout << "Listening on " << opt_.bind << "\n"
                  << "Limits: vx " << opt_.max_vx << " m/s, vy " << opt_.max_vy
                  << " m/s, omega " << opt_.max_omega << " rad/s, duration "
                  << opt_.max_duration << " s, watchdog " << opt_.watchdog
                  << " s" << std::endl;
    }

    void run()
    {
        while (g_running.load())
        {
            zmq::message_t message;
            zmq::pollitem_t items[] = {
                {rep_->handle(), 0, ZMQ_POLLIN, 0}};

            // Poll rather than block, so the watchdog still runs when the
            // client has gone quiet -- which is precisely when it matters.
            //
            // The catch is not optional. A signal lands while we are parked in
            // poll(), cppzmq turns the EINTR into a thrown zmq::error_t, and
            // nothing above catches it -- so the process aborts instead of
            // running the StopMove below. Ctrl-C would leave the robot walking.
            try
            {
                zmq::poll(items, 1, std::chrono::milliseconds(50));

                if (!(items[0].revents & ZMQ_POLLIN))
                {
                    checkWatchdog();
                    reportSuppressed();
                    continue;
                }

                const auto result = rep_->recv(message, zmq::recv_flags::none);
                if (!result) continue;
            }
            catch (const zmq::error_t& e)
            {
                if (e.num() == EINTR) continue;  // signal; g_running decides
                std::cerr << "[g1_loco_server] socket error: " << e.what()
                          << std::endl;
                break;
            }

            G1CmdReply reply{};
            reply.magic = kRepMagic;
            reply.version = kVersion;
            reply.fsm_id = -1;
            reply.fsm_mode = -1;
            reply.balance_mode = -1;

            if (message.size() != sizeof(G1CmdRequest))
            {
                std::cerr << "[g1_loco_server] ignoring " << message.size()
                          << "-byte message, expected " << sizeof(G1CmdRequest)
                          << std::endl;
                reply.ret = kErrBadCommand;
            }
            else
            {
                G1CmdRequest request{};
                std::memcpy(&request, message.data(), sizeof(request));

                if (request.magic != kReqMagic)
                {
                    std::cerr << "[g1_loco_server] bad magic, ignoring"
                              << std::endl;
                    reply.ret = kErrBadCommand;
                }
                else if (request.version != kVersion)
                {
                    std::cerr << "[g1_loco_server] client speaks packet "
                                 "version "
                              << request.version << ", this server expects "
                              << kVersion << ". Rebuild one of them."
                              << std::endl;
                    reply.ret = kErrBadCommand;
                }
                else
                {
                    reply.seq = request.seq;
                    reply.ret = dispatch(request, reply);
                }
            }

            rep_->send(zmq::const_buffer(&reply, sizeof(reply)),
                       zmq::send_flags::none);
        }

        // Leave the robot standing, not walking. This runs whether we exited
        // on a signal or fell out of the loop some other way.
        std::cout << "\n[g1_loco_server] halting the robot before exit"
                  << std::endl;
        // StopMove alone leaves a continuous-gait robot marching after this
        // process is gone, with nothing left to command it.
        G1CmdReply scratch{};
        doHalt(scratch);
    }

private:
    int32_t dispatch(const G1CmdRequest& request, G1CmdReply& reply)
    {
        switch (request.command)
        {
            case CMD_PING:
                return 0;

            case CMD_VELOCITY:
                return doVelocity(request, reply);

            case CMD_STOP:
                // Velocity only. Used at the end of every path segment, so it
                // must NOT touch balance mode -- dropping out of continuous
                // gait between segments would change the gait mid-path.
                moving_ = false;
                reply.clamped_duration = 0.0f;
                return client_->StopMove();

            case CMD_HALT:
                return doHalt(reply);

            case CMD_FSM:
                return doFsm(request);

            case CMD_BALANCE_MODE:
                // 0 = stand in place and balance, 1 = keep stepping. Anything
                // else is not a mode the controller knows.
                if (request.ivalue != 0 && request.ivalue != 1)
                {
                    std::cerr << "[g1_loco_server] balance mode "
                              << request.ivalue << " is not 0 or 1"
                              << std::endl;
                    return kErrBadCommand;
                }
                std::cout << "[g1_loco_server] balance mode -> "
                          << request.ivalue << std::endl;
                return client_->SetBalanceMode(request.ivalue);

            case CMD_STAND_HEIGHT:
                std::cout << "[g1_loco_server] stand height -> "
                          << request.fvalue << std::endl;
                return client_->SetStandHeight(request.fvalue);

            case CMD_SWING_HEIGHT:
                std::cout << "[g1_loco_server] swing height -> "
                          << request.fvalue << std::endl;
                return client_->SetSwingHeight(request.fvalue);

            case CMD_SPEED_MODE:
                std::cout << "[g1_loco_server] speed mode -> "
                          << request.ivalue << std::endl;
                return client_->SetSpeedMode(request.ivalue);

            case CMD_STATUS:
                return doStatus(reply);

            default:
                std::cerr << "[g1_loco_server] unknown command "
                          << request.command << std::endl;
                return kErrBadCommand;
        }
    }

    int32_t doVelocity(const G1CmdRequest& request, G1CmdReply& reply)
    {
        const float vx = clamp(request.vx, opt_.max_vx);
        const float vy = clamp(request.vy, opt_.max_vy);
        const float omega = clamp(request.omega, opt_.max_omega);

        // A zero or negative duration would be honoured as "stop immediately",
        // which is a confusing way to spell CMD_STOP. Treat it as one stride's
        // worth so a client that forgets to set it still moves predictably.
        float duration = request.duration;
        if (!(duration > 0.0f)) duration = 0.3f;
        if (duration > opt_.max_duration) duration = opt_.max_duration;

        reply.clamped_vx = vx;
        reply.clamped_vy = vy;
        reply.clamped_omega = omega;
        reply.clamped_duration = duration;

        if (!clamp_warned_ &&
            (vx != request.vx || vy != request.vy || omega != request.omega))
        {
            // Once, not per command: at 20 Hz this would otherwise bury the
            // log, and the reply carries the clamped values for the client to
            // notice on its own.
            clamp_warned_ = true;
            std::cerr << "[g1_loco_server] clamping velocity (" << request.vx
                      << ", " << request.vy << ", " << request.omega
                      << ") -> (" << vx << ", " << vy << ", " << omega
                      << "). Further clamps will not be logged." << std::endl;
        }

        last_command_time_ = nowSeconds();
        moving_ = (vx != 0.0f || vy != 0.0f || omega != 0.0f);

        return client_->SetVelocity(vx, vy, omega, duration);
    }

    int32_t doFsm(const G1CmdRequest& request)
    {
        const auto it = kFsmNames.find(request.ivalue);
        const bool known = (it != kFsmNames.end());

        if (!known && !opt_.allow_any_fsm)
        {
            std::cerr << "[g1_loco_server] refusing unknown FSM id "
                      << request.ivalue
                      << ". Restart with --allow-any-fsm to send it anyway."
                      << std::endl;
            return kErrBadCommand;
        }

        const std::string name = known ? it->second : "UNKNOWN";

        // Logged before and after, because the call blocks until the robot
        // finishes the motion -- a stand-up takes seconds. Without the first
        // line the server looks hung; without the second you cannot tell a slow
        // success from a command still in flight.
        std::cout << "[g1_loco_server] FSM -> " << name << " ("
                  << request.ivalue << ") ... this blocks until the robot "
                     "finishes the motion" << std::endl;

        // Any FSM change ends whatever walk was in progress, so the watchdog
        // has nothing left to stop.
        moving_ = false;

        const int32_t ret = client_->SetFsmId(request.ivalue);

        std::cout << "[g1_loco_server] FSM -> " << name
                  << (ret == 0 ? ": accepted" : ": FAILED, ret=");
        if (ret != 0) std::cout << ret;
        std::cout << "  (accepted != changed -- confirm with loco_cli status)"
                  << std::endl;

        return ret;
    }

    /** Read all three state fields, independently.
     *
     *  Bailing on the first failure hid useful information: on this robot
     *  (2026-08-10) every setter works while GetFsmId returns 7301, and
     *  stopping there meant we never learned whether the other two getters
     *  behave the same way. Each field is now attempted regardless, left at -1
     *  when unavailable, and the call only fails outright if all three do. */
    int32_t doStatus(G1CmdReply& reply)
    {
        int fsm_id = -1;
        int fsm_mode = -1;
        int balance_mode = -1;

        const int32_t ret_id = client_->GetFsmId(fsm_id);
        const int32_t ret_mode = client_->GetFsmMode(fsm_mode);
        const int32_t ret_balance = client_->GetBalanceMode(balance_mode);

        if (ret_id != 0) fsm_id = -1;
        if (ret_mode != 0) fsm_mode = -1;
        if (ret_balance != 0) balance_mode = -1;

        // Only when something changed. loco_cli polls twice a second while
        // waiting for a transition, and a line per poll buries the FSM
        // messages that matter.
        const std::string summary =
            "fsm_id " + (ret_id == 0 ? std::to_string(fsm_id)
                                     : "ret=" + std::to_string(ret_id)) +
            ", fsm_mode " + (ret_mode == 0 ? std::to_string(fsm_mode)
                                           : "ret=" + std::to_string(ret_mode)) +
            ", balance_mode " +
            (ret_balance == 0 ? std::to_string(balance_mode)
                              : "ret=" + std::to_string(ret_balance));

        if (summary != last_status_summary_)
        {
            last_status_summary_ = summary;
            std::cout << "[g1_loco_server] status: " << summary << std::endl;
        }

        reply.fsm_id = fsm_id;
        reply.fsm_mode = fsm_mode;
        reply.balance_mode = balance_mode;

        // Any one field is enough to call it a success; the client shows the
        // rest as unavailable.
        if (ret_id == 0 || ret_mode == 0 || ret_balance == 0) return 0;

        return ret_id;
    }

    /** Bring the robot to an actual standstill.
     *
     *  [WHY THIS EXISTS] Every other stop in this project is
     *  SetVelocity(0,0,0), which zeroes the *velocity*. In balance mode 1
     *  (continuous gait) the robot keeps marching in place at zero velocity,
     *  so none of it stops the robot -- not the per-command duration, not the
     *  watchdog, not the stop-on-exit. Observed 2026-08-11: the robot marched
     *  until the mode was changed by hand, and killing this server made no
     *  difference.
     *
     *  So a halt is stop-the-velocity AND leave continuous gait. This is the
     *  one that means "stop", and it is what the operator-facing paths use. */
    /** Republish the robot's state estimate onto the PUB socket.
     *
     *  Rate-limited: the topic arrives at ~500 Hz, and the consumer is a
     *  Python ROS node that only needs enough to close a walking loop. */
    void onOdom(const void* message)
    {
        const auto* state =
            static_cast<const unitree_go::msg::dds_::SportModeState_*>(message);

        const uint64_t stamp = now_ns();
        std::lock_guard<std::mutex> lock(odom_mutex_);

        odom_.x = state->position()[0];
        odom_.y = state->position()[1];
        odom_.z = state->position()[2];

        const auto& quat = state->imu_state().quaternion();
        odom_.qw = quat[0];
        odom_.qx = quat[1];
        odom_.qy = quat[2];
        odom_.qz = quat[3];

        odom_.vx = state->velocity()[0];
        odom_.vy = state->velocity()[1];
        odom_.vz = state->velocity()[2];
        odom_.yaw_speed = state->yaw_speed();
        odom_.seq++;

        const uint64_t interval_ns =
            static_cast<uint64_t>(1e9 / kOdomPublishHz);
        if (stamp - last_odom_publish_ns_ < interval_ns) return;
        last_odom_publish_ns_ = stamp;
        odom_.stamp_ns = stamp;

        if (odom_pub_ == nullptr) return;
        odom_pub_->send(zmq::const_buffer(&odom_, sizeof(odom_)),
                        zmq::send_flags::dontwait);
        odom_sent_++;
    }

    int32_t doHalt(G1CmdReply& reply)
    {
        moving_ = false;
        reply.clamped_duration = 0.0f;

        const int32_t stop_ret = client_->StopMove();
        // Balance mode 0 = balance in place without stepping.
        const int32_t mode_ret = client_->SetBalanceMode(0);

        std::cout << "[g1_loco_server] HALT: StopMove "
                  << (stop_ret == 0 ? "ok" : "ret=" + std::to_string(stop_ret))
                  << ", balance_mode 0 "
                  << (mode_ret == 0 ? "ok" : "ret=" + std::to_string(mode_ret))
                  << std::endl;

        if (stop_ret != 0) return stop_ret;
        return mode_ret;
    }

    /** Periodic one-line tally of the DDS chatter we swallowed.
     *
     *  Suppressing without ever saying so would turn a loud, obvious fault into
     *  a silent one. A count every few seconds keeps the signal without the
     *  flood. */
    void reportSuppressed()
    {
        const uint64_t total = g_suppressed.load();
        if (total == last_suppressed_) return;

        const double now = nowSeconds();
        if (now - last_suppress_report_ < 5.0) return;
        last_suppress_report_ = now;

        std::cerr << "[g1_loco_server] " << (total - last_suppressed_)
                  << " more DDS transmit errors suppressed (" << total
                  << " total). The robot is probably unreachable."
                  << std::endl;
        last_suppressed_ = total;
    }

    /** Stop the robot if the client stopped talking mid-walk.
     *
     *  The robot-side duration already covers a client that dies outright.
     *  This covers the slower failure: a client still connected but no longer
     *  sending, e.g. a path follower wedged on a blocking call. */
    void checkWatchdog()
    {
        if (!moving_) return;
        if (nowSeconds() - last_command_time_ < opt_.watchdog) return;

        std::cerr << "[g1_loco_server] watchdog: no velocity command for "
                  << opt_.watchdog << " s -- halting" << std::endl;
        moving_ = false;
        // A full halt, not just StopMove: if the robot is in continuous gait
        // it would otherwise keep marching after its commander disappeared.
        G1CmdReply scratch{};
        doHalt(scratch);
    }

    Options opt_;
    std::unique_ptr<unitree::robot::g1::LocoClient> client_;
    std::unique_ptr<zmq::context_t> ctx_;
    std::unique_ptr<zmq::socket_t> rep_;
    std::unique_ptr<zmq::socket_t> odom_pub_;
    std::shared_ptr<unitree::robot::ChannelSubscriber<
        unitree_go::msg::dds_::SportModeState_>> odom_sub_;
    std::mutex odom_mutex_;
    G1OdomPacket odom_{};
    uint64_t last_odom_publish_ns_ = 0;
    std::atomic<uint64_t> odom_sent_{0};

    double last_command_time_ = 0.0;
    bool moving_ = false;
    bool clamp_warned_ = false;
    std::string last_status_summary_;
    uint64_t last_suppressed_ = 0;
    double last_suppress_report_ = 0.0;
};

int main(int argc, char** argv)
{
    Options opt;
    if (!parseArgs(argc, argv, opt)) return 1;

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    std::cout << "G1 loco server (WRITE ACCESS -- this process can make the "
                 "robot walk)"
              << std::endl;

    G1LocoServer server(opt);
    server.run();

    return 0;
}
