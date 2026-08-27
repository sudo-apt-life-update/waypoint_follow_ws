/**
 * g1_odom_probe -- find out whether the G1 will tell us where it is.
 *
 * Read-only. Sends no motion command and changes no state.
 *
 * WHY THIS EXISTS
 *
 * This project drives the robot open loop, and eight measured runs showed the
 * robot covering about half the commanded distance with a fixed backwards
 * settle at the end. The fix is to measure displacement instead of assuming
 * it -- but before writing leg odometry (forward kinematics with no foot
 * contact sensor, because the G1's hg LowState_ has none), it is worth finding
 * out whether the robot already publishes a pose. Two leads, neither
 * documented for the G1:
 *
 * WHAT DISCOVERY FOUND (2026-08-13)
 *
 * Guessing topic names out of SDK examples produced two dead ends and one
 * FALSE NEGATIVE, which is the lesson worth keeping:
 *
 *   - Loco RPC ids 7201-7203, guessed from H1's ENABLE_ODOM/GET_ODOM
 *     (8201-8203): the service answered 3203, "Api not implemented". A real
 *     no -- note 3203 (service replied, no such api) versus 3104 (nobody
 *     answered), which is how you tell a negative result from a dead link.
 *
 *   - rt/sportmodestate: the first version of this probe subscribed with
 *     unitree_go::msg::dds_::SportModeState_ and received nothing, and
 *     reported that as "the topic does not exist". Wrong. The topic exists and
 *     is published as unitree_hg::msg::dds_::SportModeState_ -- a DIFFERENT
 *     type with the same short name. DDS matches subscribers on type name, so
 *     a wrong-typed reader is silently never eligible and looks exactly like
 *     an absent topic. The SDK ships no hg SportModeState_ header, so that
 *     topic is not reachable from this codebase without hand-writing the IDL.
 *
 * Enumerating DDS discovery instead of guessing found 103 topics, including
 * three genuine pose sources and a whole perception stack running on the
 * robot:
 *
 *   rt/odommodestate   unitree_go::msg::dds_::SportModeState_  <- type we have
 *   rt/dog_odom        nav_msgs::msg::dds_::Odometry_          <- type we have
 *   rt/sportmodestate  unitree_hg::msg::dds_::SportModeState_  (no header)
 *   rt/utlidar/cloud_livox_mid360, rt/global_map, rt/collision_clouds, ...
 *
 * IS THAT ODOMETRY LiDAR-DERIVED? Almost certainly not, which matters because
 * this project is deliberately LiDAR-free. Measured rates: ~500 Hz on
 * rt/odommodestate and ~1000 Hz on rt/dog_odom. A MID360 spins at 10 Hz and
 * LiDAR SLAM publishes at sensor rate; nothing LiDAR-based emits pose at
 * 500-1000 Hz. Both topics also reported identical x/y, so they are one
 * source in two formats, and its position.z tracks pelvis height (0.708 m
 * standing) -- a leg-kinematics quantity. This is the locomotion controller's
 * own state estimator: leg odometry fused with the IMU, computed on the robot.
 * Exactly what we were about to write ourselves, already running.
 *
 * This probe now samples the two reachable ones and reports whether their
 * pose fields actually move.
 *
 * A separate binary rather than a change to g1_loco_server: that server is
 * working and is the only thing that can move the robot. Probing undocumented
 * api ids does not belong in it.
 *
 * Run:  ros2 run g1_loco_server g1_odom_probe --iface=eno1 [--seconds=15]
 *       ros2 run g1_loco_server g1_odom_probe --topics-only   (list DDS topics)
 *
 * Walk the robot during the probe (another terminal:
 * `ros2 run g1_walk loco_cli move --vx 0.4 --seconds 7.5`) -- fields that are
 * present but always zero look identical to fields that are not populated
 * until the robot actually moves.
 */

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/client/client.hpp>

#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/idl/ros2/Odometry_.hpp>
#include <unitree/idl/ros2/PointCloud2_.hpp>

// The C API, for DDS discovery. Reading the builtin DCPSPublication topic
// lists every topic anything on this network is publishing -- which beats
// guessing names out of SDK examples, and is the only way to be sure we are
// not missing a pose source under a name nobody documented.
#include <dds/dds.h>

#include <algorithm>
#include <set>
#include <vector>

using namespace unitree::robot;

// Found by DDS discovery, not by reading headers. Both use types the SDK
// ships, so both are subscribable from here.
static const std::string kOdomModeStateTopic = "rt/odommodestate";
static const std::string kDogOdomTopic = "rt/dog_odom";

/** Enumerate every topic being published on the robot's DDS network.
 *
 *  Uses CycloneDDS's builtin DCPSPublication topic, which every participant
 *  populates through discovery. This is the authoritative answer to "what does
 *  this robot actually offer?" -- the SDK's headers only document what Unitree
 *  chose to ship examples for, and the G1 turns out not to publish the one
 *  topic those examples pointed at.
 *
 *  A second participant on the same domain is fine: discovery data is shared,
 *  and this one creates no readers on user topics. */
static void listPublishedTopics(double settle_seconds)
{
    std::cout << "\n--- every topic published on this DDS network ---\n";

    // Domain 0, inheriting the config ChannelFactory::Init already installed
    // (which is what pins us to the right network interface).
    const dds_entity_t participant = dds_create_participant(0, nullptr, nullptr);
    if (participant < 0)
    {
        std::cout << "  could not create a discovery participant: "
                  << dds_strretcode(-participant) << "\n";
        return;
    }

    const dds_entity_t reader =
        dds_create_reader(participant, DDS_BUILTIN_TOPIC_DCPSPUBLICATION,
                          nullptr, nullptr);
    if (reader < 0)
    {
        std::cout << "  could not read DCPSPublication: "
                  << dds_strretcode(-reader) << "\n";
        dds_delete(participant);
        return;
    }

    // Discovery is asynchronous; give remote announcements time to arrive.
    std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<int64_t>(settle_seconds * 1000)));

    std::set<std::string> topics;
    for (;;)
    {
        void* samples[32] = {nullptr};
        dds_sample_info_t infos[32];
        const int32_t n = dds_take(reader, samples, infos, 32, 32);
        if (n <= 0) break;

        for (int32_t i = 0; i < n; ++i)
        {
            if (!infos[i].valid_data) continue;
            const auto* endpoint =
                static_cast<dds_builtintopic_endpoint_t*>(samples[i]);
            if (endpoint != nullptr && endpoint->topic_name != nullptr)
            {
                std::string name = endpoint->topic_name;
                std::string type =
                    endpoint->type_name ? endpoint->type_name : "?";
                topics.insert(name + "   [" + type + "]");
            }
        }
        dds_return_loan(reader, samples, n);
    }

    if (topics.empty())
    {
        std::cout << "  none discovered. Either nothing is publishing, or the\n"
                     "  interface is wrong -- check `ros2 run g1_walk "
                     "preflight`.\n";
    }
    else
    {
        for (const auto& topic : topics) std::cout << "  " << topic << "\n";
        std::cout << "\n  " << topics.size() << " topics. Anything here "
                     "carrying a pose, an odometry\n  or a foot force is worth "
                     "more than the leg odometry we would\n  otherwise have to "
                     "write.\n";
    }

    dds_delete(reader);
    dds_delete(participant);
}

/** Watches rt/dog_odom, a standard nav_msgs/Odometry the robot publishes.
 *
 *  If this one is populated it is the best of the lot: a full pose with
 *  orientation and twist, in a type ROS already understands, needing no
 *  conversion beyond a frame rename. */
class DogOdomWatcher
{
public:
    void start()
    {
        subscriber_ = std::make_shared<
            ChannelSubscriber<nav_msgs::msg::dds_::Odometry_>>(kDogOdomTopic);
        subscriber_->InitChannel(
            std::bind(&DogOdomWatcher::onMessage, this, std::placeholders::_1),
            10);
    }

    void onMessage(const void* message)
    {
        const auto* odom =
            static_cast<const nav_msgs::msg::dds_::Odometry_*>(message);

        std::lock_guard<std::mutex> lock(mutex_);
        count_++;
        child_frame_ = odom->child_frame_id();
        x_ = odom->pose().pose().position().x();
        y_ = odom->pose().pose().position().y();
        vx_ = odom->twist().twist().linear().x();
        if (!seen_)
        {
            seen_ = true;
            min_x_ = max_x_ = x_;
            min_y_ = max_y_ = y_;
        }
        min_x_ = std::min(min_x_, x_); max_x_ = std::max(max_x_, x_);
        min_y_ = std::min(min_y_, y_); max_y_ = std::max(max_y_, y_);
    }

    void report()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        std::cout << "\n--- " << kDogOdomTopic << "  (nav_msgs/Odometry) ---\n";
        if (count_ == 0)
        {
            std::cout << "  nothing received.\n";
            return;
        }
        const double span = std::max(max_x_ - min_x_, max_y_ - min_y_);
        std::cout << std::fixed << std::setprecision(4);
        std::cout << "  " << count_ << " messages, child_frame_id=\""
                  << child_frame_ << "\"\n"
                  << "  pose  x=" << x_ << "  y=" << y_
                  << "   twist.linear.x=" << vx_ << "\n"
                  << "  position span over the window: " << span << " m"
                  << (span > 0.01 ? "   <-- CHANGES" : "") << "\n";
        if (span > 0.01)
        {
            std::cout << "\n  VERDICT: a live nav_msgs/Odometry straight from "
                         "the robot.\n  This is the one to use.\n";
        }
        else if (std::fabs(x_) > 1e-6 || std::fabs(y_) > 1e-6)
        {
            std::cout << "\n  VERDICT: populated, but the robot did not move. "
                         "Inconclusive --\n  rerun while walking.\n";
        }
    }

private:
    std::mutex mutex_;
    uint64_t count_ = 0;
    bool seen_ = false;
    std::string child_frame_;
    double x_ = 0, y_ = 0, vx_ = 0;
    double min_x_ = 0, max_x_ = 0, min_y_ = 0, max_y_ = 0;
    std::shared_ptr<ChannelSubscriber<nav_msgs::msg::dds_::Odometry_>>
        subscriber_;
};

/** Samples the robot's point-cloud topics and reports each one's frame_id.
 *
 *  Answers "are the obstacle clouds from the LiDAR or the camera?" by asking
 *  the data instead of inferring from topic names. The frame_id names the
 *  sensor the cloud is expressed in, and the width/height say whether it is
 *  an organised (camera-like) or unorganised (LiDAR-like) cloud -- a depth
 *  camera produces height>1, a spinning/solid-state LiDAR produces height==1.
 */
class CloudWatcher
{
public:
    explicit CloudWatcher(const std::string& topic) : topic_(topic) {}

    void start()
    {
        subscriber_ = std::make_shared<
            ChannelSubscriber<sensor_msgs::msg::dds_::PointCloud2_>>(topic_);
        subscriber_->InitChannel(
            std::bind(&CloudWatcher::onMessage, this, std::placeholders::_1),
            2);
    }

    void onMessage(const void* message)
    {
        const auto* cloud =
            static_cast<const sensor_msgs::msg::dds_::PointCloud2_*>(message);
        std::lock_guard<std::mutex> lock(mutex_);
        count_++;
        frame_ = cloud->header().frame_id();
        width_ = cloud->width();
        height_ = cloud->height();
    }

    void report() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        std::cout << "  " << std::left << std::setw(26) << topic_
                  << std::right;
        if (count_ == 0)
        {
            std::cout << "  (silent)\n";
            return;
        }
        std::cout << "  frame_id=" << std::left << std::setw(20) << frame_
                  << std::right << " " << width_ << "x" << height_
                  << (height_ > 1 ? "  organised (camera-like)"
                                  : "  unorganised (lidar-like)")
                  << "\n";
    }

private:
    mutable std::mutex mutex_;
    std::string topic_;
    uint64_t count_ = 0;
    std::string frame_;
    uint32_t width_ = 0;
    uint32_t height_ = 0;
    std::shared_ptr<ChannelSubscriber<sensor_msgs::msg::dds_::PointCloud2_>>
        subscriber_;
};

// By analogy with H1's 8201/8202/8203. Undocumented for the G1.
static constexpr int32_t kApiEnableOdom = 7201;
static constexpr int32_t kApiDisableOdom = 7202;
static constexpr int32_t kApiGetOdom = 7203;

// The G1 loco service, same name H1 uses.
static const std::string kLocoServiceName = "sport";
static const std::string kLocoApiVersion = "1.0.0.0";

static std::atomic<bool> g_running{true};

static void onSignal(int) { g_running.store(false); }

/** Watches rt/sportmodestate and records whether anything in it ever moves.
 *
 *  "Non-zero" is not the question -- a field could be populated with a
 *  constant. The question is whether it CHANGES while the robot walks, so the
 *  span (max - min) of each field is tracked. */
class SportStateWatcher
{
public:
    void start()
    {
        subscriber_ = std::make_shared<ChannelSubscriber<
            unitree_go::msg::dds_::SportModeState_>>(kOdomModeStateTopic);
        subscriber_->InitChannel(
            std::bind(&SportStateWatcher::onMessage, this,
                      std::placeholders::_1),
            10);
    }

    void onMessage(const void* message)
    {
        const auto* state =
            static_cast<const unitree_go::msg::dds_::SportModeState_*>(message);

        std::lock_guard<std::mutex> lock(mutex_);
        count_++;
        mode_ = state->mode();

        for (int i = 0; i < 3; ++i)
        {
            track(position_[i], state->position()[i]);
            track(velocity_[i], state->velocity()[i]);
        }
        track(yaw_speed_, state->yaw_speed());
        for (int i = 0; i < 4; ++i)
        {
            track(foot_force_[i], static_cast<float>(state->foot_force()[i]));
        }
    }

    void report()
    {
        std::lock_guard<std::mutex> lock(mutex_);

        std::cout << "\n--- " << kOdomModeStateTopic
                  << "  (SportModeState_) ---\n";
        if (count_ == 0)
        {
            std::cout
                << "  nothing received. If discovery lists the topic, the\n"
                   "  publisher's TYPE differs from ours -- DDS will not "
                   "deliver\n  across a type-name mismatch, and it looks "
                   "identical to an\n  absent topic. Check --topics-only.\n";
            return;
        }

        std::cout << "  " << count_ << " messages received, mode="
                  << static_cast<int>(mode_) << "\n";
        std::cout << std::fixed << std::setprecision(4);

        show("position.x", position_[0]);
        show("position.y", position_[1]);
        show("position.z", position_[2]);
        show("velocity.x", velocity_[0]);
        show("velocity.y", velocity_[1]);
        show("velocity.z", velocity_[2]);
        show("yaw_speed  ", yaw_speed_);
        show("foot_force0", foot_force_[0]);
        show("foot_force1", foot_force_[1]);
        show("foot_force2", foot_force_[2]);
        show("foot_force3", foot_force_[3]);

        const bool pose_moved = position_[0].span() > 0.01f ||
                                position_[1].span() > 0.01f;
        const bool vel_moved = velocity_[0].span() > 0.01f ||
                               velocity_[1].span() > 0.01f;

        // "Did it change" and "is it populated" are different questions, and
        // conflating them produced a wrong verdict once already: a stationary
        // robot's real odometry looks exactly like an unpopulated field if you
        // only watch the span. Populated-ness is better judged from whether
        // the values are physically plausible -- an unfilled field is exactly
        // zero, whereas a standing G1 reports a pelvis height near 0.7 m.
        const bool populated = std::fabs(position_[2].last) > 0.05f ||
                               std::fabs(position_[0].last) > 1e-6f ||
                               std::fabs(position_[1].last) > 1e-6f;

        std::cout << "\n  VERDICT: ";
        if (pose_moved)
        {
            std::cout << "position CHANGES -- live, usable odometry.\n"
                         "  Use it. No leg odometry needed.\n";
        }
        else if (vel_moved)
        {
            std::cout << "position is static but velocity CHANGES.\n"
                         "  Integrating a measured body velocity still beats "
                         "dead reckoning\n  the commanded one.\n";
        }
        else if (populated)
        {
            std::cout << "POPULATED BUT THE ROBOT DID NOT MOVE.\n"
                         "  position.z=" << position_[2].last
                      << " m is a plausible pelvis height, and an unfilled\n"
                         "  field would read exactly 0. This is real odometry "
                         "-- the probe just\n  had nothing to measure.\n"
                         "\n  INCONCLUSIVE. Rerun with the robot walking:\n"
                         "    terminal 1: ros2 run g1_loco_server "
                         "g1_loco_server --iface=eno1\n"
                         "    terminal 2: ros2 run g1_walk loco_cli "
                         "main_operation\n"
                         "                ros2 run g1_walk loco_cli move "
                         "--vx 0.4 --seconds 7.5\n"
                         "    terminal 3: this probe\n";
        }
        else
        {
            std::cout << "every field is exactly zero -- not populated by this "
                         "firmware.\n";
        }
    }

private:
    struct Range
    {
        bool seen = false;
        float min = 0.0f;
        float max = 0.0f;
        float last = 0.0f;
        float span() const { return seen ? (max - min) : 0.0f; }
    };

    static void track(Range& range, float value)
    {
        if (!std::isfinite(value)) return;
        if (!range.seen)
        {
            range.seen = true;
            range.min = range.max = value;
        }
        range.min = std::min(range.min, value);
        range.max = std::max(range.max, value);
        range.last = value;
    }

    static void show(const std::string& name, const Range& range)
    {
        std::cout << "  " << name << "  last " << std::setw(10) << range.last
                  << "   range [" << std::setw(10) << range.min << ", "
                  << std::setw(10) << range.max << "]   span "
                  << std::setw(9) << range.span()
                  << (range.span() > 0.01f ? "   <-- CHANGES" : "")
                  << "\n";
    }

    std::mutex mutex_;
    uint64_t count_ = 0;
    uint8_t mode_ = 0;
    Range position_[3];
    Range velocity_[3];
    Range yaw_speed_;
    Range foot_force_[4];
    std::shared_ptr<
        ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>> subscriber_;
};

/** Calls loco api ids the SDK's G1 client does not declare.
 *
 *  Client::Call and RegistApi are protected, so reaching the undocumented ids
 *  means subclassing. Nothing here commands motion: enable/get odom only. */
class OdomRpcProbe : public Client
{
public:
    OdomRpcProbe() : Client(kLocoServiceName, false) {}

    void Init()
    {
        SetApiVersion(kLocoApiVersion);
        UT_ROBOT_CLIENT_REG_API_NO_PROI(kApiEnableOdom);
        UT_ROBOT_CLIENT_REG_API_NO_PROI(kApiDisableOdom);
        UT_ROBOT_CLIENT_REG_API_NO_PROI(kApiGetOdom);
        SetTimeout(5.0f);
    }

    void probe()
    {
        std::cout << "\n--- loco RPC odometry ids (undocumented on G1) ---\n";

        std::string data;
        int32_t ret = Call(kApiEnableOdom, "", data);
        std::cout << "  7201 ENABLE_ODOM  ret=" << ret;
        if (!data.empty()) std::cout << "  data=" << data;
        std::cout << (ret == 0 ? "   <-- ACCEPTED" : "") << "\n";

        // Give the controller a moment to start publishing if it just enabled.
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        data.clear();
        ret = Call(kApiGetOdom, "", data);
        std::cout << "  7203 GET_ODOM     ret=" << ret;
        if (!data.empty()) std::cout << "  data=" << data;
        std::cout << (ret == 0 ? "   <-- ACCEPTED" : "") << "\n";

        if (ret == 0 && !data.empty())
        {
            std::cout << "\n  VERDICT: GET_ODOM answered. The payload above is "
                         "the pose\n"
                         "  (H1 decodes the same reply as JsonizeVec3 -> x, y, "
                         "yaw).\n"
                         "  Wire this into g1_loco_server as CMD_ODOM.\n";
        }
        else
        {
            std::cout << "\n  VERDICT: no odometry RPC on this firmware. "
                         "Expected -- these\n"
                         "  ids are an educated guess from H1. Fall back to "
                         "leg odometry.\n";
        }
    }
};

int main(int argc, char** argv)
{
    std::string iface = "eno1";
    double seconds = 15.0;
    bool topics_only = false;
    bool show_clouds = false;

    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        if (arg == "--help" || arg == "-h")
        {
            std::cout << "g1_odom_probe -- read-only check for G1 odometry\n\n"
                         "  --iface=eno1     network interface to the robot\n"
                         "  --seconds=15     how long to watch "
                         "rt/sportmodestate\n"
                         "  --topics-only    skip the RPC probe, just list "
                         "DDS topics\n"
                         "  --clouds         also sample the LiDAR-derived "
                         "cloud topics\n\n"
                         "Walk the robot during the probe, or fields that are\n"
                         "populated only while moving will look absent.\n";
            return 0;
        }
        if (arg == "--topics-only")
        {
            topics_only = true;
            continue;
        }
        if (arg == "--clouds")
        {
            show_clouds = true;
            continue;
        }
        const size_t eq = arg.find('=');
        if (eq == std::string::npos) continue;
        const std::string key = arg.substr(2, eq - 2);
        const std::string value = arg.substr(eq + 1);
        if (key == "iface") iface = value;
        else if (key == "seconds") seconds = std::stod(value);
    }

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    std::cout << "G1 odometry probe (READ-ONLY -- sends no motion command)\n"
              << "Initializing Unitree DDS on interface: " << iface
              << std::endl;

    ChannelFactory::Instance()->Init(0, iface);

    // After Init, never before: the client reaches into the ChannelFactory
    // singleton while constructing. Same constraint as g1_loco_server.
    SportStateWatcher watcher;
    watcher.start();

    DogOdomWatcher dog_odom;
    dog_odom.start();

    // Off by default. This project is deliberately LiDAR-free, and the
    // robot's cloud topics are all LiDAR-derived, so sampling them is a
    // curiosity rather than part of the job. --clouds asks anyway.
    std::vector<std::unique_ptr<CloudWatcher>> clouds;
    if (show_clouds)
    {
        for (const char* topic : {"rt/utlidar/cloud_livox_mid360",
                                  "rt/collision_clouds",
                                  "rt/pre_collision_clouds",
                                  "rt/safe_clouds", "rt/warning_clouds",
                                  "rt/grid_clouds", "rt/ele_clouds"})
        {
            clouds.emplace_back(std::make_unique<CloudWatcher>(topic));
            clouds.back()->start();
        }
    }

    if (topics_only) seconds = 0.5;

    std::cout << "Watching " << kOdomModeStateTopic << " and " << kDogOdomTopic
              << " for " << seconds
              << " s.\nWALK THE ROBOT NOW -- in another terminal:\n"
                 "  ros2 run g1_walk loco_cli move --vx 0.4 --seconds 7.5"
              << std::endl;

    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(
                              static_cast<int64_t>(seconds * 1000));
    while (g_running.load() && std::chrono::steady_clock::now() < deadline)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    watcher.report();
    dog_odom.report();

    if (show_clouds)
    {
        std::cout << "\n--- point clouds: which sensor do they come from? "
                     "---\n";
        for (const auto& cloud : clouds) cloud->report();
        std::cout << "\n  A shared frame_id with cloud_livox_mid360 means the "
                     "obstacle clouds\n  are derived from the LiDAR. A camera "
                     "optical frame would mean the\n  depth camera. Silent "
                     "topics are advertised but not produced.\n";
    }

    listPublishedTopics(3.0);

    if (!topics_only)
    {
        OdomRpcProbe rpc;
        rpc.Init();
        rpc.probe();
    }

    std::cout << "\nDone. Nothing was commanded; the robot's state is "
                 "unchanged." << std::endl;
    return 0;
}
