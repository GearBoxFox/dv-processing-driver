#include <chrono>
#include <cstdint>
#include <cstring>
#include <memory>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include <dv-processing/core/core.hpp>
#include <dv-processing/core/frame.hpp>
#include <opencv4/opencv2/core.hpp>

#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "dv_processing_driver/msg/event_array.hpp"
#include "rclcpp/rclcpp.hpp"

/**
 * Subscribes to an EventArray topic (e.g. replayed from a rosbag), feeds the events into a
 * dv::Accumulator and publishes the reconstructed frames.
 *
 * Frames are generated on *recording* time (event timestamps), not wall time, so the output is
 * the same regardless of `ros2 bag play --rate` and of callback jitter.
 *
 *   ros2 run dv_processing_driver playback_node --ros-args -p topic:=/dvxm/events
 *
 * Everything except output_topic can be changed live with `ros2 param set`, including the input
 * topic. Changing topic / qos_* / width / height starts a fresh accumulator.
 */
namespace dv_playback_node {

using EventArray = dv_processing_driver::msg::EventArray;

namespace {
constexpr const char *kEventArrayType = "dv_processing_driver/msg/EventArray";
// If event time jumps backwards by more than this, assume the bag was restarted/looped.
constexpr int64_t kRewindThresholdUs = 1'000'000;
// If event time jumps forwards by more than this, restart slicing instead of emitting
// thousands of empty frames to fill the gap.
constexpr int64_t kGapThresholdUs = 5'000'000;
// Fallback when neither the parameters nor the messages carry a resolution (DVXplorer Micro).
const cv::Size kFallbackResolution(640, 480);
} // namespace

class PlaybackNode final : public rclcpp::Node {
public:
    PlaybackNode() : Node("playback_node") {
        // Startup-only (use a remap, `-r img_accum:=...`, or this parameter)
        const auto outputTopic = declare_parameter<std::string>("output_topic", "img_accum");

        // Input subscription; changing any of these at runtime re-subscribes
        declare_parameter<std::string>("topic", "/dvxm/events");
        declare_parameter<int>("qos_depth", 100);
        declare_parameter<std::string>("qos_reliability", "reliable");

        // Output / geometry
        declare_parameter<std::string>("frame_id", "camera_link");
        declare_parameter<int>("width", 0);  // 0 = take from messages
        declare_parameter<int>("height", 0); // 0 = take from messages

        // Accumulator (defaults match dv::Accumulator's own defaults)
        declare_parameter<double>("frame_interval_ms", 33.0);
        declare_parameter<std::string>("accumulator_decay_function", "EXPONENTIAL");
        declare_parameter<double>("accumulator_decay_param", 1.0e+6);
        declare_parameter<double>("accumulator_event_contribution", 0.15);
        declare_parameter<double>("accumulator_min_potential", 0.0);
        declare_parameter<double>("accumulator_neutral_potential", 0.0);
        declare_parameter<double>("accumulator_max_potential", 1.0);
        declare_parameter<bool>("accumulator_ignore_polarity", false);
        declare_parameter<bool>("accumulator_synchronous_decay", false);

        mFramePub = create_publisher<sensor_msgs::msg::Image>(outputTopic, 10);

        // Run the declared values through the same path used for runtime updates; this also
        // creates the subscription.
        if (const auto result = applyParameters(get_parameters(kRuntimeParams)); !result.successful) {
            throw std::invalid_argument(result.reason);
        }

        // Registered after all declare_parameter() calls, since those also invoke the callback.
        mParamCallback = add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter> &params) { return onSetParameters(params); });

        // Periodically check that whatever is publishing on the topic uses a type we can read.
        mTypeCheckTimer = create_wall_timer(std::chrono::seconds(2), [this]() { checkTopicType(); });
    }

private:
    struct AccumulatorSettings {
        dv::Accumulator::Decay decayFunction = dv::Accumulator::Decay::EXPONENTIAL;
        double decayParam                    = 1.0e+6;
        float eventContribution              = 0.15f;
        float minPotential                   = 0.0f;
        float neutralPotential               = 0.0f;
        float maxPotential                   = 1.0f;
        bool ignorePolarity                  = false;
        bool synchronousDecay                = false;
    };

    /** Everything that applyParameters() understands. */
    struct State {
        std::string topic;
        int64_t qosDepth = 100;
        std::string qosReliability;
        std::string frameId;
        int64_t width            = 0;
        int64_t height           = 0;
        double frameIntervalMs   = 33.0;
        AccumulatorSettings accumulator;
    };

    inline static const std::vector<std::string> kRuntimeParams = {
        "topic",
        "qos_depth",
        "qos_reliability",
        "frame_id",
        "width",
        "height",
        "frame_interval_ms",
        "accumulator_decay_function",
        "accumulator_decay_param",
        "accumulator_event_contribution",
        "accumulator_min_potential",
        "accumulator_neutral_potential",
        "accumulator_max_potential",
        "accumulator_ignore_polarity",
        "accumulator_synchronous_decay",
    };

    static std::optional<dv::Accumulator::Decay> parseDecay(const std::string &name) {
        if (name == "NONE") return dv::Accumulator::Decay::NONE;
        if (name == "LINEAR") return dv::Accumulator::Decay::LINEAR;
        if (name == "EXPONENTIAL") return dv::Accumulator::Decay::EXPONENTIAL;
        if (name == "STEP") return dv::Accumulator::Decay::STEP;
        return std::nullopt;
    }

    static rcl_interfaces::msg::SetParametersResult reject(const std::string &reason) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = false;
        result.reason     = reason;
        return result;
    }

    rcl_interfaces::msg::SetParametersResult onSetParameters(const std::vector<rclcpp::Parameter> &params) {
        for (const auto &param : params) {
            if (param.get_name() == "output_topic") {
                return reject("'output_topic' can only be set at startup (or remap it)");
            }
        }
        return applyParameters(params);
    }

    /**
     * Validates the whole batch first and only commits it if every value is valid, so a rejected
     * `ros2 param set` leaves the node exactly as it was.
     */
    rcl_interfaces::msg::SetParametersResult applyParameters(const std::vector<rclcpp::Parameter> &params) {
        State next = mState;

        for (const auto &param : params) {
            const auto &name = param.get_name();
            if (name == "topic") next.topic = param.as_string();
            else if (name == "qos_depth") next.qosDepth = param.as_int();
            else if (name == "qos_reliability") next.qosReliability = param.as_string();
            else if (name == "frame_id") next.frameId = param.as_string();
            else if (name == "width") next.width = param.as_int();
            else if (name == "height") next.height = param.as_int();
            else if (name == "frame_interval_ms") next.frameIntervalMs = param.as_double();
            else if (name == "accumulator_decay_function") {
                const auto decay = parseDecay(param.as_string());
                if (!decay.has_value()) {
                    return reject("accumulator_decay_function must be one of NONE, LINEAR, EXPONENTIAL, STEP");
                }
                next.accumulator.decayFunction = *decay;
            }
            else if (name == "accumulator_decay_param") next.accumulator.decayParam = param.as_double();
            else if (name == "accumulator_event_contribution")
                next.accumulator.eventContribution = static_cast<float>(param.as_double());
            else if (name == "accumulator_min_potential")
                next.accumulator.minPotential = static_cast<float>(param.as_double());
            else if (name == "accumulator_neutral_potential")
                next.accumulator.neutralPotential = static_cast<float>(param.as_double());
            else if (name == "accumulator_max_potential")
                next.accumulator.maxPotential = static_cast<float>(param.as_double());
            else if (name == "accumulator_ignore_polarity") next.accumulator.ignorePolarity  = param.as_bool();
            else if (name == "accumulator_synchronous_decay") next.accumulator.synchronousDecay = param.as_bool();
        }

        // ---- validate ----
        if (next.topic.empty()) {
            return reject("topic must not be empty");
        }
        if (next.qosDepth < 1) {
            return reject("qos_depth must be greater than zero");
        }
        if (next.qosReliability != "reliable" && next.qosReliability != "best_effort") {
            return reject("qos_reliability must be 'reliable' or 'best_effort'");
        }
        if ((next.width > 0) != (next.height > 0) || next.width < 0 || next.height < 0) {
            return reject("width and height must both be > 0, or both be 0 (auto)");
        }
        if (next.frameIntervalMs <= 0.0) {
            return reject("frame_interval_ms must be greater than zero");
        }
        if (next.accumulator.decayParam < 0.0) {
            return reject("accumulator_decay_param must be non-negative");
        }
        const auto &acc = next.accumulator;
        if (!(acc.minPotential <= acc.neutralPotential && acc.neutralPotential <= acc.maxPotential)) {
            return reject("potentials must satisfy min <= neutral <= max");
        }

        const bool resubscribe = !mSubscription || next.topic != mState.topic || next.qosDepth != mState.qosDepth
                              || next.qosReliability != mState.qosReliability;
        const bool resolutionChanged = next.width != mState.width || next.height != mState.height;
        const bool intervalChanged   = next.frameIntervalMs != mState.frameIntervalMs;

        // Create the new subscription before committing anything, so an invalid topic name
        // is rejected cleanly and the old subscription keeps working.
        rclcpp::Subscription<EventArray>::SharedPtr newSubscription;
        if (resubscribe) {
            try {
                newSubscription = create_subscription<EventArray>(next.topic, makeQos(next),
                    [this](const EventArray::ConstSharedPtr message) { onEvents(*message); });
            }
            catch (const std::exception &e) {
                return reject("could not subscribe to '" + next.topic + "': " + e.what());
            }
        }

        // ---- commit ----
        mState = next;

        if (resubscribe) {
            mSubscription = newSubscription;
            mReceivedOnTopic = false;
            mWarnedTypes.clear();
            RCLCPP_INFO_STREAM(get_logger(), "Listening for " << kEventArrayType << " on ["
                                                 << mSubscription->get_topic_name() << "] (" << mState.qosReliability
                                                 << ", depth " << mState.qosDepth << "), publishing frames on ["
                                                 << mFramePub->get_topic_name() << "]");
        }

        if (resubscribe || resolutionChanged) {
            // New recording or new geometry: start from a blank accumulator. With width/height
            // = 0 the resolution is re-detected from the first message on the (new) topic.
            mAccumulator.reset();
            mSlicer.reset();
            mLastEventTime.reset();
            mLastFrameTime.reset();
            if (mState.width > 0 && mState.height > 0) {
                mResolution = cv::Size(static_cast<int>(mState.width), static_cast<int>(mState.height));
                resetPipeline();
            }
        }
        else if (mAccumulator) {
            configureAccumulator(*mAccumulator);
            if (intervalChanged) {
                resetSlicer();
            }
        }

        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
    }

    static rclcpp::QoS makeQos(const State &state) {
        rclcpp::QoS qos{rclcpp::KeepLast(static_cast<std::size_t>(state.qosDepth))};
        if (state.qosReliability == "best_effort") {
            qos.best_effort();
        }
        else {
            qos.reliable();
        }
        return qos;
    }

    /**
     * ROS 2 silently refuses to connect a publisher and subscriber whose types differ, so a bag
     * recorded with another driver's message type would just look like "no data". Warn instead.
     */
    void checkTopicType() {
        if (!mSubscription) {
            return;
        }
        const std::string topic = mSubscription->get_topic_name();
        const auto topics       = get_topic_names_and_types();
        const auto it           = topics.find(topic);
        if (it == topics.end()) {
            return;
        }
        for (const auto &type : it->second) {
            if (type != kEventArrayType && mWarnedTypes.insert(type).second) {
                RCLCPP_WARN(get_logger(),
                    "[%s] is being published as [%s], but this node can only read [%s]; those messages will "
                    "not be received.",
                    topic.c_str(), type.c_str(), kEventArrayType);
            }
        }
    }

    void configureAccumulator(dv::Accumulator &accumulator) const {
        const auto &s = mState.accumulator;
        accumulator.setDecayFunction(s.decayFunction);
        accumulator.setDecayParam(s.decayParam);
        accumulator.setEventContribution(s.eventContribution);
        accumulator.setMinPotential(s.minPotential);
        accumulator.setNeutralPotential(s.neutralPotential);
        accumulator.setMaxPotential(s.maxPotential);
        accumulator.setIgnorePolarity(s.ignorePolarity);
        accumulator.setSynchronousDecay(s.synchronousDecay);
    }

    /** Fresh accumulator (cleared potential surface) plus a fresh slicer. */
    void resetPipeline() {
        mAccumulator = std::make_unique<dv::Accumulator>(mResolution);
        configureAccumulator(*mAccumulator);
        resetSlicer();
    }

    void resetSlicer() {
        const auto interval = std::chrono::microseconds(static_cast<int64_t>(mState.frameIntervalMs * 1000.0));
        mFrameIntervalUs    = interval.count();
        mSlicer             = std::make_unique<dv::EventStreamSlicer>();
        mSlicer->doEveryTimeInterval(interval, [this](const dv::EventStore &events) { onSlice(events); });
        mLastEventTime.reset();
        mLastFrameTime.reset();
    }

    cv::Size resolveResolution(const EventArray &message) {
        if (message.width > 0 && message.height > 0) {
            return {static_cast<int>(message.width), static_cast<int>(message.height)};
        }
        RCLCPP_WARN_STREAM(get_logger(), "EventArray messages carry no resolution (width/height = 0) and no width/height "
                                         "parameters were given; assuming "
                                             << kFallbackResolution.width << "x" << kFallbackResolution.height);
        return kFallbackResolution;
    }

    void onEvents(const EventArray &message) {
        if (!mReceivedOnTopic) {
            mReceivedOnTopic = true;
            RCLCPP_INFO_STREAM(get_logger(), "Receiving events on [" << mSubscription->get_topic_name() << "]");
        }
        if (message.events.empty()) {
            return;
        }

        if (!mAccumulator) {
            mResolution = resolveResolution(message);
            RCLCPP_INFO_STREAM(get_logger(), "Accumulating at " << mResolution.width << "x" << mResolution.height);
            resetPipeline();
        }

        std::size_t outOfBounds = 0;
        dv::EventStore store    = toEventStore(message, mResolution, outOfBounds);
        if (outOfBounds > 0) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "Dropped %zu events outside the %dx%d accumulator resolution (set width/height?)", outOfBounds,
                mResolution.width, mResolution.height);
        }
        if (store.isEmpty()) {
            return;
        }

        if (mLastEventTime.has_value()) {
            const int64_t lowest = store.getLowestTime();
            if (lowest < *mLastEventTime) {
                if (*mLastEventTime - lowest > kRewindThresholdUs) {
                    RCLCPP_WARN(get_logger(), "Event time jumped backwards by %.3f s (bag restarted or looped?), "
                                              "resetting accumulator",
                        static_cast<double>(*mLastEventTime - lowest) * 1e-6);
                    resetPipeline();
                }
                else {
                    // Overlapping/duplicate events; keep only the new part so time stays monotonic.
                    store = store.sliceTime(*mLastEventTime + 1);
                    if (store.isEmpty()) {
                        return;
                    }
                }
            }
            else if (lowest - *mLastEventTime > kGapThresholdUs) {
                RCLCPP_INFO(get_logger(), "%.3f s gap in event time, resetting accumulator",
                    static_cast<double>(lowest - *mLastEventTime) * 1e-6);
                resetPipeline();
            }
        }

        mLastEventTime = store.getHighestTime();
        mSlicer->accept(store);
    }

    /** Called by the slicer once per frame_interval_ms of event time. */
    void onSlice(const dv::EventStore &events) {
        if (!events.isEmpty()) {
            mAccumulator->accumulate(events);
            mLastFrameTime = events.getHighestTime();
        }
        else if (mLastFrameTime.has_value()) {
            *mLastFrameTime += mFrameIntervalUs;
        }

        dv::Frame frame     = mAccumulator->generateFrame();
        auto msg            = toRosImageMessage(frame.image);
        msg.header.stamp    = toRosTime(mLastFrameTime.value_or(frame.timestamp));
        msg.header.frame_id = mState.frameId;
        mFramePub->publish(std::move(msg));
    }

    [[nodiscard]] static int64_t toDvTime(const builtin_interfaces::msg::Time &timestamp) {
        return (static_cast<int64_t>(timestamp.sec) * 1'000'000) + (timestamp.nanosec / 1'000);
    }

    [[nodiscard]] static builtin_interfaces::msg::Time toRosTime(const int64_t timestamp) {
        builtin_interfaces::msg::Time ts;
        ts.sec     = static_cast<int32_t>(timestamp / 1'000'000);
        ts.nanosec = static_cast<uint32_t>((timestamp % 1'000'000) * 1'000);
        return ts;
    }

    /**
     * Convert an array message into an event store, dropping events outside the resolution
     * (the accumulator does not bounds-check coordinates).
     */
    [[nodiscard]] static dv::EventStore toEventStore(
        const EventArray &message, const cv::Size &resolution, std::size_t &outOfBounds) {
        dv::EventStore store;
        for (const auto &event : message.events) {
            if (event.x >= resolution.width || event.y >= resolution.height) {
                ++outOfBounds;
                continue;
            }
            store.emplace_back(toDvTime(event.ts), event.x, event.y, event.polarity);
        }
        return store;
    }

    /**
     * Convert OpenCV image into ROS image message. Supports single channel 8-bit and three channel
     * 8-bit BGR images, continuous and non-continuous memory. Performs a deep copy.
     */
    [[nodiscard]] static sensor_msgs::msg::Image toRosImageMessage(const cv::Mat &image) {
        sensor_msgs::msg::Image msg;
        msg.height = image.rows;
        msg.width  = image.cols;

        if (image.empty()) {
            return msg;
        }

        switch (image.type()) {
            case CV_8UC1: msg.encoding = sensor_msgs::image_encodings::MONO8; break;
            case CV_8UC3: msg.encoding = sensor_msgs::image_encodings::BGR8; break;
            default: throw dv::exceptions::RuntimeError("Received unsupported image type");
        }

        msg.is_bigendian  = false;
        msg.step          = msg.width * image.elemSize();
        const size_t size = msg.step * msg.height;
        msg.data.resize(size);

        if (image.isContinuous()) {
            std::memcpy(msg.data.data(), image.data, size);
        }
        else {
            uchar *dst       = msg.data.data();
            const uchar *src = image.data;
            for (int i = 0; i < image.rows; ++i) {
                std::memcpy(dst, src, msg.step);
                dst += msg.step;
                src += image.step;
            }
        }
        return msg;
    }

    rclcpp::Subscription<EventArray>::SharedPtr mSubscription;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mFramePub;
    rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr mParamCallback;
    rclcpp::TimerBase::SharedPtr mTypeCheckTimer;

    State mState;
    bool mReceivedOnTopic = false;
    std::set<std::string> mWarnedTypes;

    cv::Size mResolution;
    int64_t mFrameIntervalUs = 33'000;
    std::unique_ptr<dv::Accumulator> mAccumulator;
    std::unique_ptr<dv::EventStreamSlicer> mSlicer;
    std::optional<int64_t> mLastEventTime;
    std::optional<int64_t> mLastFrameTime;
};

} // namespace dv_playback_node

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<dv_playback_node::PlaybackNode>());
    rclcpp::shutdown();
    return 0;
}
