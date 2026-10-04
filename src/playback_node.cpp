#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <dv-processing/core/core.hpp>
#include <dv-processing/core/utils.hpp>
#include <opencv4/opencv2/core.hpp>
#include <dv-processing/core/frame.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include "dv_processing_driver/msg/event_array.hpp"
#include "rclcpp/rclcpp.hpp"

/**
 * Subscribes to an EventArray topic and keeps all received events in memory.
 * The topic can be selected at runtime with:
 *   ros2 run <package> playback_node --ros-args -p topic:=/events
 */
namespace dv_playback_node {
    class PlaybackNode final : public rclcpp::Node {
    public:
    	PlaybackNode() : Node("playback_node") {
    		const auto topic = declare_parameter<std::string>("topic", "/events");
    		const auto qos_depth = declare_parameter<int>("qos_depth", 10);

    		if (qos_depth < 1) {
    			throw std::invalid_argument("qos_depth must be greater than zero");
    		}

    		subscription_ = create_subscription<dv_processing_driver::msg::EventArray>(
    				topic, rclcpp::QoS(static_cast<std::size_t>(qos_depth)),
    				std::bind(&PlaybackNode::on_events, this, std::placeholders::_1));

            mAccumFramePub = this->create_publisher<sensor_msgs::msg::Image>("img_accum", 10);

            mFrameTimer = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&PlaybackNode::on_frame_timer, this));

            mAccumulator = dv::Accumulator(cv::Size(640, 480));

            std::cout << "Finished creating playback node" << std::endl;
    	}

    private:
    	void on_events(const dv_processing_driver::msg::EventArray::SharedPtr message) {
            dv::EventStore store = this->toEventStore(*message);
            mAccumulator.accumulate(store);
    	}

        void on_frame_timer() {
            auto frame = mAccumulator.generateFrame();
            auto msg   = this->frameToRosImageMessage(frame);
            mAccumFramePub->publish(msg);
        }

        /**
        * Convert ros::Time time into UNIX microsecond timestamp
        * @param timestamp ROS timestamp
        * @return DV format UNIX microsecond timestamp
        */
        [[nodiscard]] inline int64_t toDvTime(const builtin_interfaces::msg::Time &timestamp) {
            return (static_cast<int64_t>(timestamp.sec) * 1'000'000) + (timestamp.nanosec / 1'000);
        }

        /**
        * Converts UNIX microsecond timestamp into ros::Time format.
        * @param timestamp	DV format UNIX microsecond timestamp
        * @return ROS timestamp
        */
        [[nodiscard]] inline builtin_interfaces::msg::Time toRosTime(const int64_t timestamp) {
            builtin_interfaces::msg::Time ts;
            ts.sec = static_cast<uint32_t>(timestamp / 1'000'000);
            ts.nanosec = static_cast<uint32_t>((timestamp % 1'000'000) * 1'000);
            return ts;
        }

        /**
        * Convert an array message into an event store.
        * @param message Event array message
        * @return DV Event store
        */
        [[nodiscard]] inline dv::EventStore toEventStore(
            const dv_processing_driver::msg::EventArray &message) {
            if (message.events.empty()) {
                return {};
            }
            uint32_t seconds  = message.events.front().ts.sec;
            int64_t timestamp = static_cast<int64_t>(seconds) * 1'000'000;
            dv::EventStore store;
            // store.setShardCapacity(message.events.size());
            for (const auto &event : message.events) {
                if (event.ts.sec != seconds) {
                    seconds   = event.ts.sec;
                    timestamp = static_cast<int64_t>(seconds) * 1'000'000;
                }
                const int64_t eventTimestamp = timestamp + static_cast<int64_t>(event.ts.sec / 1000);

                if (eventTimestamp != toDvTime(event.ts)) {
                    throw dv::exceptions::RuntimeError("Timestamp conversion failed!");
                }

                store.emplace_back(eventTimestamp, event.x, event.y, event.polarity);
            }
            return store;
        }

        /**
        * Convert OpenCV image into ROS image message. Supports only single channel 8-bit, three channel 8-bit BGR images,
        * and continuous and non-continuous memory.
        * Performs deep data copy.
        * @param image OpenCV Image
        * @return ROS image (sensor_msgs::Image)
        * @throws RuntimeError If image data layout is not supported
        */
        [[nodiscard]] inline sensor_msgs::msg::Image toRosImageMessage(const cv::Mat &image) {
            sensor_msgs::msg::Image msg;

            msg.height = image.rows;
            msg.width  = image.cols;

            if (image.empty()) {
                return msg;
            }

            switch (image.type()) {
                case CV_8UC1:
                    msg.encoding = sensor_msgs::image_encodings::MONO8;
                    break;
                case CV_8UC3:
                    msg.encoding = sensor_msgs::image_encodings::BGR8;
                    break;
                default:
                    throw dv::exceptions::RuntimeError("Received unsupported image type");
            }

            msg.is_bigendian  = false;
            msg.step          = msg.width * image.elemSize();
            const size_t size = msg.step * msg.height;
            msg.data.resize(size);

            if (image.isContinuous()) {
                memcpy((char *) (&msg.data[0]), image.data, size);
            }
            else {
                auto ros_data_ptr  = (uchar *) (&msg.data[0]);
                uchar *cv_data_ptr = image.data;
                for (int i = 0; i < image.rows; ++i) {
                    memcpy(ros_data_ptr, cv_data_ptr, msg.step);
                    ros_data_ptr += msg.step;
                    cv_data_ptr += image.step;
                }
            }
            return msg;
        }

        /**
        * Converts dv::Frame into sensor_msgs::Image.
        * @param frame DV Frame containing an image.
        * @return ROS image (sensor_msgs::Image)
        * @throws RuntimeError If image data layout is not supported
        */
        [[nodiscard]] inline sensor_msgs::msg::Image frameToRosImageMessage(const dv::Frame &frame) {
            sensor_msgs::msg::Image imageMessage = toRosImageMessage(frame.image);
            imageMessage.header.stamp = toRosTime(frame.timestamp);
            return imageMessage;
        }

    	rclcpp::Subscription<dv_processing_driver::msg::EventArray>::SharedPtr subscription_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mAccumFramePub;
    	dv::Accumulator mAccumulator;
        rclcpp::TimerBase::SharedPtr mFrameTimer;
    };
} // namespace dv_playback_node

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<dv_playback_node::PlaybackNode>());
  rclcpp::shutdown();
  return 0;
}
