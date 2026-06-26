#pragma once

#include <dv-processing/io/camera/dvxplorer_m.hpp>
#include <dv-processing/camera/calibration_set.hpp>
#include <dv-processing/camera/calibrations/camera_calibration.hpp>
#include <dv-processing/io/camera/discovery.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <optional>

// #include <sensor_msgs/srv/SetCameraInfo.hpp>

#include <opencv4/opencv2/core.hpp>
#include <filesystem>
#include <optional>

#include <std_msgs/msg/string.hpp>

#include "rclcpp/rclcpp.hpp"
#include "dv_processing_driver/msg/event_array.hpp"

namespace fs = std::filesystem;

namespace dv_capture_node {
class CaptureNode : public rclcpp::Node {
    public:
        // Initialize the publisher node
        CaptureNode();

        // Stop the running threads
        ~CaptureNode() = default;

        // Start the capture of data
        void startCapture();

        // Stop the capture of data
        void stop();

        // Returns whether the capture of data is still running
        bool isRunning() const;

    private:
        // declare the camera device
        dv::io::camera::CameraPtr mCamera = dv::io::camera::open();

        // publisher declaration
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr mCamInfoPub;
        rclcpp::Publisher<dv_processing_driver::msg::EventArray>::SharedPtr mEventPub;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr mImuPub;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mAccumFramePub;
        rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr mTransformPublisher;

        sensor_msgs::msg::CameraInfo mCameraInfoMsg;

        // Event publisher objects
        rclcpp::TimerBase::SharedPtr mEventTimer;
        std::optional<dv::EventStore> mEvents = std::nullopt;
        cv::Size mResolution;

        // Frame accumulator
        dv::Accumulator mAccumulator;
        rclcpp::TimerBase::SharedPtr mFrameTimer;

        rclcpp::TimerBase::SharedPtr mImuTimer;
        std::optional<std::vector<dv::IMU>> mImuData = std::nullopt;

        // Camera calibration and configuration
        dv::camera::CalibrationSet mCalibration;
        int64_t mImuTimeOffset      = 0;
        Eigen::Vector3f mAccBiases  = Eigen::Vector3f::Zero();
        Eigen::Vector3f mGyroBiases = Eigen::Vector3f::Zero();
        rclcpp::Time startupTime;
        std::atomic<int64_t> mCurrentSeek;
        tf2_msgs::msg::TFMessage mImuToCamTransforms;
        dv::kinematics::Transformationf mImuToCamTransform
            = dv::kinematics::Transformationf(0, Eigen::Vector3f::Zero(), Eigen::Quaternion<float>::Identity());

        // startup functions
        void populateInfoMsg(const dv::camera::CameraGeometry &cameraGeometry);
        void declareParameters();

        /**
         * Generate the CalibrationSet with the data from the Set Camera Info and the set IMU services.
         * @return dv::camera::CalibrationSet
         */
        void updateCalibrationSet();

        /**
         * Stores the calibration data into a new file.
         * @return path to the new file.
         */
        [[nodiscard]] fs::path saveCalibration();

        /**
         * Write current capture node calibration parameters into an active calibration file.
         */
        void generateActiveCalibrationFile();

        /**
         * Get the path to the active calibration file.
         * @return Filesystem path to the currently opened camera active calibration file.
         */
        [[nodiscard]] fs::path getActiveCalibrationPath() const;

        /**
         * Get camera calibration directory for the currently opened camera, it uses
         * @param createDirectories If true, the method will create the directory if it's not existing in the filesystem.
         * @return Path to the calibration
         */
        fs::path getCameraCalibrationDirectory(bool createDirectories = true) const;

        sensor_msgs::msg::Imu transformImuFrame(sensor_msgs::msg::Imu &&imu);

        /** Handles the callback logic for publishing event data */
        void eventCallback();

        /** Handles the callback logic for publishing frame data */
        void frameCallback();

        void imuCallback();

        // services
        // bool setCameraInfo(sensor_msgs::srv::SetCameraInfo::Request &req, sensor_msgs::SetCameraInfo::Response &rsp);
        // bool setImuInfo(dv_ros_capture::SetImuInfo::Request &req, dv_ros_capture::SetImuInfo::Response &rsp);
	    // bool setImuBiases(dv_ros_capture::SetImuBiases::Request &req, dv_ros_capture::SetImuBiases::Response &rsp);

        // Datatype converters
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
         * Convert ros::Time time into UNIX microsecond timestamp
         * @param timestamp ROS timestamp
         * @return DV format UNIX microsecond timestamp
         */
        [[nodiscard]] inline int64_t toDvTime(const builtin_interfaces::msg::Time &timestamp) {
            return (static_cast<int64_t>(timestamp.sec) * 1'000'000) + (timestamp.nanosec / 1'000);
        }

        dv_processing_driver::msg::EventArray toRosEventsMessage(const dv::EventStore &events, const cv::Size &resolution) {
        dv_processing_driver::msg::EventArray msg;
        builtin_interfaces::msg::Time time = toRosTime(events.getLowestTime());

        int64_t secInMicro = static_cast<int64_t>(time.sec) * 1'000'000;
        builtin_interfaces::msg::Time ts = builtin_interfaces::msg::Time();
        ts.nanosec = static_cast<double>(events.getHighestTime()) * 1e-6;
        msg.header.stamp = ts;
        msg.events.reserve(events.size());
        for (const auto &event : events) {
            int64_t time_diff = event.timestamp() - secInMicro;
            if (time_diff < 1'000'000) {
                // We are in the same second, we only need to update the nano-second part
                time.nanosec = time_diff * 1'000;
            }
            else {
                time       = toRosTime(event.timestamp());
                secInMicro = static_cast<int64_t>(time.sec) * 1'000'000;
            }
            auto &e    = msg.events.emplace_back();
            e.x        = event.x();
            e.y        = event.y();
            e.polarity = event.polarity();
            e.ts       = time;
        }

        msg.width  = resolution.width;
        msg.height = resolution.height;
        return msg;
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

    /**
     * Convert dv::IMU into sensor_msgs::Imu
     * @param imu DV IMU measurement
     * @return ROS Imu message
     */
    [[nodiscard]] inline sensor_msgs::msg::Imu toRosImuMessage(const dv::IMU &imu) {
        sensor_msgs::msg::Imu imuMessage;
        imuMessage.header.stamp = toRosTime(imu.timestamp);

        constexpr float deg2rad = std::numbers::pi_v<float> / 180.0f;
        constexpr float earthG  = 9.81007f;

        imuMessage.angular_velocity.x    = imu.gyroscopeX * deg2rad;
        imuMessage.angular_velocity.y    = imu.gyroscopeY * deg2rad;
        imuMessage.angular_velocity.z    = imu.gyroscopeZ * deg2rad;
        imuMessage.linear_acceleration.x = imu.accelerometerX * earthG;
        imuMessage.linear_acceleration.y = imu.accelerometerY * earthG;
        imuMessage.linear_acceleration.z = imu.accelerometerZ * earthG;

        return imuMessage;
    }
};
} // namespace dv_capture_node