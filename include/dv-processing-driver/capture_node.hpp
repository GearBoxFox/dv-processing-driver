#include <dv-processing/io/camera/dvxplorer_m.hpp>
#include <dv-processing/camera/calibration_set.hpp>
#include <dv-processing/camera/calibrations/camera_calibration.hpp>
#include <dv-processing/io/camera/discovery.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

// #include <sensor_msgs/srv/SetCameraInfo.hpp>

#include <opencv4/opencv2/core.hpp>
#include <filesystem>

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

        sensor_msgs::msg::CameraInfo mCameraInfoMsg;

        dv::camera::CalibrationSet mCalibration;

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

        // services
        // bool setCameraInfo(sensor_msgs::srv::SetCameraInfo::Request &req, sensor_msgs::SetCameraInfo::Response &rsp);
        // bool setImuInfo(dv_ros_capture::SetImuInfo::Request &req, dv_ros_capture::SetImuInfo::Response &rsp);
	    // bool setImuBiases(dv_ros_capture::SetImuBiases::Request &req, dv_ros_capture::SetImuBiases::Response &rsp);
};
} // namespace dv_capture_node