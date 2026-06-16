#include <dv-processing/io/camera/dvxplorer_m.hpp>
#include <dv-processing/camera/calibrations/camera_calibration.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <opencv4/opencv2/core.hpp>

#include <std_msgs/msg/string.hpp>

#include "rclcpp/rclcpp.hpp"
#include "dv_processing_driver/msg/event_array.hpp"

namespace dv_capture_node {
class CaptureNode : public rclcpp::Node {
    public:
        // Initialize the publisher node
        CaptureNode();

        // Stop the running threads
        ~CaptureNode();

        // Start the capture of data
        void startCapture();

        // Stop the capture of data
        void stop();

        // Returns whether the capture of data is still running
        bool isRunning() const;

    private:
        // declare the camera device
        dv::io::camera::DVXplorerM mCamera{};

        // publisher declaration
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr mCamInfoPub;
        rclcpp::Publisher<dv_processing_driver::msg::EventArray>::SharedPtr mEventPub;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr mImuPub;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mAccumFramePub;

        sensor_msgs::msg::CameraInfo mCameraInfoMsg;

        // startup functions
        void populateInfoMsg(const dv::camera::CameraGeometry &cameraGeometry);
        void declareParameters();
};
} // namespace dv_capture_node