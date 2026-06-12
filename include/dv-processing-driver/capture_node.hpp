#include <dv-processing/io/camera/dvxplorer_m.hpp>
#include "rclcpp/rclcpp.hpp"

namespace dv_capture_node {
class CaptureNode : public rclcpp::Node {
    public:
        // Initialize the publisher node
        CaptureNode();

        // Stop the running threads
        ~CaptureNode();

        // Start the threads for reading data
        void startCapture();

        // Stop the running threads
        void stop();

        // Returns whether the read threads are still running
        bool isRunning() const;

    private:
        // declare the camera device
        dv::io::camera::CameraPtr capture;

        // publisher declaration

        // thread related
};
} // namespace dv_capture_node