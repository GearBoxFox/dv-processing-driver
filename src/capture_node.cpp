#include "../include/dv-processing-driver/capture_node.hpp"

#include <optional>
#include <logging.hpp>

namespace dv_capture_node {
    CaptureNode::CaptureNode() : Node("dv_capture_node") {
        // create all publishers for data
        mEventPub = this->create_publisher<dv_processing_driver::msg::EventArray>("events", 10);
        mImuPub = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
        mAccumFramePub = this->create_publisher<sensor_msgs::msg::Image>("img_accum", 10);

        // Configure the camera and calibration
        auto calibrationPath = this->get_parameter("calibration_path").as_string();
        if (!calibrationPath.empty()) {

        } else {
            RCLCPP_DEBUG_STREAM(this->get_logger(), "[" << mCamera.getCameraName() << "] No calibration found, assuming ideal pinhole (no distortion).");
            uint8_t height = mCamera.DVS_RESOLUTION_Y;
            uint8_t width = mCamera.DVS_RESOLUTION_X;
            const auto width = static_cast<float>(width);
            populateInfoMsg(dv::camera::CameraGeometry(
                width, width, width * 0.5f, static_cast<float>(height) * 0.5f, ));
            //generateActiveCalibrationFile();
        }
    }

    void CaptureNode::populateInfoMsg(const dv::camera::CameraGeometry &cameraGeometry) {
        mCameraInfoMsg.width  = cameraGeometry.getResolution().width;
        mCameraInfoMsg.height = cameraGeometry.getResolution().height;

        const auto distortion = cameraGeometry.getDistortion();

        switch (cameraGeometry.getDistortionModel()) {
            case dv::camera::DistortionModel::EQUIDISTANT: {
                mCameraInfoMsg.distortion_model = std_msgs::msg::String().data = "equidistant";
                mCameraInfoMsg.d.assign(distortion.begin(), distortion.end());
                break;
            }

            case dv::camera::DistortionModel::RADIAL_TANGENTIAL: {
                mCameraInfoMsg.distortion_model = std_msgs::msg::String().data = "plumb_bob";
                mCameraInfoMsg.d.assign(distortion.begin(), distortion.end());
                if (mCameraInfoMsg.d.size() < 5) {
                    mCameraInfoMsg.d.resize(5, 0.0);
                }
                break;
            }

            case dv::camera::DistortionModel::NONE: {
                mCameraInfoMsg.distortion_model = std_msgs::msg::String().data = "plump_bob";
                mCameraInfoMsg.d                = {0.0, 0.0, 0.0, 0.0, 0.0};
                break;
            }

            default:
                throw dv::exceptions::InvalidArgument<dv::camera::DistortionModel>(
                    "Unsupported camera distortion model.", cameraGeometry.getDistortionModel());
        }

        auto cx = cameraGeometry.getCentralPoint().x;
        auto cy = cameraGeometry.getCentralPoint().y;
        auto fx = cameraGeometry.getFocalLength().x;
        auto fy = cameraGeometry.getFocalLength().y;

        mCameraInfoMsg.k = {fx, 0, cx, 0, fy, cy, 0, 0, 1};
        mCameraInfoMsg.r = {1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0};
        mCameraInfoMsg.p = {fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1.0, 0};
    }


    void CaptureNode::declareParameters() {
        this->declare_parameter("calibration_path", "");
    }

    
} // namespace dv_capture_node

