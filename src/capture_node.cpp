#include "../include/dv-processing-driver/capture_node.hpp"

#include <optional>
#include<filesystem>
#include <opencv4/opencv2/core/types.hpp>
#include <iostream>

namespace dv_capture_node {
    CaptureNode::CaptureNode() : Node("dv_capture_node") {
        declareParameters();

        // create all publishers for data
        mEventPub = this->create_publisher<dv_processing_driver::msg::EventArray>("events", 10);
        mImuPub = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
        mAccumFramePub = this->create_publisher<sensor_msgs::msg::Image>("img_accum", 10);

        // Configure the camera and calibration
        auto calibrationPath = getActiveCalibrationPath();
        std::cout << calibrationPath << std::endl;
        if (this->get_parameter("calibration_path").as_string() != "") {
            RCLCPP_INFO_STREAM(this->get_logger(), "Loading user supplied calibration at path [" << calibrationPath << "]");
            if (!fs::exists(calibrationPath)) {
                throw dv::exceptions::InvalidArgument<std::string>(
                    "User supplied calibration file does not exist!", calibrationPath);
            }
            RCLCPP_INFO_STREAM(this->get_logger(), fmt::format("Loading calibration data from {0}...", calibrationPath));
            fs::copy_file(this->get_parameter("calibration_path").as_string(), calibrationPath, fs::copy_options::overwrite_existing);
        }

        if (fs::exists(calibrationPath)) {

        } else {
            RCLCPP_WARN_STREAM(this->get_logger(), "[" << mCamera->getCameraName() << "] No calibration found, assuming ideal pinhole (no distortion).");
            cv::Size resolution = mCamera->getEventResolution().value();
            const auto width = static_cast<float>(resolution.width);
            populateInfoMsg(dv::camera::CameraGeometry(
                width, width, width * 0.5f, static_cast<float>(resolution.height) * 0.5f, resolution));
            // generateActiveCalibrationFile();
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

    fs::path CaptureNode::getCameraCalibrationDirectory(const bool createDirectories) const {
        const fs::path directory
            = fmt::format("{0}/.dv_camera/camera_calibration/{1}", std::getenv("HOME"), mCamera->getCameraName());
        if (createDirectories && !fs::exists(directory)) {
            fs::create_directories(directory);
        }
        return directory;
    }

    fs::path CaptureNode::getActiveCalibrationPath() const {
        return getCameraCalibrationDirectory() / "active_calibration.json";
    }

    void CaptureNode::generateActiveCalibrationFile() {
        RCLCPP_INFO_STREAM(this->get_logger(), "Generating active calibration file...");
        updateCalibrationSet();
        mCalibration.writeToFile(getActiveCalibrationPath());
    }

    fs::path CaptureNode::saveCalibration() {
        auto date = fmt::format("{:%Y_%m_%d_%H_%M_%S}", dv::toTimePoint(dv::now()));
        const std::string calibrationFileName
            = fmt::format("calibration_camera_{0}_{1}.json", mCamera->getCameraName(), date);
        const fs::path calibPath = getCameraCalibrationDirectory() / calibrationFileName;
        updateCalibrationSet();
        mCalibration.writeToFile(calibPath);

        fs::copy_file(calibPath, getActiveCalibrationPath(), fs::copy_options::overwrite_existing);
        return calibPath;
    }

    void CaptureNode::updateCalibrationSet() {
        RCLCPP_INFO_STREAM(this->get_logger(), "Generating calibration set...");
        const std::string cameraName = mCamera->getCameraName();
        dv::camera::calibrations::CameraCalibration calib;
        bool calibrationExists = false;
        if (auto camCalibration = mCalibration.getCameraCalibrationByName(cameraName); camCalibration.has_value()) {
            calib             = *camCalibration;
            calibrationExists = true;
        }
        else {
            calib.name = cameraName;
        }
        calib.resolution = cv::Size(static_cast<int>(mCameraInfoMsg.width), static_cast<int>(mCameraInfoMsg.height));
        calib.distortion.clear();
        calib.distortion.assign(mCameraInfoMsg.d.begin(), mCameraInfoMsg.d.end());
        if (mCameraInfoMsg.distortion_model == "plumb_bob") {
            calib.distortionModel = dv::camera::DistortionModel::RADIAL_TANGENTIAL;
        }
        else if (mCameraInfoMsg.distortion_model == "equidistant") {
            calib.distortionModel = dv::camera::DistortionModel::EQUIDISTANT;
        }
        else {
            // throw dv::exceptions::InvalidArgument<std_msgs::msg::String>(
                // "Unknown camera model.", mCameraInfoMsg.distortion_model);
        }
        calib.focalLength = cv::Point2f(static_cast<float>(mCameraInfoMsg.k[0]), static_cast<float>(mCameraInfoMsg.k[4]));
        calib.principalPoint
            = cv::Point2f(static_cast<float>(mCameraInfoMsg.k[2]), static_cast<float>(mCameraInfoMsg.k[5]));

        calib.transformationToC0 = dv::kinematics::Transformationf{};

        if (calibrationExists) {
            mCalibration.updateCameraCalibration(calib);
        }
        else {
            mCalibration.addCameraCalibration(calib);
        }

        dv::camera::calibrations::IMUCalibration imuCalibration;
        bool imuCalibrationExists = false;
        if (auto imuCalib = mCalibration.getImuCalibrationByName(cameraName); imuCalib.has_value()) {
            imuCalibration       = *imuCalib;
            imuCalibrationExists = true;
        }
        else {
            imuCalibration.name = cameraName;
        }
        bool imuHasValues = false;
        if ((!mImuToCamTransforms.transforms.empty())) {
            const Eigen::Matrix4f mat         = mImuToCamTransform.getTransform().transpose();
            imuCalibration.transformationToC0 = dv::kinematics::Transformationf{0, mat};
            imuHasValues                      = true;
        }

        if (!mAccBiases.isZero()) {
            imuCalibration.accOffsetAvg.x = mAccBiases.x();
            imuCalibration.accOffsetAvg.y = mAccBiases.y();
            imuCalibration.accOffsetAvg.z = mAccBiases.z();
            imuHasValues                  = true;
        }

        if (!mGyroBiases.isZero()) {
            imuCalibration.omegaOffsetAvg.x = mGyroBiases.x();
            imuCalibration.omegaOffsetAvg.y = mGyroBiases.y();
            imuCalibration.omegaOffsetAvg.z = mGyroBiases.z();
            imuHasValues                    = true;
        }

        if (mImuTimeOffset > 0) {
            imuCalibration.timeOffsetMicros = mImuTimeOffset;
            imuHasValues                    = true;
        }

        if (imuCalibrationExists) {
            mCalibration.updateImuCalibration(imuCalibration);
        }
        else if (imuHasValues) {
            mCalibration.addImuCalibration(imuCalibration);
        }
    }


    void CaptureNode::declareParameters() {
        this->declare_parameter("calibration_path", "");
    }

    
} // namespace dv_capture_node

