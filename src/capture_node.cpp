#include "../include/dv-processing-driver/capture_node.hpp"

#include<filesystem>
#include <opencv4/opencv2/core/types.hpp>
#include <iostream>
#include <chrono>

namespace dv_capture_node {
    CaptureNode::CaptureNode() : Node("dv_capture_node") {
        declareParameters();

        // create all publishers for data
        mEventPub = this->create_publisher<dv_processing_driver::msg::EventArray>("events", 10);
        mImuPub = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
        mAccumFramePub = this->create_publisher<sensor_msgs::msg::Image>("img_accum", 10);

        // create the accumulator for the frames
        mAccumulator = dv::Accumulator(mCamera->getEventResolution().value());

        // Configure the camera and calibration
        fs::path calibrationPath = getActiveCalibrationPath();
        if (this->get_parameter("calibration_path").as_string() != "") {
            RCLCPP_INFO_STREAM(this->get_logger(), "Loading user supplied calibration at path [" << this->get_parameter("calibration_path").as_string() << "]");
            if (!fs::exists(this->get_parameter("calibration_path").as_string())) {
                throw dv::exceptions::InvalidArgument<std::string>(
                    "User supplied calibration file does not exist!", this->get_parameter("calibration_path").as_string());
            }
            RCLCPP_INFO_STREAM(this->get_logger(), fmt::format("Loading calibration data from {0}...", this->get_parameter("calibration_path").as_string()));
            fs::copy_file(this->get_parameter("calibration_path").as_string(), calibrationPath, fs::copy_options::overwrite_existing);
        }

        if (fs::exists(calibrationPath)) {
            RCLCPP_INFO_STREAM(this->get_logger(), "Loading calibration file [" << calibrationPath << "]");
            mCalibration                 = dv::camera::CalibrationSet::LoadFromFile(calibrationPath);
            const std::string cameraName = mCamera->getCameraName();
            auto cameraCalibration       = mCalibration.getCameraCalibrationByName(cameraName);
            if (const auto &imuCalib = mCalibration.getImuCalibrationByName(cameraName); imuCalib.has_value()) {
                mTransformPublisher = this->create_publisher<tf2_msgs::msg::TFMessage>("/tf", 100);
                mImuTimeOffset      = imuCalib->timeOffsetMicros;

                geometry_msgs::msg::TransformStamped msg;
                msg.header.frame_id = this->get_parameter("imu_frame_name").as_string();
                msg.child_frame_id  = this->get_parameter("camera_frame_name").as_string();

                mImuToCamTransform = dv::kinematics::Transformationf(0, imuCalib->transformationToC0.getTransform());

                mAccBiases.x() = imuCalib->accOffsetAvg.x;
                mAccBiases.y() = imuCalib->accOffsetAvg.y;
                mAccBiases.z() = imuCalib->accOffsetAvg.z;

                mGyroBiases.x() = imuCalib->omegaOffsetAvg.x;
                mGyroBiases.y() = imuCalib->omegaOffsetAvg.y;
                mGyroBiases.z() = imuCalib->omegaOffsetAvg.z;

                const auto translation      = mImuToCamTransform.getTranslation<Eigen::Vector3d>();
                msg.transform.translation.x = translation.x();
                msg.transform.translation.y = translation.y();
                msg.transform.translation.z = translation.z();

                const auto rotation      = mImuToCamTransform.getQuaternion();
                msg.transform.rotation.x = rotation.x();
                msg.transform.rotation.y = rotation.y();
                msg.transform.rotation.z = rotation.z();
                msg.transform.rotation.w = rotation.w();

                mImuToCamTransforms = tf2_msgs::msg::TFMessage();
                mImuToCamTransforms.transforms.push_back(msg);
            }
            if (cameraCalibration.has_value()) {
                populateInfoMsg(cameraCalibration->getCameraGeometry());
            }
            else {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Calibration in [" << calibrationPath << "] does not contain calibration for camera ["
                                                    << cameraName << "]");
                std::vector<std::string> names;
                for (const auto &calib : mCalibration.getCameraCalibrations()) {
                    names.push_back(calib.second.name);
                }
                const std::string nameString = fmt::format("{}", fmt::join(names, "; "));
                RCLCPP_ERROR_STREAM(this->get_logger(), "The file only contains calibrations for these cameras: [" << nameString << "]");
                throw std::runtime_error("Calibration is not available!");
            }
        }
        else {
            RCLCPP_WARN_STREAM(this->get_logger(),
                "[" << mCamera->getCameraName() << "] No calibration was found, assuming ideal pinhole (no distortion).");
            std::optional<cv::Size> resolution;
            if (mCamera->isFrameStreamAvailable()) {
                resolution = mCamera->getFrameResolution();
            }
            else if (mCamera->isEventStreamAvailable()) {
                resolution = mCamera->getEventResolution();
            }
            if (resolution.has_value()) {
                const auto width = static_cast<float>(resolution->width);
                populateInfoMsg(dv::camera::CameraGeometry(
                    width, width, width * 0.5f, static_cast<float>(resolution->height) * 0.5f, *resolution));
                generateActiveCalibrationFile();
            }
            else {
                throw std::runtime_error("Sensor resolution not available.");
            }
        }


        // setup callback publishers on a timer
        RCLCPP_INFO_STREAM(this->get_logger(), "Creating event callback timer...");
        mEventTimer = this->create_wall_timer(
            std::chrono::milliseconds(500), std::bind(&CaptureNode::eventCallback, this)
        );

        if (this->get_parameter("accumulate_frames").as_bool()) {
            RCLCPP_INFO_STREAM(this->get_logger(), "Creating frame callback timer...");
            mFrameTimer = this->create_wall_timer(
                std::chrono::milliseconds(500), std::bind(&CaptureNode::frameCallback, this)
            );
        }
    }

    // Populates the camera info message with the given camera geometry
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

    // Returns the directory where camera calibration files are stored
    fs::path CaptureNode::getCameraCalibrationDirectory(const bool createDirectories) const {
        const fs::path directory
            = fmt::format("{0}/.dv_camera/camera_calibration/{1}", std::getenv("HOME"), mCamera->getCameraName());
        if (createDirectories && !fs::exists(directory)) {
            fs::create_directories(directory);
        }
        return directory;
    }

    // Returns the path to the active calibration file for the currently opened camera
    fs::path CaptureNode::getActiveCalibrationPath() const {
        return getCameraCalibrationDirectory() / "active_calibration.json";
    }

    // Generates the active calibration file for the currently opened camera
    void CaptureNode::generateActiveCalibrationFile() {
        RCLCPP_INFO_STREAM(this->get_logger(), "Generating active calibration file...");
        updateCalibrationSet();
        mCalibration.writeToFile(getActiveCalibrationPath());
    }

    //  Saves the current calibration set to a new file and returns the path to the new file
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

    // Updates the calibration set with the current camera info and IMU info
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

    // Handles the event callback logic
    void CaptureNode::eventCallback() {
        // RCLCPP_INFO_STREAM(this->get_logger(), "Starting to publish events.");
        int i = 0;

        if (!mEvents.has_value()) {
                mEvents = mCamera->getNextEventBatch();
                // RCLCPP_INFO_STREAM(this->get_logger(), "No events! Getting next batch");
            }
        while (mEvents.has_value() && !mEvents->isEmpty()) {
            dv::EventStore store;
            // todo add filtering
            store = *mEvents;

            if (mEventPub->get_subscription_count() > 0) {
                auto msg = this->toRosEventsMessage(store, mResolution);
                mEventPub->publish(msg);
            }

            if (this->get_parameter("accumulate_frames").as_bool()) {
                mAccumulator.accumulate(store);
            }

            i++;
            mEvents = mCamera->getNextEventBatch();
        }

        // RCLCPP_INFO_STREAM(this->get_logger(), "Finished publishing " << i << " events!");
    }

    void CaptureNode::frameCallback() {
        auto frame = mAccumulator.generateFrame();
        auto msg   = this->toRosImageMessage(frame.image);
        mAccumFramePub->publish(msg);
    }

    // Declares the parameters for the capture node
    void CaptureNode::declareParameters() {
        this->declare_parameter("calibration_path", "");
        this->declare_parameter("accumulate_frames", true);
        this->declare_parameter("imu_frame_name", "imu_link");
        this->declare_parameter("camera_frame_name", "camera_link");
        this->declare_parameter("camera_calibration_path", "");
        this->declare_parameter("imu_calibration_path", "");
    }

    
} // namespace dv_capture_node

