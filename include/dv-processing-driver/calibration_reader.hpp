#include "pugixml.hpp"
#include <dv-processing/camera/calibrations/camera_calibration.hpp>
#include <dv-processing/camera/calibration_set.hpp>

// Reads the XML calibration file and returns the calibration data as a dv::camera::CalibrationSet object.
dv::camera::calibrations::CameraCalibration readCameraCalibrationFromXml(const std::string &filePath, const std::string &cameraName) {
    dv::camera::calibrations::CameraCalibration calibrationSet;

    pugi::xml_document doc;
    pugi::xml_parse_result result = doc.load_file(filePath.c_str());

    if (!result) {
        throw dv::exceptions::RuntimeError("Failed to load XML file: " + std::string(result.description()));
    }

    pugi::xml_node root = doc.child("opencv_storage").child(cameraName.c_str());
    return calibrationSet;
}
