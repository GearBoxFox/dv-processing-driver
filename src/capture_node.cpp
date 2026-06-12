#include "../include/dv-processing-driver/capture_node.hpp"

namespace dv_capture_node {
    CaptureNode::CaptureNode() : Node("dv_capture_node") {
        mEventPub = 
        this->create_publisher<dv_processing_driver::msg::EventArray>("events", 10);
    }
} // namespace dv_capture_node

