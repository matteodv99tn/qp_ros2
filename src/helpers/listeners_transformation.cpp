#include <tf2_ros/buffer.h>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include "qp_ros2/helpers/listeners.hpp"
#include "qp_ros2/type_aliases.hpp"

using geometry_msgs::msg::TransformStamped;
using qp_ros2::Quaternion_t;
using qp_ros2::TransformationHandler;
using qp_ros2::Vec3_t;

TransformationHandler::TransformationHandler(tf2_ros::Buffer::SharedPtr& buffer) :
        _desired_domain(""), _tf_buffer(buffer), _data_received(false) {}

TransformStamped
TransformationHandler::get_transform(
        const std::string& source_rf, std::optional<const std::string> target_rf
) {
    assert(_tf_buffer != nullptr);
    return _tf_buffer->lookupTransform(
            target_rf.value_or(_desired_domain), source_rf, rclcpp::Time(0)
    );
}

const std::string&
TransformationHandler::get_desired_reference_frame() const {
    return _desired_domain;
}

void
TransformationHandler::set_desired_reference_frame(const std::string& frame) {
    _desired_domain = frame;
}

bool
TransformationHandler::data_received() const {
    return _data_received;
}
