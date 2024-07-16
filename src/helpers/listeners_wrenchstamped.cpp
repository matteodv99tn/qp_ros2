#include <tf2_ros/buffer.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/node.hpp>

#include "qp_ros2/helpers/listeners.hpp"
#include "qp_ros2/helpers/msgs_conversions.hpp"
#include "qp_ros2/type_aliases.hpp"

using geometry_msgs::msg::TransformStamped;
using geometry_msgs::msg::WrenchStamped;
using qp_ros2::Quaternion_t;
using qp_ros2::Vec3_t;
using qp_ros2::WrenchStampedListener;

WrenchStampedListener::WrenchStampedListener(
        rclcpp::Node*              node,
        tf2_ros::Buffer::SharedPtr tf_buffer,
        const std::string&         topic
) :
        TransformationHandler(tf_buffer),
        _force(Vec3_t::Zero()),
        _torque(Vec3_t::Zero()),
        _data_domain("") {
    _sub = node->create_subscription<WrenchStamped>(
            topic,
            rclcpp::QoS(10),
            [this](const WrenchStamped::ConstSharedPtr& msg) {
                return on_wrench_msg_received(msg);
            }
    );
}

void
WrenchStampedListener::on_wrench_msg_received(const WrenchStamped::ConstSharedPtr& msg
) {
    _force         = get_force(*msg);
    _torque        = get_torque(*msg);
    _data_domain   = msg->header.frame_id;
    _data_received = true;

    if (get_desired_reference_frame().empty()
        || (get_desired_reference_frame() == _data_domain))
        return;

    const TransformStamped transf = get_transform(msg->header.frame_id);
    const RotMatrix_t      R      = get_rotation_matrix(transf);

    _force       = R * _force;
    _torque      = R * _torque;
    _data_domain = get_desired_reference_frame();
}

const Vec3_t&
WrenchStampedListener::last_force() const {
    return _force;
}

const Vec3_t&
WrenchStampedListener::last_torque() const {
    return _torque;
}

const std::string&
WrenchStampedListener::get_data_reference_frame() const {
    return _data_domain;
}
