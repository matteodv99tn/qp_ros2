#include <tf2_ros/buffer.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/node.hpp>

#include "qp_ros2/helpers/listeners.hpp"
#include "qp_ros2/helpers/msgs_conversions.hpp"
#include "qp_ros2/type_aliases.hpp"

using geometry_msgs::msg::TransformStamped;
using geometry_msgs::msg::TwistStamped;
using qp_ros2::Quaternion_t;
using qp_ros2::TwistStampedListener;
using qp_ros2::Vec3_t;

TwistStampedListener::TwistStampedListener(
        rclcpp::Node*              node,
        tf2_ros::Buffer::SharedPtr tf_buffer,
        const std::string&         topic
) :
        TransformationHandler(tf_buffer),
        _vel(Vec3_t::Zero()),
        _omega(Vec3_t::Zero()),
        _data_domain("") {
    _sub = node->create_subscription<TwistStamped>(
            topic,
            rclcpp::QoS(10),
            [this](const TwistStamped::ConstSharedPtr& msg) {
                return on_twist_msg_received(msg);
            }
    );
}

void
TwistStampedListener::on_twist_msg_received(const TwistStamped::ConstSharedPtr& msg) {
    _vel         = get_linear_velocity(*msg);
    _omega       = get_angular_velocity(*msg);
    _data_domain = msg->header.frame_id;
    _data_received    = true;

    if (get_desired_reference_frame().empty()
        || (get_desired_reference_frame() == _data_domain))
        return;

    const TransformStamped transf = get_transform(msg->header.frame_id);
    const RotMatrix_t      R      = get_rotation_matrix(transf);

    _vel         = R * _vel;
    _omega       = R * _omega;
    _data_domain = get_desired_reference_frame();
}

const Vec3_t&
TwistStampedListener::last_linear_velocity() const {
    return _vel;
}

const Vec3_t&
TwistStampedListener::last_angular_velocity() const {
    return _omega;
}

const std::string&
TwistStampedListener::get_data_reference_frame() const {
    return _data_domain;
}
