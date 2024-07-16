#include <tf2_ros/buffer.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/node.hpp>

#include "qp_ros2/helpers/listeners.hpp"
#include "qp_ros2/helpers/msgs_conversions.hpp"
#include "qp_ros2/type_aliases.hpp"

using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::TransformStamped;
using qp_ros2::PoseStampedListener;
using qp_ros2::Quaternion_t;
using qp_ros2::Vec3_t;

PoseStampedListener::PoseStampedListener(
        rclcpp::Node*              node,
        tf2_ros::Buffer::SharedPtr tf_buffer,
        const std::string&         topic
) :
        TransformationHandler(tf_buffer),
        _pos(Vec3_t::Zero()),
        _ori(Quaternion_t::Identity()),
        _data_domain("") {
    _sub = node->create_subscription<PoseStamped>(
            topic,
            rclcpp::QoS(10),
            [this](const PoseStamped::ConstSharedPtr& msg) {
                return on_pose_msg_received(msg);
            }
    );
}

void
PoseStampedListener::on_pose_msg_received(const PoseStamped::ConstSharedPtr& msg) {
    _pos         = get_position(*msg);
    _ori         = get_orientation(*msg);
    _data_domain = msg->header.frame_id;
    _data_received    = true;


    if (get_desired_reference_frame().empty()
        || (get_desired_reference_frame() == _data_domain))
        return;

    const TransformStamped transf = get_transform(msg->header.frame_id);

    const Vec3_t       t  = get_translation(transf);
    const Quaternion_t qR = get_rotation(transf);
    const RotMatrix_t  R  = qR.toRotationMatrix();

    _pos         = t + R * _pos;
    _ori         = qR * _ori;
    _data_domain = get_desired_reference_frame();
}

const Vec3_t&
PoseStampedListener::last_position() const {
    return _pos;
}

const Quaternion_t&
PoseStampedListener::last_orientation() const {
    return _ori;
}

const std::string&
PoseStampedListener::get_data_reference_frame() const {
    return _data_domain;
}
