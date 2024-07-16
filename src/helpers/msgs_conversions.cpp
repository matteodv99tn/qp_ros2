#include "qp_ros2/helpers/msgs_conversions.hpp"

#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>

#include "qp_ros2/type_aliases.hpp"

using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::Transform;
using geometry_msgs::msg::TransformStamped;
using geometry_msgs::msg::Twist;
using geometry_msgs::msg::TwistStamped;
using geometry_msgs::msg::Wrench;
using geometry_msgs::msg::WrenchStamped;

using qp_ros2::Quaternion_t;
using qp_ros2::RotMatrix_t;
using qp_ros2::Vec3_t;

Vec3_t
qp_ros2::get_position(const Pose& msg) {
    return {msg.position.x, msg.position.y, msg.position.z};
}

Vec3_t
qp_ros2::get_position(const PoseStamped& msg) {
    return qp_ros2::get_position(msg.pose);
}

Quaternion_t
qp_ros2::get_orientation(const Pose& msg) {
    return {msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z};
}

Quaternion_t
qp_ros2::get_orientation(const PoseStamped& msg) {
    return qp_ros2::get_orientation(msg.pose);
}

Vec3_t
qp_ros2::get_linear_velocity(const Twist& msg) {
    return {msg.linear.x, msg.linear.y, msg.linear.z};
}

Vec3_t
qp_ros2::get_linear_velocity(const TwistStamped& msg) {
    return qp_ros2::get_linear_velocity(msg.twist);
}

Vec3_t
qp_ros2::get_angular_velocity(const Twist& msg) {
    return {msg.angular.x, msg.angular.y, msg.angular.z};
}

Vec3_t
qp_ros2::get_angular_velocity(const TwistStamped& msg) {
    return qp_ros2::get_angular_velocity(msg.twist);
}

Vec3_t
qp_ros2::get_translation(const Transform& msg) {
    return {msg.translation.x, msg.translation.y, msg.translation.z};
}

Vec3_t
qp_ros2::get_translation(const TransformStamped& msg) {
    return get_translation(msg.transform);
}

Quaternion_t
qp_ros2::get_rotation(const Transform& msg) {
    return {msg.rotation.w, msg.rotation.x, msg.rotation.y, msg.rotation.z};
}

Quaternion_t
qp_ros2::get_rotation(const TransformStamped& msg) {
    return get_rotation(msg.transform);
}

RotMatrix_t
qp_ros2::get_rotation_matrix(const Transform& msg) {
    return get_rotation(msg).toRotationMatrix();
}

RotMatrix_t
qp_ros2::get_rotation_matrix(const TransformStamped& msg) {
    return get_rotation(msg).toRotationMatrix();
}

Vec3_t
qp_ros2::get_force(const Wrench& msg) {
    return {msg.force.x, msg.force.y, msg.force.z};
}

Vec3_t
qp_ros2::get_force(const WrenchStamped& msg) {
    return qp_ros2::get_force(msg.wrench);
}

Vec3_t
qp_ros2::get_torque(const Wrench& msg) {
    return {msg.torque.x, msg.torque.y, msg.torque.z};
}

Vec3_t
qp_ros2::get_torque(const WrenchStamped& msg) {
    return qp_ros2::get_torque(msg.wrench);
}

std::string
qp_ros2::get_frame(const PoseStamped& msg) {
    return msg.header.frame_id;
}

std::string
qp_ros2::get_frame(const TwistStamped& msg) {
    return msg.header.frame_id;
}

std::string
qp_ros2::get_frame(const TransformStamped& msg) {
    return msg.header.frame_id;
}
