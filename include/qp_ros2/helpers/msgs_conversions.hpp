#ifndef QPROS2_MSGS_CONVERSIONS_HPP
#define QPROS2_MSGS_CONVERSIONS_HPP

#include <string>

#include <geometry_msgs/msg/detail/wrench_stamped__struct.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>

#include "qp_ros2/type_aliases.hpp"

namespace qp_ros2 {

Vec3_t       get_position(const geometry_msgs::msg::Pose& msg);
Vec3_t       get_position(const geometry_msgs::msg::PoseStamped& msg);
Quaternion_t get_orientation(const geometry_msgs::msg::Pose& msg);
Quaternion_t get_orientation(const geometry_msgs::msg::PoseStamped& msg);


Vec3_t get_linear_velocity(const geometry_msgs::msg::Twist& msg);
Vec3_t get_linear_velocity(const geometry_msgs::msg::TwistStamped& msg);
Vec3_t get_angular_velocity(const geometry_msgs::msg::Twist& msg);
Vec3_t get_angular_velocity(const geometry_msgs::msg::TwistStamped& msg);

std::string get_frame(const geometry_msgs::msg::PoseStamped& msg);
std::string get_frame(const geometry_msgs::msg::TwistStamped& msg);
std::string get_frame(const geometry_msgs::msg::TransformStamped& msg);

Vec3_t get_translation(const geometry_msgs::msg::Transform& msg);
Vec3_t get_translation(const geometry_msgs::msg::TransformStamped& msg);

Quaternion_t get_rotation(const geometry_msgs::msg::Transform& msg);
Quaternion_t get_rotation(const geometry_msgs::msg::TransformStamped& msg);
RotMatrix_t  get_rotation_matrix(const geometry_msgs::msg::Transform& msg);
RotMatrix_t  get_rotation_matrix(const geometry_msgs::msg::TransformStamped& msg);

Vec3_t get_force(const geometry_msgs::msg::Wrench& msg);
Vec3_t get_force(const geometry_msgs::msg::WrenchStamped& msg);
Vec3_t get_torque(const geometry_msgs::msg::Wrench& msg);
Vec3_t get_torque(const geometry_msgs::msg::WrenchStamped& msg);


}  // namespace qp_ros2


#endif  // QPROS2_MSGS_CONVERSIONS_HPP
