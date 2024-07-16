#ifndef QPROS2_LISTENER_HELPERS_HPP
#define QPROS2_LISTENER_HELPERS_HPP

#include <string>
#include <tf2_ros/buffer.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>

#include "qp_ros2/type_aliases.hpp"

namespace qp_ros2 {

class TransformationHandler {
public:
    TransformationHandler(tf2_ros::Buffer::SharedPtr& buffer);

    const std::string& data_frame_name() const;

    void set_desired_reference_frame(const std::string& frame);

    const std::string& get_desired_reference_frame() const;

    geometry_msgs::msg::TransformStamped get_transform(
            const std::string&               source_rf,
            std::optional<const std::string> target_rf = std::nullopt
    );

    bool data_received() const;


private:
    std::string                _desired_domain;
    tf2_ros::Buffer::SharedPtr _tf_buffer;

protected:
    bool _data_received;  //< Wether first message has been received or not
};

class PoseStampedListener : public TransformationHandler {
public:
    PoseStampedListener(
            rclcpp::Node*              node,
            tf2_ros::Buffer::SharedPtr tf_buffer,
            const std::string&         topic
    );

    const Vec3_t&       last_position() const;
    const Quaternion_t& last_orientation() const;
    const std::string&  get_data_reference_frame() const;


private:
    void on_pose_msg_received(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg
    );
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _sub;

    Vec3_t       _pos;
    Quaternion_t _ori;
    std::string  _data_domain;
};

class TwistStampedListener : public TransformationHandler {
public:
    TwistStampedListener(
            rclcpp::Node*              node,
            tf2_ros::Buffer::SharedPtr tf_buffer,
            const std::string&         topic
    );

    const Vec3_t&      last_linear_velocity() const;
    const Vec3_t&      last_angular_velocity() const;
    const std::string& get_data_reference_frame() const;


private:
    void on_twist_msg_received(
            const geometry_msgs::msg::TwistStamped::ConstSharedPtr& msg
    );
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr _sub;

    Vec3_t      _vel;
    Vec3_t      _omega;
    std::string _data_domain;
};

class WrenchStampedListener : public TransformationHandler {
public:
    WrenchStampedListener(
            rclcpp::Node*              node,
            tf2_ros::Buffer::SharedPtr tf_buffer,
            const std::string&         topic
    );

    const Vec3_t&      last_force() const;
    const Vec3_t&      last_torque() const;
    const std::string& get_data_reference_frame() const;


private:
    void on_wrench_msg_received(
            const geometry_msgs::msg::WrenchStamped::ConstSharedPtr& msg
    );
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr _sub;

    Vec3_t      _force;
    Vec3_t      _torque;
    std::string _data_domain;
};

}  // namespace qp_ros2


#endif  // QPROS2_LISTENER_HELPERS_HPP
