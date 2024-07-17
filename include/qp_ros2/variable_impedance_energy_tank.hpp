#ifndef QPROS2_CARTESIAN_ENERGY_TANK_HPP
#define QPROS2_CARTESIAN_ENERGY_TANK_HPP

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <rclcpp/node.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "qp_ros2/helpers/listeners.hpp"
#include "qp_ros2/qp_wrapper.hpp"
#include "qp_ros2/type_aliases.hpp"

namespace qp_ros2 {

class VariableImpedanceWithEnergyTank : public rclcpp::Node {
public:
    VariableImpedanceWithEnergyTank();

private:
    using Float64MultiArray = std_msgs::msg::Float64MultiArray;
    using Float64           = std_msgs::msg::Float64;
    QpWrapper<6, 9> _qp;

    std::unique_ptr<PoseStampedListener>   _desired_pos;
    std::unique_ptr<PoseStampedListener>   _current_pos;
    std::unique_ptr<TwistStampedListener>  _desired_vel;
    std::unique_ptr<TwistStampedListener>  _current_vel;
    std::unique_ptr<WrenchStampedListener> _desired_wrench;
    bool                                   _all_messages_received;

    tf2_ros::Buffer::SharedPtr                  _tf_buffer;
    std::unique_ptr<tf2_ros::TransformListener> _tf_listener;

    rclcpp::Publisher<Float64MultiArray>::SharedPtr _stiffness_publisher;
    Float64MultiArray                               _stiffness_msg;

    rclcpp::Publisher<Float64>::SharedPtr _tank_state_publisher;
    rclcpp::Publisher<Float64>::SharedPtr _tank_power_publisher;
    rclcpp::Publisher<Float64>::SharedPtr _tank_energy_publisher;

    double                       _dt;
    rclcpp::TimerBase::SharedPtr _qp_timer;

    Vec6_t _Kmin_diag, _Kmax_diag, _Kdes_diag, _Kvar_diag;
    Vec6_t _hmin, _hmax, _hdes;
    Mat6_t _Q, _R;

    double _x_tank;
    double _dx_dt_tank;
    double _T_sigma_th;
    double _T_min;
    double _rho_pos;
    double _rho_ori;


    Vec6_t x_tilde() const;
    Vec6_t x_dot_tilde() const;

    Mat6_t Kmin() const;
    Mat6_t Kmax() const;
    Mat6_t Kvar() const;
    Mat6_t Damp() const;
    Vec6_t Fdes() const;


    void   on_tank_update();
    double tank_energy() const;

    void declare_node_parameters();
    void initialise_node_parameters();

    void log_parameters();
};

}  // namespace qp_ros2


#endif  // QPROS2_CARTESIAN_ENERGY_TANK_HPP
