/*
 * MISSING
 * - Desired stiffness from topic
 */
#include <algorithm>
#include <cmath>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <qpOASES/Types.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "qp_ros2/helpers/listeners.hpp"
#include "qp_ros2/type_aliases.hpp"
#include "qp_ros2/variable_impedance_energy_tank.hpp"

using qp_ros2::Mat6_t;
using qp_ros2::VariableImpedanceWithEnergyTank;
using qp_ros2::Vec6_t;
using std_msgs::msg::Float64MultiArray;
using RealVec_t = std::vector<double>;


static const RealVec_t def_kmin_coeffs{0.1, 0.1, 0.1, 0.01, 0.01, 0.01};
static const RealVec_t def_kmax_coeffs{1000.0, 1000.0, 1000.0, 100.0, 100.0, 100.0};
static const RealVec_t def_kdes_coeffs{700.0, 700.0, 700.0, 70.0, 70.0, 70.0};
static const RealVec_t def_hmin_coeffs{-25.0, -25.0, -25.0, -2.5, -2.5, -2.5};
static const RealVec_t def_hmax_coeffs{25.0, 25.0, 25.0, 2.5, 2.5, 2.5};
static const RealVec_t def_hdes_coeffs{0.0, 0.0, 10.0, 0.0, 0.0, 0.0};
static const RealVec_t def_r_coeffs{0.01, 0.01, 0.01, 0.01, 0.01, 0.01};
static const RealVec_t def_q_coeffs{3200.0, 3200.0, 3200.0, 3200.0, 3200.0, 3200.0};

static std::string
describe(const RealVec_t& vec) {
    std::string res;
    for (const double& v : vec) res += std::to_string(v) + " ";
    return res;
}

void
VariableImpedanceWithEnergyTank::declare_node_parameters() {
    // Paramater declaration
    declare_parameter("compliance_frame", "end_effector");
    declare_parameter("sampling_period", 0.002);
    declare_parameter("desired_pose_topic", "/controller/desired_pose");
    declare_parameter("current_pose_topic", "/controller/current_pose");
    declare_parameter("desired_velocity_topic", "/controller/desired_velocity");
    declare_parameter("current_velocity_topic", "/controller/current_velocity");
    declare_parameter("desired_wrench_topic", "/controller/desired_wrench");
    declare_parameter("stiffness_output", "/controller/stiffness");

    declare_parameter("minimum_stiffness", def_kmin_coeffs);
    declare_parameter("maximum_stiffness", def_kmax_coeffs);
    declare_parameter("desired_stiffness", def_kdes_coeffs);
    declare_parameter("minimum_force", def_hmin_coeffs);
    declare_parameter("maximum_force", def_hmax_coeffs);
    declare_parameter("desired_force", def_hdes_coeffs);
    declare_parameter("Qmatrix_coefficients", def_q_coeffs);
    declare_parameter("Rmatrix_coefficients", def_r_coeffs);
    declare_parameter("tank_initial_state", 2.0);
    declare_parameter("tank_sigma_energy_threshold", 3.0);
    declare_parameter("tank_minimum_energy", 0.5);
    declare_parameter("tank_maximum_position_power_outflow", -0.5);
    declare_parameter("tank_maximum_orientation_power_outflow", -0.5);
}

void
VariableImpedanceWithEnergyTank::initialise_node_parameters() {
    _tf_buffer   = std::make_shared<tf2_ros::Buffer>(get_clock());
    _tf_listener = std::make_unique<tf2_ros::TransformListener>(*_tf_buffer, this);

    // Parameter retrieval
    const std::string compliance_frame = get_parameter("compliance_frame").as_string();
    const std::string des_pos_topic  = get_parameter("desired_pose_topic").as_string();
    const std::string curr_pos_topic = get_parameter("current_pose_topic").as_string();
    const std::string des_vel_topic =
            get_parameter("desired_velocity_topic").as_string();
    const std::string curr_vel_topic =
            get_parameter("current_velocity_topic").as_string();
    const std::string des_wrench_topic =
            get_parameter("desired_wrench_topic").as_string();
    const std::string stiff_out_topic = get_parameter("stiffness_output").as_string();
    const double      x0_tank         = get_parameter("tank_initial_state").as_double();
    const double sigma_th = get_parameter("tank_sigma_energy_threshold").as_double();
    const double Tmin     = get_parameter("tank_minimum_energy").as_double();
    const double rho_pos =
            get_parameter("tank_maximum_position_power_outflow").as_double();
    const double rho_ori =
            get_parameter("tank_maximum_orientation_power_outflow").as_double();

    double    dt          = get_parameter("sampling_period").as_double();
    RealVec_t kmin_coeffs = get_parameter("minimum_stiffness").as_double_array();
    RealVec_t kmax_coeffs = get_parameter("maximum_stiffness").as_double_array();
    RealVec_t kdes_coeffs = get_parameter("desired_stiffness").as_double_array();
    RealVec_t hmin_coeffs = get_parameter("minimum_force").as_double_array();
    RealVec_t hmax_coeffs = get_parameter("maximum_force").as_double_array();
    RealVec_t hdes_coeffs = get_parameter("desired_force").as_double_array();
    RealVec_t Q_coeff     = get_parameter("Qmatrix_coefficients").as_double_array();
    RealVec_t R_coeff     = get_parameter("Rmatrix_coefficients").as_double_array();

    RCLCPP_INFO(
            get_logger(),
            "Creating energy tank with compliance frame '%s'",
            compliance_frame.c_str()
    );

    RCLCPP_INFO(
            get_logger(), "Listening desired pose on topic %s", des_pos_topic.c_str()
    );
    _desired_pos =
            std::make_unique<PoseStampedListener>(this, _tf_buffer, des_pos_topic);
    RCLCPP_INFO(
            get_logger(), "Listening current pose on topic %s", curr_pos_topic.c_str()
    );
    _current_pos =
            std::make_unique<PoseStampedListener>(this, _tf_buffer, curr_pos_topic);

    RCLCPP_INFO(
            get_logger(), "Listening desired twist on topic %s", des_vel_topic.c_str()
    );
    _desired_vel =
            std::make_unique<TwistStampedListener>(this, _tf_buffer, des_vel_topic);
    RCLCPP_INFO(
            get_logger(), "Listening current twist on topic %s", curr_vel_topic.c_str()
    );
    _current_vel =
            std::make_unique<TwistStampedListener>(this, _tf_buffer, curr_vel_topic);
    RCLCPP_INFO(
            get_logger(),
            "Listening desired wrench on topic %s",
            des_wrench_topic.c_str()
    );
    _desired_wrench =
            std::make_unique<WrenchStampedListener>(this, _tf_buffer, des_wrench_topic);

    _desired_pos->set_desired_reference_frame(compliance_frame);
    _current_pos->set_desired_reference_frame(compliance_frame);
    _desired_vel->set_desired_reference_frame(compliance_frame);
    _current_vel->set_desired_reference_frame(compliance_frame);
    _desired_wrench->set_desired_reference_frame(compliance_frame);

    _stiffness_publisher = create_publisher<Float64MultiArray>(
            stiff_out_topic, rclcpp::QoS(1).keep_last(1)
    );
    RCLCPP_INFO(
            get_logger(),
            "Writing computed stiffness on topic %s",
            stiff_out_topic.c_str()
    );
    _stiffness_msg.data.resize(6);

    if (dt <= 0) {
        RCLCPP_WARN(
                get_logger(), "Set sampling period non-positive, using default value..."
        );
        dt = 0.002;
    }
    _dt = dt;
    RCLCPP_INFO(get_logger(), "Sampling period: %fms", dt * 1e3);

    if (kmin_coeffs.size() != 6) {
        RCLCPP_WARN(
                get_logger(),
                "Provided size for minimum stiffness coefficient is %zu, expected 6. "
                "Switching to default values",
                kmin_coeffs.size()
        );
        kmin_coeffs = def_kmin_coeffs;
    }
    RCLCPP_INFO(
            get_logger(),
            "Minimum stiffness coefficients: %s",
            describe(kmin_coeffs).c_str()
    );
    for (long i = 0; i < 6; ++i) _Kmin_diag(i) = kmin_coeffs[i];

    if (kmax_coeffs.size() != 6) {
        RCLCPP_WARN(
                get_logger(),
                "Provided size for maximum stiffness coefficient is %zu, expected 6. "
                "Switching to default values",
                kmax_coeffs.size()
        );
        kmax_coeffs = def_kmax_coeffs;
    }
    RCLCPP_INFO(
            get_logger(),
            "Maximum stiffness coefficients: %s",
            describe(kmax_coeffs).c_str()
    );
    for (long i = 0; i < 6; ++i) _Kmax_diag(i) = kmin_coeffs[i];

    if (kdes_coeffs.size() != 6) {
        RCLCPP_WARN(
                get_logger(),
                "Provided size for desired stiffness coefficient is %zu, expected 6. "
                "Switching to default values",
                kdes_coeffs.size()
        );
        kdes_coeffs = def_kdes_coeffs;
    }
    RCLCPP_INFO(
            get_logger(),
            "Desired stiffness coefficients: %s",
            describe(kdes_coeffs).c_str()
    );
    for (long i = 0; i < 6; ++i) _Kdes_diag(i) = kmin_coeffs[i];

    if (hmin_coeffs.size() != 6) {
        RCLCPP_WARN(
                get_logger(),
                "Provided minimum force vector has %zu coefficients, expected 6. "
                "Switching to default values",
                hmin_coeffs.size()
        );
        hmin_coeffs = def_hmin_coeffs;
    }
    RCLCPP_INFO(
            get_logger(), "Minimum force vector: %s", describe(hmin_coeffs).c_str()
    );
    for (long i = 0; i < 6; ++i) _hmin(i, i) = hmin_coeffs[i];

    if (hmax_coeffs.size() != 6) {
        RCLCPP_WARN(
                get_logger(),
                "Provided maximum force vector has %zu coefficients, expected 6. "
                "Switching to default values",
                hmax_coeffs.size()
        );
        hmax_coeffs = def_hmax_coeffs;
    }
    RCLCPP_INFO(
            get_logger(), "Maximum force vector: %s", describe(hmax_coeffs).c_str()
    );
    for (long i = 0; i < 6; ++i) _hmax(i, i) = hmax_coeffs[i];

    if (hdes_coeffs.size() != 6) {
        RCLCPP_WARN(
                get_logger(),
                "Provided desired force vector has %zu coefficients, expected 6. "
                "Switching to default values",
                hdes_coeffs.size()
        );
        hdes_coeffs = def_hdes_coeffs;
    }
    RCLCPP_INFO(
            get_logger(), "Desired force vector: %s", describe(hdes_coeffs).c_str()
    );
    for (long i = 0; i < 6; ++i) _hdes(i, i) = hdes_coeffs[i];


    if (Q_coeff.size() != 6) {
        RCLCPP_WARN(
                get_logger(),
                "Provided coefficients of Q are %zu, expected 6. Switching to default "
                "values",
                Q_coeff.size()
        );
        Q_coeff = def_q_coeffs;
    }
    RCLCPP_INFO(get_logger(), "Coefficients of Q: %s", describe(Q_coeff).c_str());
    for (long i = 0; i < 6; ++i) _Q(i, i) = Q_coeff[i];

    if (R_coeff.size() != 6) {
        RCLCPP_WARN(
                get_logger(),
                "Provided coefficients of R are %zu, expected 6. Switching to default "
                "values",
                R_coeff.size()
        );
        R_coeff = def_r_coeffs;
    }
    RCLCPP_INFO(get_logger(), "Coefficients of R: %s", describe(R_coeff).c_str());
    for (long i = 0; i < 6; ++i) _Q(i, i) = Q_coeff[i];


    _x_tank = std::max(x0_tank, -x0_tank);
    RCLCPP_INFO(
            get_logger(), "Tank initial state: %f (energy: %f)", _x_tank, tank_energy()
    );

    _T_min = std::max(Tmin, -Tmin);
    RCLCPP_INFO(get_logger(), "Tank minimum allowed energy: %f", _T_min);

    _T_sigma_th = std::abs(sigma_th);
    RCLCPP_INFO(get_logger(), "Energy threshold to disable sigma: %f", _T_sigma_th);

    _rho_pos = rho_pos;
    RCLCPP_INFO(get_logger(), "Maximum power outflow from positon part: %f", _rho_pos);

    _rho_ori = rho_ori;
    RCLCPP_INFO(
            get_logger(), "Maximum power outflow from orietnation part: %f", _rho_ori
    );
}
