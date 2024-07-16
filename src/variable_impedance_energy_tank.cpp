/*
 * MISSING
 * - Desired stiffness from topic
 */
#include "qp_ros2/variable_impedance_energy_tank.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <qpOASES/Types.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "qp_ros2/helpers/listeners.hpp"
#include "qp_ros2/type_aliases.hpp"
#include "qpOASES/MessageHandling.hpp"
#include "qpOASES/Utils.hpp"

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

std::string
describe(const RealVec_t& vec) {
    std::string res;
    for (const double& v : vec) res += std::to_string(v) + " ";
    return res;
}

VariableImpedanceWithEnergyTank::VariableImpedanceWithEnergyTank() :
        rclcpp::Node("cartesian_energy_tank"),
        _desired_pos(nullptr),
        _current_pos(nullptr),
        _desired_vel(nullptr),
        _current_vel(nullptr),
        _desired_wrench(nullptr),
        _all_messages_received(false),
        _tf_buffer(nullptr),
        _tf_listener(nullptr),
        _stiffness_publisher(nullptr),
        _stiffness_msg(),
        _dt(0.0),
        _Kmin_diag(Vec6_t::Zero()),
        _Kmax_diag(Vec6_t::Zero()),
        _Kdes_diag(Vec6_t::Zero()),
        _Kvar_diag(Vec6_t::Zero()),
        _hmin(Vec6_t::Zero()),
        _hmax(Vec6_t::Zero()),
        _hdes(Vec6_t::Zero()),
        _Q(Mat6_t::Zero()),
        _R(Mat6_t::Zero()),
        _x_tank(0),
        _T_sigma_th(0),
        _T_min(0.0),
        _rho_pos(0.0),
        _rho_ori(0.0) {
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
    long dt_as_ns = static_cast<long>(dt * 1e9);
    _qp_timer     = create_wall_timer(std::chrono::nanoseconds(dt_as_ns), [this]() {
        on_tank_update();
    });

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

    // Set constants QP variables
    _qp.x_lb = Vec6_t::Zero();
    _qp.x_ub = _Kmax_diag - _Kmin_diag;
}

void
VariableImpedanceWithEnergyTank::on_tank_update() {
    if (!_all_messages_received) {
        _all_messages_received =
                _desired_pos->data_received() && _current_pos->data_received()
                && _desired_vel->data_received() && _current_vel->data_received();
        RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                1e3,
                "Not all data has been received, not updating the tank..."
        );
        return;
    }
    RCLCPP_INFO_ONCE(get_logger(), "Started variable impedance control");

    // Retrieve main parameters
    const Vec6_t x_tilde  = this->x_tilde();
    const Vec6_t xd_tilde = this->x_dot_tilde();
    const Mat6_t X_tilde  = x_tilde.asDiagonal();
    const Mat6_t Xd_tilde = xd_tilde.asDiagonal();

    const Mat6_t D = Damp();
    const Vec6_t e = D * xd_tilde + X_tilde * _Kmin_diag;
    const Vec6_t d = _hdes - e;

    const double T     = tank_energy();
    const double sigma = (T <= _T_sigma_th) ? 1.0 : 0.0;

    // Set optimisation variables
    _qp.H = X_tilde * _Q * X_tilde + _R;
    _qp.g = -X_tilde * _Q * d - _R * (_Kdes_diag - _Kmax_diag);

    _qp.A_lb.head<6>()          = _hmin - e;
    _qp.A_ub.head<6>()          = _hmax - e;
    _qp.A.topLeftCorner<6, 6>() = X_tilde;

    _qp.A.row(6) = -x_tilde.transpose() * Xd_tilde;
    _qp.A_lb(6)  = -1e9;  // unbounded
    _qp.A_ub(6)  = sigma * xd_tilde.transpose() * D * xd_tilde + (T - _T_min) / _dt;

    const auto px_tilde  = x_tilde.head<3>();
    const auto pXd_tilde = Xd_tilde.topLeftCorner<3, 3>();
    const auto pxd_tilde = xd_tilde.head<3>();
    const auto pD        = D.topLeftCorner<3, 3>();

    _qp.A.row(7).head<3>() = -px_tilde.transpose() * pXd_tilde;
    _qp.A_lb(7)            = -1e9;  // unbounded
    _qp.A_ub(7)            = sigma * pxd_tilde.transpose() * pD * pxd_tilde + _rho_pos;

    const auto qx_tilde  = x_tilde.tail<3>();
    const auto qXd_tilde = Xd_tilde.bottomRightCorner<3, 3>();
    const auto qxd_tilde = xd_tilde.tail<3>();
    const auto qD        = D.bottomRightCorner<3, 3>();

    _qp.A.row(8).head<3>() = -qx_tilde.transpose() * qXd_tilde;
    _qp.A_lb(8)            = -1e9;  // unbounded
    _qp.A_ub(8)            = sigma * qxd_tilde.transpose() * qD * qxd_tilde + _rho_ori;

    // Solve the problem
    qpOASES::returnValue ret                = _qp.solve();
    qpOASES::int_t       qp_solution_status = qpOASES::getSimpleStatus(ret);

    if (qp_solution_status == qpOASES::SUCCESSFUL_RETURN) {
        _Kvar_diag = _qp.get_last_solution().value();
    } else {
        _Kvar_diag.setZero();
        RCLCPP_WARN(
                get_logger(),
                "Unable to find solution to the QP problem. Return code: %d",
                ret
        );
    }

    const Vec6_t w         = (T < _T_min) ? Vec6_t(-Kvar() * x_tilde) : Vec6_t::Zero();
    const double dx_tank_1 = sigma * xd_tilde.transpose() * Damp() * xd_tilde;
    const double dx_tank_2 = -w.transpose() * xd_tilde;
    const double dx_tank   = (dx_tank_1 + dx_tank_2) / _x_tank;
    _x_tank += dx_tank * _dt;

    assert(_stiffness_publisher != nullptr);
    for (long i = 0; i < 6; ++i) _stiffness_msg.data[i] = _Kmin_diag[i] + _Kvar_diag[i];
    _stiffness_publisher->publish(_stiffness_msg);
}

Vec6_t
VariableImpedanceWithEnergyTank::x_tilde() const {
    Vec6_t res;
    res.head<3>() = _desired_pos->last_position() - _current_pos->last_position();

    const Quaternion_t qdiff = _current_pos->last_orientation()
                               * _desired_pos->last_orientation().conjugate();
    const Vec3_t& u = qdiff.vec();
    if (u.norm() < 1e-5) res.tail<3>() = Vec3_t::Zero();
    else res.tail<3>() = 2 * std::acos(qdiff.w()) * u.normalized();

    return res;
}

Vec6_t
VariableImpedanceWithEnergyTank::x_dot_tilde() const {
    Vec6_t res;
    res.head<3>() =
            _desired_vel->last_linear_velocity() - _current_vel->last_linear_velocity();
    res.tail<3>() = _desired_vel->last_angular_velocity()
                    - _current_vel->last_angular_velocity();
    return res;
}

Mat6_t
VariableImpedanceWithEnergyTank::Kmin() const {
    return _Kmin_diag.asDiagonal();
}

Mat6_t
VariableImpedanceWithEnergyTank::Kvar() const {
    return _Kvar_diag.asDiagonal();
}

Mat6_t
VariableImpedanceWithEnergyTank::Damp() const {
    return 2.0 * _Kvar_diag.cwiseSqrt().asDiagonal();
}

Vec6_t
VariableImpedanceWithEnergyTank::Fdes() const {
    Vec6_t res;
    res.head<3>() = _desired_wrench->last_force();
    res.tail<3>() = _desired_wrench->last_torque();
    return res;
}

double
VariableImpedanceWithEnergyTank::tank_energy() const {
    return 0.5 * _x_tank * _x_tank;
}

int
main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VariableImpedanceWithEnergyTank>());
    rclcpp::shutdown();
    return 0;
}
