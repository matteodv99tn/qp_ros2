/*
 * MISSING
 * - Desired stiffness from topic
 */
#include <chrono>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <qpOASES/Types.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "qp_ros2/helpers/listeners.hpp"
#include "qp_ros2/type_aliases.hpp"
#include "qp_ros2/variable_impedance_energy_tank.hpp"
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
    declare_node_parameters();
    initialise_node_parameters();

    long dt_as_ns = static_cast<long>(_dt * 1e9);
    _qp_timer     = create_wall_timer(std::chrono::nanoseconds(dt_as_ns), [this]() {
        on_tank_update();
    });
    RCLCPP_INFO(get_logger(), "Starting tasks with period %.2fms", _dt * 1e3);

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
                3e3,
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
