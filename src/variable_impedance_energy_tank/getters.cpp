#include <cmath>

#include "qp_ros2/type_aliases.hpp"
#include "qp_ros2/variable_impedance_energy_tank.hpp"

using qp_ros2::Mat6_t;
using qp_ros2::VariableImpedanceWithEnergyTank;
using qp_ros2::Vec6_t;
using std_msgs::msg::Float64MultiArray;
using RealVec_t = std::vector<double>;

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
