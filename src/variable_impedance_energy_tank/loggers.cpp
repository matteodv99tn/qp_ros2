#include "qp_ros2/variable_impedance_energy_tank.hpp"

using qp_ros2::VariableImpedanceWithEnergyTank;

void
VariableImpedanceWithEnergyTank::log_parameters() {
    assert(_stiffness_publisher != nullptr);
    assert(_tank_state_publisher != nullptr);
    assert(_tank_power_publisher != nullptr);
    assert(_tank_energy_publisher != nullptr);

    for (long i = 0; i < 6; ++i) _stiffness_msg.data[i] = _Kmin_diag[i] + _Kvar_diag[i];
    _stiffness_publisher->publish(_stiffness_msg);

    Float64 state_msg;
    Float64 power_msg;
    Float64 energy_msg;

    state_msg.data  = _x_tank;
    power_msg.data  = _dx_dt_tank;
    energy_msg.data = tank_energy();

    _tank_state_publisher->publish(state_msg);
    _tank_power_publisher->publish(power_msg);
    _tank_energy_publisher->publish(energy_msg);
}
