/*
 * MISSING
 * - Desired stiffness from topic
 */
#include <rclcpp/rclcpp.hpp>

#include "qp_ros2/variable_impedance_energy_tank.hpp"

using qp_ros2::VariableImpedanceWithEnergyTank;

int
main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VariableImpedanceWithEnergyTank>());
    rclcpp::shutdown();
    return 0;
}
