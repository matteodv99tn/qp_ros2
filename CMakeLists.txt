cmake_minimum_required(VERSION 3.8)
project(qp_ros2)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(QPROS2_USE_CLANGTIDY ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


set(CMAKE_CXX_STANDARD 20)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

#  ____                            _                 _
# |  _ \  ___ _ __   ___ _ __   __| | ___ _ __   ___(_) ___  ___
# | | | |/ _ \ '_ \ / _ \ '_ \ / _` |/ _ \ '_ \ / __| |/ _ \/ __|
# | |_| |  __/ |_) |  __/ | | | (_| |  __/ | | | (__| |  __/\__ \
# |____/ \___| .__/ \___|_| |_|\__,_|\___|_| |_|\___|_|\___||___/
#            |_|

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

# Builds qpOASES from source and makes available the target qpoases::qpoases
include(cmake/qpoases.cmake)

set(ROS2_REQUIRED_COMPONENTS
    rclcpp
    tf2_ros
    geometry_msgs
    std_msgs
)
foreach(COMPONENT ${ROS2_REQUIRED_COMPONENTS})
    find_package(${COMPONENT} REQUIRED)
endforeach()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
)

add_library(qp_wrapper INTERFACE)
target_include_directories(
    qp_wrapper
    INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/include
)
target_link_libraries(
    qp_wrapper
    INTERFACE
    qpoases::qpoases
    Eigen3::Eigen
)

add_library(
    qp_helpers
    src/helpers/msgs_conversions.cpp
    src/helpers/listeners_transformation.cpp
    src/helpers/listeners_posestamped.cpp
    src/helpers/listeners_twiststamped.cpp
    src/helpers/listeners_wrenchstamped.cpp
)
target_include_directories(
    qp_helpers
    PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/include
)
ament_target_dependencies(
    qp_helpers
    PUBLIC
    rclcpp
    geometry_msgs
    tf2_ros
)
target_link_libraries(
    qp_helpers
    PUBLIC
    Eigen3::Eigen
)

add_executable(
    variable_impedance_energy_tank
    src/variable_impedance_energy_tank/main.cpp
    src/variable_impedance_energy_tank/algorithm.cpp
    src/variable_impedance_energy_tank/parameters.cpp
    src/variable_impedance_energy_tank/getters.cpp
)
target_link_libraries(
    variable_impedance_energy_tank
    qp_wrapper
    qp_helpers
)
ament_target_dependencies(
    variable_impedance_energy_tank
    rclcpp
    geometry_msgs
    std_msgs
)

install(
    TARGETS variable_impedance_energy_tank
    DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(${ROS2_REQUIRED_COMPONENTS})
ament_package()
