#ifndef QPROS2_TYPE_ALIASES_HPP
#define QPROS2_TYPE_ALIASES_HPP

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace qp_ros2 {

template <int N>
using VecN_t = Eigen::Vector<double, N>;

template <int N>
using MatN_t = Eigen::Matrix<double, N, N>;

using Vec3_t = VecN_t<3>;
using Vec6_t = VecN_t<6>;

using Quaternion_t = Eigen::Quaterniond;


using Mat3_t = MatN_t<3>;
using Mat6_t = MatN_t<6>;

using RotMatrix_t = Mat3_t;

}  // namespace  qp_ros2


#endif  // QPROS2_TYPE_ALIASES_HPP
