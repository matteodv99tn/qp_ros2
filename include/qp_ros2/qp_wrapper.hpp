#ifndef QPROS2_QPOASES_EIGEN_WRAPPER_HPP
#define QPROS2_QPOASES_EIGEN_WRAPPER_HPP

#include <Eigen/Dense>
#include <optional>

#include "qpOASES/MessageHandling.hpp"
#include "qpOASES/SQProblem.hpp"
#include "qpOASES/Types.hpp"

namespace qp_ros2 {

/**
 * @brief Wrapper with Eigen matrices/vectors for the qpOASES solver. As safety measure,
 * this problem supposes compile-time knowledge of the QP problem.
 *
 * @tparam N_VARIABLES      number of variables of the QP problem
 * @tparam N_CONSTRAINTS    number of constraints in the QP problem
 */
template <int N_VARIABLES, int N_CONSTRAINTS>
class QpWrapper {
private:
    using real_t = qpOASES::real_t;

public:
    using qpH_t = Eigen::Matrix<real_t, N_VARIABLES, N_VARIABLES, Eigen::RowMajor>;
    using qpA_t = Eigen::Matrix<real_t, N_CONSTRAINTS, N_VARIABLES, Eigen::RowMajor>;
    using qpx_t = Eigen::Vector<real_t, N_VARIABLES>;
    using qpc_t = Eigen::Vector<real_t, N_CONSTRAINTS>;
    using qpg_t = qpx_t;
    using qpxBounds_t = qpx_t;
    using qpABounds_t = qpc_t;

    QpWrapper() :
            H(decltype(H)::Zero()),
            x(decltype(x)::Zero()),
            A(decltype(A)::Zero()),
            g(decltype(g)::Zero()),
            x_lb(decltype(x_lb)::Zero()),
            x_ub(decltype(x_ub)::Zero()),
            A_lb(decltype(A_lb)::Zero()),
            A_ub(decltype(A_ub)::Zero()),
            _nv(N_VARIABLES),
            _nc(N_CONSTRAINTS),
            _qp_problem(_nv, _nc),
            _last_cpu_time(0),
            _max_cpu_time(1.0) {}

    int
    n_variables() const {
        return _nv;
    }

    int
    n_constraints() const {
        return _nc;
    }

    qpOASES::returnValue
    solve() {
        qpOASES::returnValue ret;
        _n_WSR         = 10;
        _last_cpu_time = _max_cpu_time;

        if (_is_first_iteration) [[unlikely]] {
            ret = _qp_problem.init(
                    H.data(),
                    g.data(),
                    A.data(),
                    x_lb.data(),
                    x_ub.data(),
                    A_lb.data(),
                    A_ub.data(),
                    _n_WSR,
                    &_last_cpu_time
            );
            _is_first_iteration = false;
        } else [[likely]] {
            ret = _qp_problem.hotstart(
                    H.data(),
                    g.data(),
                    A.data(),
                    x_lb.data(),
                    x_ub.data(),
                    A_lb.data(),
                    A_ub.data(),
                    _n_WSR,
                    &_last_cpu_time
            );
        }

        _last_retval = ret;
        return ret;
    }

    real_t
    get_last_execution_time() const {
        return _last_cpu_time;
    }

    std::optional<const qpx_t>
    get_last_solution() const {
        if (_last_retval == qpOASES::SUCCESSFUL_RETURN) {
            _qp_problem.getPrimalSolution(x.data());
            return x;
        } else return std::nullopt;
    }

    qpH_t         H;
    mutable qpx_t x;
    qpA_t         A;
    qpg_t         g;
    qpxBounds_t   x_lb, x_ub;
    qpABounds_t   A_lb, A_ub;

private:
    int _nv;
    int _nc;

    qpOASES::SQProblem   _qp_problem;
    bool                 _is_first_iteration{true};
    int                  _n_WSR;
    real_t               _last_cpu_time;
    real_t               _max_cpu_time;
    qpOASES::returnValue _last_retval;
};
}  // namespace qp_ros2

#endif  // QPROS2_QPOASES_EIGEN_WRAPPER_HPP
