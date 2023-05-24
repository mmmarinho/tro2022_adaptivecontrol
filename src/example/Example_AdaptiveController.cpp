/**
(C) Copyright 2020-2023 Murilo Marques Marinho (www.murilomarinho.info)

This file is part of adaptive_control_example.

    DQ Robotics is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    adaptive_control_example is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with adaptive_control_example.  If not, see <http://www.gnu.org/licenses/>.

Author:
    Murilo M. Marinho (murilomarinho@ieee.org)

Contributors (aside from author):
    None
*/
/**
 * Example code for:
 * M. M. Marinho and B. V. Adorno,
 * "Adaptive Constrained Kinematic Control Using Partial or Complete Task-Space Measurements,"
 * in IEEE Transactions on Robotics, vol. 38, no. 6, pp. 3498-3513, Dec. 2022,
 * doi: 10.1109/TRO.2022.3181047.
 */
#include <dqrobotics/utils/DQ_Math.h>

#include "example/Example_AdaptiveController.h"


std::tuple<MatrixXd, VectorXd> get_variable_boundary_inequalities(const VectorXd& q, const std::tuple<VectorXd, VectorXd>& boundaries, const VectorXd& damping_matrix_diagonal=VectorXd())
{
    VectorXd q_min, q_max;
    std::tie(q_min, q_max) = boundaries;

    const int n = q.size();

    MatrixXd W(2*n, n);
    W << -MatrixXd::Identity(n, n),MatrixXd::Identity(n, n);

    VectorXd w(2*n);

    if(damping_matrix_diagonal.size()!=0)
        w << -damping_matrix_diagonal.cwiseProduct(q_min-q), damping_matrix_diagonal.cwiseProduct(q_max-q);
    else
        w << -(q_min-q), (q_max-q);

    return {W, w};
}

std::tuple<VectorXd,double> __closest_invariant_error(const DQ& x, const DQ& xd, const Example_MeasureSpace& measure_space)
{
    switch(measure_space)
    {
    case Example_MeasureSpace::None:
        throw std::runtime_error("None is not a valid DQ_MeasureSpace");
    case Example_MeasureSpace::Distance:
    {
        //There is no double cover in distance
        VectorXd e_d(1);
        e_d << (static_cast<double>(x)-static_cast<double>(xd));
        return {e_d, 1};
    }
    case Example_MeasureSpace::Rotation:
    {
        //Address double cover in rotation space
        double er_1_norm       = vec4(conj(x)*xd - 1).norm();
        double er_1minus_norm  = vec4(conj(x)*xd + 1).norm();
        DQ er;
        double invariant;
        if(er_1_norm<er_1minus_norm)
        {
            er = conj(x)*xd - 1;
            invariant = -1;
        }
        else
        {
            er = conj(x)*xd + 1;
            invariant = +1;
        }
        return {vec4(er),invariant};
    }
    case Example_MeasureSpace::Translation:
    {
        //There is no double cover in translation
        return {vec4(x-xd),1};
    }
    case Example_MeasureSpace::Pose:
        //Address double cover in pose space
        const double ex_1_norm       = vec8(conj(x)*xd - 1).norm();
        const double ex_1minus_norm  = vec8(conj(x)*xd + 1).norm();
        DQ ex;
        double invariant;
        if(ex_1_norm<ex_1minus_norm)
        {
            ex = conj(x)*xd - 1;
            invariant = -1;
        }
        else
        {
            ex = conj(x)*xd + 1;
            invariant = +1;
        }
        return {vec8(ex),invariant};
    }
    throw std::runtime_error("Not supposed to be reachable");
}

/**
 * @brief compute_setpoint_control_signal One possible implementation of Algorithm 1 of
 * M. M. Marinho and B. V. Adorno,
 * "Adaptive Constrained Kinematic Control Using Partial or Complete Task-Space Measurements,"
 * in IEEE Transactions on Robotics, vol. 38, no. 6, pp. 3498-3513, Dec. 2022,
 * doi: 10.1109/TRO.2022.3181047.
 *
 * @param control_strategy See Example_AdaptiveControlStrategy for possible values.
 * @param q the current configuration of the robot.
 * @param xd the desired task-space value.
 * @param y the current measurement.
 * @param vfis the vector of VFIs.
 * @return {uq, ua, x_tilde, y_tilde, y_partial} in which
 * uq: is the control signal, to be applied at the robot's configurations
 * ua: the adaptation signal, to be applied on the robot's parameters,
 * x_tilde: the task error,
 * y_tilde: the measurement error,
 * y_partial: the measurement error in the partial space when the measurements are not pose.
 */
std::tuple<VectorXd, VectorXd, VectorXd, VectorXd, DQ> Example_AdaptiveController::compute_setpoint_control_signal(const Example_AdaptiveControlStrategy& control_strategy,
                                                                                       const VectorXd&q,
                                                                                       const DQ& xd,
                                                                                       const DQ& y,
                                                                                       std::vector<Example_VFI>& vfis)
{
    const int n = robot_->get_dim_configuration_space();
    const int p = robot_->get_dim_parameter_space();
    const double& eta_task = simulation_arguments_.proportional_gain;
    const double& eta_parameter = simulation_arguments_.proportional_gain;
    const double& vfi_gain = simulation_arguments_.vfi_gain;
    const double& vfi_weight = simulation_arguments_.vfi_weight;
    const double& lambda = simulation_arguments_.damping;
    const Example_MeasureSpace& measure_space = simulation_arguments_.measure_space;

    const double& MAX_ACCEPTABLE_CONSTRAINT_PENETRATION = 0.001;
    const double& MAX_ACCEPTABLE_CONSTRAINT_PENETRATION_SQUARED = MAX_ACCEPTABLE_CONSTRAINT_PENETRATION*MAX_ACCEPTABLE_CONSTRAINT_PENETRATION;

    VectorXd uq = VectorXd::Zero(n);
    ///Estimated measurement (used in both control strategies)
    const DQ x_hat = robot_->fkm(q);
    double x_invariant;
    VectorXd x_tilde;
    std::tie(x_tilde, x_invariant) = __closest_invariant_error(x_hat, xd, Example_MeasureSpace::Pose);
    ///VFI state that is independent of control strategy
    const int& vfis_size = static_cast<int>(vfis.size());
    VectorXd w_vfi(vfis_size);
    for(int i=0;i<vfis_size;i++)
    {
        Example_VFI& vfi = vfis[i];
        w_vfi(i) = vfi.get_distance_error(x_hat);
        switch(vfi.get_distance_type())
        {
        case Example_VFI_DistanceType::None:
            throw std::runtime_error("Expected valid value");
        case Example_VFI_DistanceType::EUCLIDEAN:
        {
            if(w_vfi(i) < -MAX_ACCEPTABLE_CONSTRAINT_PENETRATION)
            {
                std::cout << vfi.get_vfi_name() << " estimated penetration: " << w_vfi(i) << std::endl;
                //throw std::runtime_error("Distance to obstacle point over threshold " + std::to_string(w_vfi(i)));
            }
        }
        case Example_VFI_DistanceType::EUCLIDEAN_SQUARED:
        {
            if(w_vfi(i) < -MAX_ACCEPTABLE_CONSTRAINT_PENETRATION_SQUARED)
            {
                std::cout << vfi.get_vfi_name() << " estimated penetration: " << sqrt(fabs(w_vfi(i))) << std::endl;
                //throw std::runtime_error("Distance to obstacle point over threshold " + std::to_string(w_vfi(i)));
            }
        }
        }

        //Store information
        vfi.set_last_estimated_distance(x_hat);
        if(y != DQ(0))
            vfi.set_last_real_distance(y);
    }

    if(control_strategy == Example_AdaptiveControlStrategy::FULL || control_strategy == Example_AdaptiveControlStrategy::TASK_ONLY)
    {
        ///Task
        const MatrixXd J_x_q = robot_->pose_jacobian(q);
        const MatrixXd N_x_q = haminus8(xd)*C8()*robot_->pose_jacobian(q);

        const MatrixXd Hx = (N_x_q.transpose()*N_x_q + lambda*MatrixXd::Identity(n,n));
        const VectorXd fx = 2.*N_x_q.transpose()*eta_task*x_tilde;

        //I*q <= q_dot_upper
        //I*q >= q_dot_lower <=> -I*q <= -q_dot_lower
        MatrixXd W_q_dot_limits(2*n, n); W_q_dot_limits << MatrixXd::Identity(n,n), -1.0*MatrixXd::Identity(n,n);
        VectorXd w_q_dot_limits(2*n);    w_q_dot_limits << robot_->get_upper_q_dot_limit(), -1.0*robot_->get_lower_q_dot_limit();

        const int& vfis_size = static_cast<int>(vfis.size());

        ///Inequality constraint for VFIs
        MatrixXd W_vfi_q(vfis_size, n);
        for(int i=0;i<vfis_size;i++)
        {
            Example_VFI& vfi = vfis[i];
            W_vfi_q.row(i) = vfi.get_vfi_matrix(x_hat, J_x_q);
        }

        MatrixXd W_q_limits;
        VectorXd w_q_limits;
        std::tie(W_q_limits, w_q_limits) = get_variable_boundary_inequalities(q,
                                                                              {robot_->get_lower_q_limit(),
                                                                               robot_->get_upper_q_limit()});

        MatrixXd W_q(4*n+W_vfi_q.rows(),n);W_q << W_q_dot_limits, W_q_limits, W_vfi_q;
        VectorXd w_q(4*n+w_vfi.size()); w_q << w_q_dot_limits, w_q_limits, vfi_gain*vfi_weight*w_vfi;

        uq = task_space_solver_.solve_quadratic_program(2.0*Hx, fx, W_q, w_q, MatrixXd::Zero(0,0), VectorXd(0));
    }


    VectorXd ua = VectorXd::Zero(p);
    VectorXd y_tilde;
    DQ y_partial;
    ///Adaptation
    if(y != DQ(0) && ( control_strategy == Example_AdaptiveControlStrategy::FULL || control_strategy == Example_AdaptiveControlStrategy::MEASUREMENT_ONLY ))
    {
        //Get a partial measurement if needed and the measurement error
        const DQ y_hat = x_hat;

        y_partial = _convert_pose_to_measure_space(y, measure_space);
        const DQ y_hat_partial = _convert_pose_to_measure_space(y_hat, measure_space);
        double y_invariant;
        std::tie(y_tilde, y_invariant) = __closest_invariant_error(y_hat_partial, y_partial, measure_space);

        const MatrixXd J_y_a = robot_->parameter_pose_jacobian(q);
        const MatrixXd J_y_a_partial = _convert_pose_jacobian_to_measure_space(J_y_a, y_hat, y, measure_space);

        ///Objective function
        const MatrixXd Hy = (J_y_a_partial.transpose()*J_y_a_partial + lambda*MatrixXd::Identity(p,p));
        const VectorXd fy = 2.*J_y_a_partial.transpose()*eta_parameter*y_tilde;

        ///Equality constraint
        const MatrixXd N_a = _get_complimentary_measure_space_jacobian(J_y_a, y_hat, measure_space);
        const VectorXd b_N_a = VectorXd::Zero(N_a.rows());

        ///Inequality constraint for Lyapunov stability
        const MatrixXd A_y = x_tilde.transpose()*haminus8(xd)*C8()*J_y_a;
        const VectorXd b_y = VectorXd::Zero(1);

        MatrixXd W_vfi_a(vfis_size, p);
        for(int i=0; i<vfis_size; i++)
        {
            const Example_VFI& vfi = vfis.at(i);
            W_vfi_a.row(i) = vfi.get_vfi_matrix(x_hat, J_y_a);
        }

        ///Inequality constraint for parameter limits
        const VectorXd a_hat = robot_->get_parameter_space_values();
        auto parameter_boundaries = robot_->get_parameter_space_boundaries();
        MatrixXd W_a_hat_limits;
        VectorXd w_a_hat_limits;
        std::tie(W_a_hat_limits, w_a_hat_limits) = get_variable_boundary_inequalities(a_hat, parameter_boundaries);

        ///Composition of inequality constraints
        MatrixXd W_a_hat;
        W_a_hat.resize(A_y.rows()+W_a_hat_limits.rows()+W_vfi_a.rows(),p); W_a_hat << A_y, W_a_hat_limits, W_vfi_a;
        VectorXd w_a_hat;
        w_a_hat.resize(b_y.size()+w_a_hat_limits.size()+w_vfi.size()); w_a_hat << b_y, w_a_hat_limits, vfi_gain*(1.0 - vfi_weight)*w_vfi;


        try
        {
            ua = parameter_space_solver_.solve_quadratic_program(2.0*Hy, fy, W_a_hat, w_a_hat, N_a, b_N_a);
        }
        catch(const std::exception& e)
        {
            std::cerr << "Error solving parametric quadratic program " << std::endl;
            std::cerr << "Hy size = (" << Hy.rows() << "," << Hy.cols() << ")." << std::endl;
            FullPivLU<MatrixXd> Hy_lu_decomp(Hy);
            std::cerr << "Hy rank = (" << Hy_lu_decomp.rank() << ")." << std::endl;
        }
    }

    return {uq, ua, x_tilde, y_tilde, y_partial};
}

/**
 * @brief convert_pose_jacobian_to_measure_space as discussed in Section IV of
 * M. M. Marinho and B. V. Adorno,
 * "Adaptive Constrained Kinematic Control Using Partial or Complete Task-Space Measurements,"
 * in IEEE Transactions on Robotics, vol. 38, no. 6, pp. 3498-3513, Dec. 2022,
 * doi: 10.1109/TRO.2022.3181047.
 * @param Jx the pose_jacobian, i.e. the complete Jacobian.
 * @param x the pose, i.e. the complete measurement.
 * @param xd the desired pose, used to calculate the rotation Jacobian.
 * @param measure_space see Example_MeasureSpace for possible values.
 * @return the (partial) Jacobian defined by Example_MeasureSpace.
 */
MatrixXd Example_AdaptiveController::_convert_pose_jacobian_to_measure_space(const MatrixXd& Jx, const DQ& x, const DQ& xd, const Example_MeasureSpace& measure_space)
{
    switch(measure_space)
    {
    case Example_MeasureSpace::None:
        throw std::runtime_error("Measurespace None not acceptable.");
    case Example_MeasureSpace::Pose:
        return haminus8(xd)*C8()*Jx;
    case Example_MeasureSpace::Rotation:
        return haminus4(rotation(xd))*C4()*DQ_Kinematics::rotation_jacobian(Jx);
    case Example_MeasureSpace::Translation:
        return DQ_Kinematics::translation_jacobian(Jx, x);
    case Example_MeasureSpace::Distance:
        return DQ_Kinematics::point_to_point_distance_jacobian(DQ_Kinematics::translation_jacobian(Jx, x), translation(x), DQ(0));
    }
    throw std::runtime_error("Not supposed to be reachable");
}

/**
 * @brief get_complimentary_measure_space_jacobian as discussed in Section IV of
 * M. M. Marinho and B. V. Adorno,
 * "Adaptive Constrained Kinematic Control Using Partial or Complete Task-Space Measurements,"
 * in IEEE Transactions on Robotics, vol. 38, no. 6, pp. 3498-3513, Dec. 2022,
 * doi: 10.1109/TRO.2022.3181047.
 * @param Jx the pose_jacobian, i.e. the complete Jacobian.
 * @param x the pose, i.e. the complete measurement.
 * @param measure_space see Example_MeasureSpace for possible values.
 * @return the complimentary of the (partial) Jacobian defined by Example_MeasureSpace.
 */
MatrixXd Example_AdaptiveController::_get_complimentary_measure_space_jacobian(const MatrixXd& Jx, const DQ &x, const Example_MeasureSpace& measure_space)
{
    switch(measure_space)
    {
    case Example_MeasureSpace::None:
        throw std::runtime_error("Measure space None not acceptable.");
    case Example_MeasureSpace::Pose:
        return MatrixXd(0,0);
    case Example_MeasureSpace::Rotation:
        return DQ_Kinematics::translation_jacobian(Jx, x);
    case Example_MeasureSpace::Translation:
        return DQ_Kinematics::rotation_jacobian(Jx);
    case Example_MeasureSpace::Distance:
        //TODO Update with the code that was used in the experiment with the robot.
        throw std::runtime_error("NOT IMPLEMENTED YET");
    }
    throw std::runtime_error("Not supposed to be reachable");
}

Example_AdaptiveController::Example_AdaptiveController(const std::shared_ptr<Example_SerialManipulatorEDH> &robot, const Example_SimulationParameters &simulation_arguments):
    robot_(robot),
    simulation_arguments_(simulation_arguments)
{

}

/**
 * @brief get_complimentary_measure_space_jacobian as discussed in Section IV of
 * M. M. Marinho and B. V. Adorno,
 * "Adaptive Constrained Kinematic Control Using Partial or Complete Task-Space Measurements,"
 * in IEEE Transactions on Robotics, vol. 38, no. 6, pp. 3498-3513, Dec. 2022,
 * doi: 10.1109/TRO.2022.3181047.
 * @param x the pose, e.g. the complete measurement.
 * @param measure_space see Example_MeasureSpace for possible values.
 * @return the (partial) measurement defined by Example_MeasureSpace.
 */
DQ Example_AdaptiveController::_convert_pose_to_measure_space(const DQ& x, const Example_MeasureSpace &measure_space)
{
    switch(measure_space)
    {
    case Example_MeasureSpace::None:
        throw std::runtime_error("None is not a valid DQ_MeasureSpace");
    case Example_MeasureSpace::Pose:
        return x;
    case Example_MeasureSpace::Rotation:
        return rotation(x);
    case Example_MeasureSpace::Translation:
        return translation(x);
    case Example_MeasureSpace::Distance:
        return DQ(DQ_Geometry::point_to_point_squared_distance(translation(x), DQ(0)));
    }
    throw std::runtime_error("Not supposed to be reachable");
}

VectorXd Example_AdaptiveController::_smart_vec(const DQ& x, const Example_MeasureSpace& measure_space)
{
    switch(measure_space)
    {
    case Example_MeasureSpace::None:
        throw std::runtime_error("None is not a valid DQ_MeasureSpace");
    case Example_MeasureSpace::Distance:
    {
        VectorXd x_vec(1); x_vec << static_cast<double>(x);
        return x_vec;
    }
    case Example_MeasureSpace::Rotation:
        return vec4(x);
    case Example_MeasureSpace::Translation:
        return vec4(x);
    case Example_MeasureSpace::Pose:
        return vec8(x);
    }
    throw std::runtime_error("Not supposed to be reachable");
}


