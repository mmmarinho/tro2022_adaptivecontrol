/**
(C) Copyright 2020-2023 Murilo Marques Marinho (www.murilomarinho.info)

This file is part of DQ adaptive_control_example.

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
#include <robot_control/DQ_AdaptiveController.h>
#include "utils.h"
#include <dqrobotics/utils/DQ_Math.h>

std::tuple<VectorXd,double> __closest_invariant_error(const DQ& x, const DQ& xd, const DQ_MeasureSpace& measure_space)
{
    switch(measure_space)
    {
    case DQ_MeasureSpace::None:
        throw std::runtime_error("None is not a valid DQ_MeasureSpace");
    case DQ_MeasureSpace::Distance:
    {
        //There is no double cover in distance
        VectorXd e_d(1);
        e_d << (static_cast<double>(x)-static_cast<double>(xd));
        return {e_d, 1};
    }
    case DQ_MeasureSpace::Rotation:
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
    case DQ_MeasureSpace::Translation:
    {
        //There is no double cover in translation
        return {vec4(x-xd),1};
    }
    case DQ_MeasureSpace::Pose:
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


std::tuple<VectorXd, VectorXd, VectorXd, VectorXd, DQ> compute_setpoint_control_signal(const DQ_AdaptiveControlStrategy& control_strategy,
                                                                                         const VectorXd&q,
                                                                                         const DQ_SerialManipulatorEDH& robot,
                                                                                         const DQ& x_d,
                                                                                         const DQ& y,
                                                                                         std::vector<VFI_Information>& vfis,
                                                                                         const SimulationArguments& cla)
{
    const int n = robot.get_dim_configuration_space();
    const int p = robot.get_dim_parameter_space();
    const double& eta_task = cla.proportional_gain;
    const double& eta_parameter = cla.proportional_gain;
    const double& vfi_gain = cla.vfi_gain;
    const double& vfi_weight = cla.vfi_weight;
    const double& lambda = cla.damping;
    const DQ_MeasureSpace& measure_space = cla.measure_space;

    const double& MAX_ACCEPTABLE_CONSTRAINT_PENETRATION = 0.001;
    const double& MAX_ACCEPTABLE_CONSTRAINT_PENETRATION_SQUARED = MAX_ACCEPTABLE_CONSTRAINT_PENETRATION*MAX_ACCEPTABLE_CONSTRAINT_PENETRATION;

    VectorXd uq = VectorXd::Zero(n);
    ///Estimated measurement (used in both control strategies)
    const DQ x_hat = robot.fkm(q);
    double x_invariant;
    VectorXd x_tilde;
    std::tie(x_tilde, x_invariant) = __closest_invariant_error(x_hat, x_d, DQ_MeasureSpace::Pose);
    ///VFI state that is independent of control strategy
    const int& vfis_size = static_cast<int>(vfis.size());
    VectorXd w_vfi(vfis_size);
    for(int i=0;i<vfis_size;i++)
    {
        VFI_Information& vfi = vfis[i];
        w_vfi(i) = vfi.get_distance_error(x_hat);
        switch(vfi.get_distance_type())
        {
        case VFI_DistanceType::None:
            throw std::runtime_error("Expected valid value");
        case VFI_DistanceType::EUCLIDEAN:
        {
            if(w_vfi(i) < -MAX_ACCEPTABLE_CONSTRAINT_PENETRATION)
            {
                std::cout << vfi.get_vfi_name() << " estimated penetration: " << w_vfi(i) << std::endl;
                //throw std::runtime_error("Distance to obstacle point over threshold " + std::to_string(w_vfi(i)));
            }
        }
        case VFI_DistanceType::EUCLIDEAN_SQUARED:
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

    if(control_strategy == DQ_AdaptiveControlStrategy::FULL || control_strategy == DQ_AdaptiveControlStrategy::TASK_ONLY)
    {
        ///Task
        const MatrixXd J_x_q = robot.pose_jacobian(q);
        const MatrixXd N_x_q = haminus8(x_d)*C8()*robot.pose_jacobian(q);

        const MatrixXd Hx = (N_x_q.transpose()*N_x_q + lambda*MatrixXd::Identity(n,n));
        const VectorXd fx = 2.*N_x_q.transpose()*eta_task*x_tilde;

        //I*q <= q_dot_upper
        //I*q >= q_dot_lower <=> -I*q <= -q_dot_lower
        MatrixXd W_q_dot_limits(2*n, n); W_q_dot_limits << MatrixXd::Identity(n,n), -1.0*MatrixXd::Identity(n,n);
        VectorXd w_q_dot_limits(2*n);    w_q_dot_limits << robot.get_upper_q_dot_limit(), -1.0*robot.get_lower_q_dot_limit();

        const int& vfis_size = static_cast<int>(vfis.size());

        ///Inequality constraint for VFIs
        MatrixXd W_vfi_q(vfis_size, n);
        for(int i=0;i<vfis_size;i++)
        {
            VFI_Information& vfi = vfis[i];
            W_vfi_q.row(i) = vfi.get_vfi_matrix(x_hat, J_x_q);
        }

        MatrixXd W_q_limits;
        VectorXd w_q_limits;
        std::tie(W_q_limits, w_q_limits) = utils::get_variable_boundary_inequalities(q,
                                                                                     {robot.get_lower_q_limit(),
                                                                                     robot.get_upper_q_limit()});

        MatrixXd W_q(4*n+W_vfi_q.rows(),n);W_q << W_q_dot_limits, W_q_limits, W_vfi_q;
        VectorXd w_q(4*n+w_vfi.size()); w_q << w_q_dot_limits, w_q_limits, vfi_gain*vfi_weight*w_vfi;

        DQ_QPOASESSolver task_space_solver;
        uq = task_space_solver.solve_quadratic_program(2.0*Hx, fx, W_q, w_q, MatrixXd::Zero(0,0), VectorXd(0));
    }


    VectorXd ua = VectorXd::Zero(p);
    VectorXd y_tilde;
    DQ y_partial;
    ///Adaptation
    if(y != DQ(0) && ( control_strategy == DQ_AdaptiveControlStrategy::FULL || control_strategy == DQ_AdaptiveControlStrategy::MEASUREMENT_ONLY ))
    {
        //Get a partial measurement if needed and the measurement error
        const DQ y_hat = x_hat;

        y_partial = convert_pose_to_measure_space(y, measure_space);
        const DQ y_hat_partial = convert_pose_to_measure_space(y_hat, measure_space);
        double y_invariant;
        std::tie(y_tilde, y_invariant) = __closest_invariant_error(y_hat_partial, y_partial, measure_space);

        const MatrixXd J_y_a = robot.parameter_pose_jacobian(q);
        const MatrixXd J_y_a_partial = convert_pose_jacobian_to_measure_space(J_y_a, y_hat, y, measure_space);

        ///Objective function
        const MatrixXd Hy = (J_y_a_partial.transpose()*J_y_a_partial + lambda*MatrixXd::Identity(p,p));
        const VectorXd fy = 2.*J_y_a_partial.transpose()*eta_parameter*y_tilde;

        ///Equality constraint
        const MatrixXd N_a = get_complimentary_measure_space_jacobian(J_y_a, y_hat, measure_space);
        const VectorXd b_N_a = VectorXd::Zero(N_a.rows());

        ///Inequality constraint for Lyapunov stability
        const MatrixXd A_y = x_tilde.transpose()*haminus8(x_d)*C8()*J_y_a;
        const VectorXd b_y = VectorXd::Zero(1);

        MatrixXd W_vfi_a(vfis_size, p);
        for(int i=0; i<vfis_size; i++)
        {
            const VFI_Information& vfi = vfis.at(i);
            W_vfi_a.row(i) = vfi.get_vfi_matrix(x_hat, J_y_a);
        }

        ///Inequality constraint for parameter limits
        const VectorXd a_hat = robot.get_parameter_space_values();
        auto parameter_boundaries = robot.get_parameter_space_boundaries();
        MatrixXd W_a_hat_limits;
        VectorXd w_a_hat_limits;
        std::tie(W_a_hat_limits, w_a_hat_limits) = utils::get_variable_boundary_inequalities(a_hat, parameter_boundaries);

        ///Composition of inequality constraints
        MatrixXd W_a_hat;
        W_a_hat.resize(A_y.rows()+W_a_hat_limits.rows()+W_vfi_a.rows(),p); W_a_hat << A_y, W_a_hat_limits, W_vfi_a;
        VectorXd w_a_hat;
        w_a_hat.resize(b_y.size()+w_a_hat_limits.size()+w_vfi.size()); w_a_hat << b_y, w_a_hat_limits, vfi_gain*(1.0 - vfi_weight)*w_vfi;

        DQ_QPOASESSolver parameter_space_solver;
        try
        {
            ua = parameter_space_solver.solve_quadratic_program(2.0*Hy, fy, W_a_hat, w_a_hat, N_a, b_N_a);
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

MatrixXd convert_pose_jacobian_to_measure_space(const MatrixXd& Jx, const DQ& x, const DQ& xd, const DQ_MeasureSpace& measure_space)
{
    switch(measure_space)
    {
    case DQ_MeasureSpace::None:
        throw std::runtime_error("Measurespace None not acceptable.");
    case DQ_MeasureSpace::Pose:
        return haminus8(xd)*C8()*Jx;
    case DQ_MeasureSpace::Rotation:
        return haminus4(rotation(xd))*C4()*DQ_Kinematics::rotation_jacobian(Jx);
    case DQ_MeasureSpace::Translation:
        return DQ_Kinematics::translation_jacobian(Jx, x);
    case DQ_MeasureSpace::Distance:
        return DQ_Kinematics::point_to_point_distance_jacobian(DQ_Kinematics::translation_jacobian(Jx, x), translation(x), DQ(0));
    }
    throw std::runtime_error("Not supposed to be reachable");
}

MatrixXd get_complimentary_measure_space_jacobian(const MatrixXd& Jx, const DQ &x, const DQ_MeasureSpace& measure_space)
{
    switch(measure_space)
    {
    case DQ_MeasureSpace::None:
        throw std::runtime_error("Measurespace None not acceptable.");
    case DQ_MeasureSpace::Pose:
        return MatrixXd(0,0);
    case DQ_MeasureSpace::Rotation:
        return DQ_Kinematics::translation_jacobian(Jx, x);
    case DQ_MeasureSpace::Translation:
        return DQ_Kinematics::rotation_jacobian(Jx);
    case DQ_MeasureSpace::Distance:
        MatrixXd J_r = DQ_Kinematics::rotation_jacobian(Jx);

        // Choose an arbitrary vector. In this case I'll always choose z.
        DQ z = k_;
        // Get the direction of the distance as a.
        DQ a = translation(x).normalize();

        // One rotation to align z to a will be
        double angle = acos(dot(z, a).q(0));
        DQ axis = cross(z, a);
        DQ r = cos(angle/2.0) + axis*sin(angle/2.0);

        // We then rotate the Translation Jacobian
        MatrixXd J_t_hat = hamiplus4(conj(r))*haminus4(r)*DQ_Kinematics::translation_jacobian(Jx, x);

        // We then get only the translation about x and y in this rotated frame
        MatrixXd J_t_x_y = J_t_hat.block(0, 0, 2, J_t_hat.cols());

        MatrixXd N_a(6, Jx.cols());
        N_a << J_r, J_t_x_y;

        return N_a;
    }
    throw std::runtime_error("Not supposed to be reachable");
}


DQ convert_pose_to_measure_space(const DQ& x, const DQ_MeasureSpace& measure_space)
{
    switch(measure_space)
    {
    case DQ_MeasureSpace::None:
        throw std::runtime_error("None is not a valid DQ_MeasureSpace");
    case DQ_MeasureSpace::Pose:
        return x;
    case DQ_MeasureSpace::Rotation:
        return rotation(x);
    case DQ_MeasureSpace::Translation:
        return translation(x);
    case DQ_MeasureSpace::Distance:
        return DQ(DQ_Geometry::point_to_point_squared_distance(translation(x), DQ(0)));
    }
    throw std::runtime_error("Not supposed to be reachable");
}

VectorXd smart_vec(const DQ& x, const DQ_MeasureSpace& measure_space)
{
    switch(measure_space)
    {
    case DQ_MeasureSpace::None:
        throw std::runtime_error("None is not a valid DQ_MeasureSpace");
    case DQ_MeasureSpace::Distance:
    {
        VectorXd x_vec(1); x_vec << static_cast<double>(x);
        return x_vec;
    }
    case DQ_MeasureSpace::Rotation:
        return vec4(x);
    case DQ_MeasureSpace::Translation:
        return vec4(x);
    case DQ_MeasureSpace::Pose:
        return vec8(x);
    }
    throw std::runtime_error("Not supposed to be reachable");
}

int get_measure_space_dimension(const DQ_MeasureSpace &measure_space)
{
    switch(measure_space)
    {
    case DQ_MeasureSpace::None:
        return 0;
    case DQ_MeasureSpace::Pose:
        return 8;
    case DQ_MeasureSpace::Rotation:
        return 4;
    case DQ_MeasureSpace::Translation:
        return 4;
    case DQ_MeasureSpace::Distance:
        return 1;
    }
    throw std::runtime_error("Not supposed to be reachable");
}
