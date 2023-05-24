/*
# Copyright (c) 2020-2023 Murilo Marques Marinho (www.murilomarinho.info)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     - Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     - Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     - Neither the name of the Mitsuishi Sugita Laboratory (NML) nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License LGPL as
# published by the Free Software Foundation, either version 3 of the
# License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU Lesser General Public License LGPL for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License LGPL along with this program.
# If not, see <http://www.gnu.org/licenses/>.
#
# ################################################################
#
#   Author: Murilo M. Marinho, email: murilomarinho@ieee.org
#
# ################################################################*/

#include "utils.h"

#include <random>

#include <dqrobotics/utils/DQ_Constants.h>

namespace utils
{

double deg2rad(const double& a)
{
    return (a)*2.0*pi/(360.0);
}

DQ get_closest_xd_to_x(const DQ& x, const DQ& xd)
{
    if(vec8(x-xd).norm() < vec8(x+xd).norm())
    {
        return xd;
    }
    else
    {
        return -1.0*xd;
    }
}

DQ add_noise_to_measurement(const DQ& y, const double& metric_error_3sigma, const double& degrees_error_3sigma)
{
    std::default_random_engine generator;

    std::normal_distribution<double> angular_N(0,deg2rad(degrees_error_3sigma)/3.0);
    std::normal_distribution<double> metric_N(0,metric_error_3sigma/3.0);

    std::uniform_real_distribution<double> unit_vector_U (0.0,1.0);

    // Get random unit vector for the rotation quaternion
    VectorXd v_vec(3); v_vec << unit_vector_U(generator), unit_vector_U(generator), unit_vector_U(generator);
    const DQ v(v_vec); v_vec.normalize();
    // Get random angle
    double angle = angular_N(generator);
    // Get random rotation quaternion
    const DQ r = cos(angle/2.0) + v*sin(angle/2.0);

    // Get random translation quaternion
    VectorXd t_vec(3); t_vec << metric_N(generator), metric_N(generator), metric_N(generator);
    const DQ t(t_vec);

    // Get the random DQ
    const DQ x = r + 0.5*E_*t*r;
    return y*x;
}

std::tuple<MatrixXd, VectorXd> get_variable_boundary_inequalities(const VectorXd& q, const std::tuple<VectorXd, VectorXd>& boundaries, const VectorXd& damping_matrix_diagonal)
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

}
