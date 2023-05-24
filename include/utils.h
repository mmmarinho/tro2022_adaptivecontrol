#pragma once
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
#include <dqrobotics/DQ.h>

using namespace DQ_robotics;

namespace utils
{

double deg2rad(const double& a);

DQ get_closest_xd_to_x(const DQ& x, const DQ& xd);

DQ add_noise_to_measurement(const DQ& y, const double& metric_error_3sigma, const double& degrees_error_3sigma);

std::tuple<MatrixXd, VectorXd> get_variable_boundary_inequalities(const VectorXd& q, const std::tuple<VectorXd, VectorXd>& boundaries, const VectorXd &damping_matrix_diagonal=VectorXd());

}

