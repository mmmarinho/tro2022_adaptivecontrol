#pragma once
/**
 * (C) Copyright 2023-2026 Murilo Marques Marinho (www.murilomarinho.info)
 *
 * This file is part of adaptive_control_example.
 *
 * SPDX-License-Identifier: MIT
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @brief The Example_MeasureSpace enum represent the use cases discurssed in.
 *
 * M. M. Marinho and B. V. Adorno,
 * "Adaptive Constrained Kinematic Control Using Partial or Complete Task-Space Measurements,"
 * in IEEE Transactions on Robotics, vol. 38, no. 6, pp. 3498-3513, Dec. 2022,
 * doi: 10.1109/TRO.2022.3181047.
 */

namespace marinholab::papers::tro2022::adaptive_control
{

enum class MeasureSpace
{
    None=0,
    Pose,
    Rotation,
    Translation,
    Distance,
};

int get_measure_space_dimension(const MeasureSpace &measure_space);

}  // namespace marinholab::papers::tro2022::adaptive_control
