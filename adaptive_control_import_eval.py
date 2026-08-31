# (C) Copyright 2025-2026 Murilo Marques Marinho (www.murilomarinho.info)
#
# This file is part of adaptive_control_example.
#
# SPDX-License-Identifier: MIT
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import glob
import importlib.util
import os
import site
import sys

import numpy as np
from numpy import pi
from dqrobotics import *

# When this script runs from the repository root, the source `marinholab/`
# tree shadows the installed package (which contains the compiled `_core`
# extension). Register the installed extension under the package name so the
# import below resolves it. No-op when the package is already importable
# (e.g. running from another directory or from an in-place build).
_CORE_NAME = "marinholab.papers.tro2022.adaptive_control._core"
try:
    import marinholab.papers.tro2022.adaptive_control  # noqa: F401
except ModuleNotFoundError:
    _pkg_rel = os.path.join("marinholab", "papers", "tro2022", "adaptive_control")
    _site_dirs = list(site.getsitepackages())
    try:
        _user_site = site.getusersitepackages()
        if _user_site:
            _site_dirs.append(_user_site)
    except Exception:
        pass
    for _site_dir in _site_dirs:
        for _pattern in ("_core*.so", "_core*.pyd", "_core*.dll"):
            _hits = sorted(glob.glob(os.path.join(_site_dir, _pkg_rel, _pattern)))
            if _hits:
                _spec = importlib.util.spec_from_file_location(_CORE_NAME, _hits[0])
                _module = importlib.util.module_from_spec(_spec)
                sys.modules[_CORE_NAME] = _module
                _spec.loader.exec_module(_module)
                break
        if _CORE_NAME in sys.modules:
            break

from marinholab.papers.tro2022.adaptive_control import *

plot_enabled = None
try:
    import matplotlib.pyplot as plt
    import dqrobotics_extensions.pyplot as dqp
    plot_enabled = True
except ImportError:
    print("Running example without plot as the modules could not be found.")
    plot_enabled = False

def setup_plot():
    # Set up the plot
    fig = plt.figure()
    plot_size = 1
    ax = plt.axes(projection='3d')
    ax.set_xlabel('$x$')
    ax.set_xlim((-plot_size, plot_size))
    ax.set_ylabel('$y$')
    ax.set_ylim((-plot_size, plot_size))
    ax.set_zlabel('$z$')
    ax.set_zlim((-plot_size, plot_size))
    return fig, ax

def vs050_raw_kinematics() -> M3_SerialManipulatorEDH:
    """
    Gets the ideal kinematics of the VS050 robot.
    :return:
    """
    VS050_dh_matrix = np.array([
        [-pi, pi / 2, -pi / 2, 0, pi, 0],
        [0.345, 0, 0, 0.255, 0, 0.07],
        [0, 0.250, 0.01, 0, 0, 0],
        [pi / 2, 0, -pi / 2, pi / 2, pi / 2, 0],
        [0, 0, 0, 0, 0, 0]
    ])

    lower_joint_limits = np.array([-170, -100, -60, -265, -119, -355]) * pi / 180
    upper_joint_limits = np.array([170, 100, 124, 265, 89.9, 355]) * pi / 180

    robot = M3_SerialManipulatorEDH(VS050_dh_matrix)
    robot.set_lower_q_limit(lower_joint_limits)
    robot.set_upper_q_limit(upper_joint_limits)

    robot.set_base_frame(DQ([1]))
    robot.set_effector_frame(DQ([1]))

    return robot

def main():
    if plot_enabled: setup_plot()

    estimated_robot = vs050_raw_kinematics()
    q_init = np.array([0, pi/4, pi/2, 0, pi/4, 0])

    if plot_enabled:
        dqp.plot(estimated_robot,
                 q=q_init,
                 line_color='r',
                 cylinder_color="r",
                 cylinder_alpha=0.3)
        plt.show(block=True)


if __name__ == "__main__":
    main()