from dqrobotics import *
from dqrobotics.robot_modeling import DQ_SerialManipulator
from marinholab.papers.tro2022.adaptive_control._core import *

# Scene loading (YAML) and a pyplot-based M3_Simulator visualization backend.
# (Lazy-safe: importing this package does not require matplotlib /
# dqrobotics-pyplot; they are only needed when M3_PyPlotSimulator is used.)
# ``plane`` and ``line`` build the VFI-convention primitive DQs:
#   plane(position, normal)    ->  normal + E*dot(position, normal)
#   line(position, direction)  ->  direction + E*cross(position, direction)
# (they match what M3_VFI constructs internally).
from .scene import Scene, load_scene, M3_PyPlotSimulator, plane, line
