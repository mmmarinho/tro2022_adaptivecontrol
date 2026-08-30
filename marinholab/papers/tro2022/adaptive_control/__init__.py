from dqrobotics import *
from dqrobotics.robot_modeling import DQ_SerialManipulator
from marinholab.papers.tro2022.adaptive_control._core import *

# Scene loading (YAML) and a pyplot-based M3_Simulator visualization backend.
# (Lazy-safe: importing this package does not require matplotlib /
# dqrobotics-pyplot; they are only needed when M3_PyPlotSimulator is used.)
# ``plane`` and ``line`` build the VFI-convention primitive DQs from pure
# quaternions: ``n``/``l`` (unit normal/direction) and ``p`` (a point on the
# primitive, i.e. a translation):
#   plane(n, p)  ->  n + E*dot(p, n)
#   line(l, p)   ->  l + E*cross(p, l)
# (they match what M3_VFI constructs internally).
from .scene import Scene, load_scene, M3_PyPlotSimulator, plane, line
