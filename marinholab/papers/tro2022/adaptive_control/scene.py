"""
Scene loading (YAML) and a pyplot-based visualization backend for M3_Simulator.

A scene file (YAML) describes the robot(s), the environment (planes, lines) and
the task targets (points) of a TRO2022-style adaptive-control example. It is
loaded into a :class:`M3_Simulator` (named object poses + robot configuration)
and can be drawn with :class:`M3_PyPlotSimulator`, which uses
``dqrobotics_extensions.pyplot`` (matplotlib).

Scene file format
-----------------
::

    robots:
      - name: vs050            # object name for the robot base frame
        dh:                    # 5 x n DH matrix: [theta; d; a; alpha; offset]
          - [theta_1, theta_2, ...]
          - [d_1,    d_2,    ...]
          - [a_1,    a_2,    ...]
          - [alpha_1, alpha_2, ...]
          - [offset_1, offset_2, ...]
        limits:                # optional
          lower: [-170, -100, -60, -265, -119, -355]
          upper: [ 170,  100, 124,  265,   89.9, 355]
          unit: deg            # 'deg' (default) or 'rad'
        base: [0, 0, 0]        # optional base-frame translation
        effector: [0, 0, 0]    # optional effector-frame translation
    points:
      - name: xd0
        pos: [0.0, 0.0, 0.0]
        radius: 0.03           # optional (default 0.02)
        color: 'g'             # optional
    planes:
      - name: wall_1           # plane with unit `normal` through `position`
        position: [0.2, 0.0, 0.0]
        normal:   [-1.0, 0.0, 0.0]
        color: 'g'             # optional
    lines:
      - name: tube_1           # line with unit `direction` through `position`
        position: [0.0, 0.12, 0.0]
        direction: [0.0, 0.0, 1.0]
        color: 'r'             # optional
    q0: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]   # optional initial configuration (rad)

Conventions match :class:`M3_VFI`: a plane is defined by a point (``position``)
and a (unit) normal (``normal``); a line by a point (``position``) and a (unit)
direction (``direction``). Each primitive is also stored on the simulator as a
frame pose whose k-axis is the normal/direction and whose origin is the
``position``, so a :class:`M3_VFI` built from that object pose reconstructs the
same primitive.
"""

from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np

from dqrobotics import *
from marinholab.papers.tro2022.adaptive_control._core import (
    M3_SerialManipulatorEDH,
    M3_Simulator,
)


@dataclass
class Scene:
    """A parsed scene file loaded into a simulator.

    Attributes:
        robots: (name, M3_SerialManipulatorEDH) pairs.
        points: (name, radius, color) pairs (position is in the simulator).
        planes: (name, plane DQ, color) triples.
        lines: (name, line DQ, color) triples.
        q0: optional initial configuration (radians) or None.
        sim: the simulator the scene was loaded into.
    """

    robots: List[Tuple[str, M3_SerialManipulatorEDH]]
    points: List[Tuple[str, float, str]]
    planes: List[Tuple[str, DQ, str]]
    lines: List[Tuple[str, DQ, str]]
    q0: Optional[np.ndarray]
    sim: M3_Simulator


def _unit(v):
    """The unit vector of a non-zero 3-vector ``v`` (via DQ.normalize)."""
    v = np.asarray(v, dtype=float)
    if np.linalg.norm(v) < 1e-12:
        raise ValueError("expected a non-zero vector, got a zero vector")
    # DQ() on a 3-vector is a pure quaternion; normalize() returns the
    # normalized DQ (a pure unit quaternion), so the vector part is q[1:4].
    return np.array(DQ(v).normalize().q[1:4])


def _pose_from_translation(t) -> DQ:
    """A DQ pose from a translation ``t`` and the identity rotation."""
    return DQ([1.0]) + 0.5 * E_ * DQ(np.asarray(t, dtype=float)) * DQ([1.0])


def _rotation_quat_between(a, b) -> DQ:
    """Unit quaternion (as a DQ) rotating unit vector ``a`` onto unit vector ``b``."""
    a = _unit(a)
    b = _unit(b)
    cosang = float(np.clip(np.dot(a, b), -1.0, 1.0))
    if cosang > 1.0 - 1e-12:            # same direction: identity
        return DQ([1.0, 0.0, 0.0, 0.0])
    if cosang < -1.0 + 1e-12:           # opposite: 180 deg about an orthogonal axis
        tmp = np.array([1.0, 0.0, 0.0]) if abs(a[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
        v = np.cross(a, tmp)
        return DQ([0.0, v[0], v[1], v[2]])
    v = np.cross(a, b)
    vhat = v / np.linalg.norm(v)
    half_cos = float(np.sqrt((1.0 + cosang) / 2.0))
    half_sin = float(np.sqrt((1.0 - cosang) / 2.0))
    return DQ([half_cos, vhat[0] * half_sin, vhat[1] * half_sin, vhat[2] * half_sin])


def _frame_pose(r: DQ, t) -> DQ:
    """A frame pose: rotation ``r`` (unit quaternion DQ) + translation ``t``."""
    return r + 0.5 * E_ * DQ(np.asarray(t, dtype=float)) * r


def _plane_dq(position, normal) -> DQ:
    """Plane DQ (VFI convention) for unit ``normal`` through ``position``."""
    n_dq = DQ(np.asarray(normal, dtype=float))
    return n_dq + E_ * dot(DQ(np.asarray(position, dtype=float)), n_dq)


def _line_dq(position, direction) -> DQ:
    """Line DQ (VFI convention) for unit ``direction`` through ``position``."""
    l_dq = DQ(np.asarray(direction, dtype=float))
    return l_dq + E_ * cross(DQ(np.asarray(position, dtype=float)), l_dq)


def load_scene(path: str, sim: M3_Simulator) -> Scene:
    """Load a scene from a YAML file into ``sim`` and return a :class:`Scene`.

    See the module docstring for the accepted keys (robots, points, planes,
    lines, q0). Named object poses are stored on the simulator; the parsed
    robots and primitives are returned for visualization.
    """
    import yaml

    with open(path, "r") as f:
        data = yaml.safe_load(f) or {}

    sim.stop_simulation()

    robots: List[Tuple[str, M3_SerialManipulatorEDH]] = []
    for r in data.get("robots", []) or []:
        name = r["name"]
        dh = np.asarray(r["dh"], dtype=float)
        if dh.shape[0] != 5:
            raise ValueError(
                f"robot '{name}': the DH matrix must have 5 rows "
                f"[theta; d; a; alpha; offset]; got shape {dh.shape}"
            )
        robot = M3_SerialManipulatorEDH(dh)
        limits = r.get("limits")
        if limits is not None:
            scale = np.pi / 180.0 if limits.get("unit", "deg") == "deg" else 1.0
            robot.set_lower_q_limit(np.asarray(limits["lower"], dtype=float) * scale)
            robot.set_upper_q_limit(np.asarray(limits["upper"], dtype=float) * scale)
        if "base" in r:
            robot.set_base_frame(_pose_from_translation(r["base"]))
        else:
            robot.set_base_frame(DQ([1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
        if "effector" in r:
            robot.set_effector_frame(_pose_from_translation(r["effector"]))
        else:
            robot.set_effector_frame(DQ([1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
        sim.set_object_pose(name, robot.get_base_frame())
        robots.append((name, robot))

    points: List[Tuple[str, float, str]] = []
    for p in data.get("points", []) or []:
        name = p["name"]
        sim.set_object_pose(name, _pose_from_translation(p["pos"]))
        points.append((name, float(p.get("radius", 0.02)), p.get("color", "b")))

    planes: List[Tuple[str, DQ, str]] = []
    for p in data.get("planes", []) or []:
        name = p["name"]
        n_vec = _unit(p["normal"])
        p0 = np.asarray(p["position"], dtype=float)
        r = _rotation_quat_between(np.array([0.0, 0.0, 1.0]), n_vec)
        sim.set_object_pose(name, _frame_pose(r, p0))
        planes.append((name, _plane_dq(p0, n_vec), p.get("color", "g")))

    lines: List[Tuple[str, DQ, str]] = []
    for l in data.get("lines", []) or []:
        name = l["name"]
        d_vec = _unit(l["direction"])
        p0 = np.asarray(l["position"], dtype=float)
        r = _rotation_quat_between(np.array([0.0, 0.0, 1.0]), d_vec)
        sim.set_object_pose(name, _frame_pose(r, p0))
        lines.append((name, _line_dq(p0, d_vec), l.get("color", "r")))

    q0 = None
    if "q0" in data:
        q0 = np.asarray(data["q0"], dtype=float)
        sim.set_configuration_space_positions(q0)

    return Scene(robots=robots, points=points, planes=planes, lines=lines, q0=q0, sim=sim)


def _require_dqp():
    try:
        import dqrobotics_extensions.pyplot as dqp
    except ImportError as e:
        raise ImportError(
            "M3_PyPlotSimulator requires matplotlib and dqrobotics-pyplot; "
            "install them with:  python3 -m pip install dqrobotics-pyplot"
        ) from e
    return dqp


def _draw_robot(ax, scene: Scene, sim: M3_Simulator):
    dqp = _require_dqp()
    q = np.asarray(sim.get_configuration_space_positions())
    for _name, robot in scene.robots:
        dqp.plot(robot, q=q, ax=ax)


def _draw_points(ax, scene: Scene, sim: M3_Simulator):
    dqp = _require_dqp()
    for name, radius, color in scene.points:
        # _plot_sphere expects a pure quaternion at the point's position.
        p = translation(sim.get_object_pose(name))
        dqp.plot(p, sphere=True, radius=radius, color=color, ax=ax)


def _draw_planes(ax, scene: Scene):
    dqp = _require_dqp()
    for _name, pi_dq, color in scene.planes:
        dqp.plot(pi_dq, plane=True, scale=0.4, color=color, ax=ax)


def _draw_lines(ax, scene: Scene):
    dqp = _require_dqp()
    for _name, l_dq, color in scene.lines:
        dqp.plot(l_dq, line=True, scale=0.5, color=color, ax=ax)


class M3_PyPlotSimulator(M3_Simulator):
    """A visualization backend for :class:`M3_Simulator`.

    Loads a YAML scene (see :func:`load_scene`) and draws it with
    ``dqrobotics_extensions.pyplot`` (matplotlib). It overrides the virtual
    :meth:`draw_scene` provided by the C++ base (through the pybind11
    trampoline), so it is a concrete example of subclassing ``M3_Simulator``
    from Python.

    Example:

        import matplotlib.pyplot as plt
        from marinholab.papers.tro2022.adaptive_control import M3_PyPlotSimulator

        sim = M3_PyPlotSimulator()
        sim.load_scene("scene.yaml")
        plt.figure()
        plt.axes(projection="3d")
        sim.draw_scene()
        plt.show()

    The current matplotlib axes is used (the dqrobotics_extensions.pyplot
    convention); a 3d axes is created on demand if none is current.
    """

    def __init__(self):
        super().__init__()
        self._scene: Optional[Scene] = None

    def load_scene(self, path: str) -> Scene:
        """Load a YAML scene into this simulator (see :func:`load_scene`)."""
        self._scene = load_scene(path, self)
        return self._scene

    def draw_scene(self):
        """Draw the loaded scene onto the current (3d) matplotlib axes."""
        if self._scene is None:
            raise RuntimeError("no scene loaded; call load_scene(path) first")
        from matplotlib import pyplot as plt

        ax = plt.gca()
        if not hasattr(ax, "proj3d"):
            ax = plt.axes(projection="3d")
        _draw_robot(ax, self._scene, self)
        _draw_points(ax, self._scene, self)
        _draw_planes(ax, self._scene)
        _draw_lines(ax, self._scene)
