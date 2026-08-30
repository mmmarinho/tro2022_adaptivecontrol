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


def _unit(v) -> DQ:
    """Pure unit quaternion DQ of a non-zero 3-vector ``v`` (via DQ.normalize)."""
    v_dq = DQ(np.asarray(v, dtype=float))
    if float(norm(v_dq).q[0]) < 1e-12:
        raise ValueError("expected a non-zero vector, got a zero vector")
    return v_dq.normalize()


def _pose_from_translation(t) -> DQ:
    """A DQ pose from a translation ``t`` and the identity rotation."""
    return DQ([1.0]) + 0.5 * E_ * DQ(np.asarray(t, dtype=float)) * DQ([1.0])


def _rotation_quat_between(a: DQ, b: DQ) -> DQ:
    """Unit quaternion (as a DQ) rotating unit vector ``a`` onto unit vector ``b``."""
    cosang = float(np.clip(dot(a, b).q[0], -1.0, 1.0))
    if cosang > 1.0 - 1e-12:            # same direction: identity
        return DQ([1.0, 0.0, 0.0, 0.0])
    if cosang < -1.0 + 1e-12:           # opposite: 180 deg about an orthogonal axis
        tmp = DQ([1.0, 0.0, 0.0]) if abs(a.q[1]) < 0.9 else DQ([0.0, 1.0, 0.0])
        # cross(a, tmp) is a pure quaternion; normalized, it is the 180-degree
        # (half-angle 90-degree) quaternion about that axis.
        return cross(a, tmp).normalize()
    vhat = cross(a, b).normalize()      # pure unit quaternion = the rotation axis
    half_cos = float(np.sqrt((1.0 + cosang) / 2.0))
    half_sin = float(np.sqrt((1.0 - cosang) / 2.0))
    return half_cos + half_sin * vhat


def _frame_pose(r: DQ, t: DQ) -> DQ:
    """A frame pose: rotation ``r`` (unit quaternion DQ) + translation ``t``."""
    return r + 0.5 * E_ * t * r


def plane(n: DQ, p: DQ) -> DQ:
    """Plane DQ (VFI convention) for a unit normal ``n`` through the point
    ``p``. ``n`` and ``p`` are pure quaternions (``p`` is a translation, i.e.
    a point on the plane)."""
    return n + E_ * dot(p, n)


def line(l: DQ, p: DQ) -> DQ:
    """Line DQ (VFI convention) for a unit direction ``l`` through the point
    ``p``. ``l`` and ``p`` are pure quaternions (``p`` is a translation, i.e.
    a point on the line)."""
    return l + E_ * cross(p, l)


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
    for pl in data.get("planes", []) or []:
        name = pl["name"]
        normal = _unit(pl["normal"])
        p0 = DQ(np.asarray(pl["position"], dtype=float))
        r = _rotation_quat_between(DQ([0.0, 0.0, 1.0]), normal)
        sim.set_object_pose(name, _frame_pose(r, p0))
        planes.append((name, plane(normal, p0), pl.get("color", "g")))

    lines: List[Tuple[str, DQ, str]] = []
    for ln in data.get("lines", []) or []:
        name = ln["name"]
        d_vec = _unit(ln["direction"])
        p0 = DQ(np.asarray(ln["position"], dtype=float))
        r = _rotation_quat_between(DQ([0.0, 0.0, 1.0]), d_vec)
        sim.set_object_pose(name, _frame_pose(r, p0))
        lines.append((name, line(d_vec, p0), ln.get("color", "r")))

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


def _as_configuration_sequence(q) -> List[np.ndarray]:
    """Normalize ``q`` into a list of configuration arrays.

    Accepts a single 1-D configuration, a list/tuple of configurations, or a
    2-D array of shape ``(n, dim)`` (one configuration per row). A list/tuple
    of scalars is treated as one configuration.
    """
    if isinstance(q, (list, tuple)):
        # A list of scalars is a single configuration; a list of sequences is
        # a sequence of configurations.
        if q and np.asarray(q[0]).ndim == 0:
            return [np.asarray(q, dtype=float)]
        return [np.asarray(c, dtype=float) for c in q]
    arr = np.asarray(q, dtype=float)
    if arr.ndim == 1:
        return [arr]
    if arr.ndim == 2:
        return [arr[i] for i in range(arr.shape[0])]
    raise ValueError(
        f"expected a 1-D configuration or a 2-D array of shape (n, dim), got "
        f"an array of shape {arr.shape}"
    )


def _animation_writer(path: str, fps: float):
    """Pick a matplotlib ``MovieWriter`` for ``path`` (GIF, or ffmpeg-based)."""
    import os
    import shutil

    import matplotlib.animation as manim

    ext = os.path.splitext(path)[1].lower()
    if ext == ".gif":
        return manim.PillowWriter(fps=fps)
    if shutil.which("ffmpeg") is None:
        raise RuntimeError(
            f"writing '{path}' needs ffmpeg, which is not on PATH; use a "
            f".gif path instead (always available)"
        )
    return manim.FFMpegWriter(fps=fps)


class M3_PyPlotSimulator(M3_Simulator):
    """A visualization backend for :class:`M3_Simulator`.

    Loads a YAML scene (see :func:`load_scene`) and draws it with
    ``dqrobotics_extensions.pyplot`` (matplotlib). It overrides the virtual
    :meth:`draw_scene` provided by the C++ base (through the pybind11
    trampoline), so it is a concrete example of subclassing ``M3_Simulator``
    from Python.

    Example (single image):

        import matplotlib.pyplot as plt
        from marinholab.papers.tro2022.adaptive_control import M3_PyPlotSimulator

        sim = M3_PyPlotSimulator()
        sim.load_scene("scene.yaml")
        plt.figure()
        plt.axes(projection="3d")
        sim.draw_scene()
        plt.show()

    ``draw_scene`` is idempotent: the static scene elements (points, planes,
    lines) are drawn once per axes and only the robot is redrawn afterwards,
    so repeated calls do not create new axes or accumulate artists. That is
    also what makes :meth:`animation` cheap.

    Example (matplotlib animation):

        sim.load_scene("scene.yaml")
        qs = [q0, q1, ..., qn]             # configurations to animate
        anim = sim.animation(qs)                    # interactive backends
        # or headless (no display needed):
        anim = sim.animation(qs, save_as="traj.gif")
    """

    def __init__(self):
        super().__init__()
        self._scene: Optional[Scene] = None

    def load_scene(self, path: str) -> Scene:
        """Load a YAML scene into this simulator (see :func:`load_scene`)."""
        self._scene = load_scene(path, self)
        return self._scene

    def draw_scene(self, axes=None):
        """Draw the loaded scene onto a (3d) matplotlib axes.

        If ``axes`` is given (an ``Axes3D``) it is used directly; otherwise the
        current axes is used and a new 3d axes is created if the current one is
        not 3d. Calling this repeatedly on the same axes only redraws the
        robot (the static scene elements are kept), so it can be used as the
        per-frame update of a matplotlib animation.
        """
        if self._scene is None:
            raise RuntimeError("no scene loaded; call load_scene(path) first")
        from matplotlib import pyplot as plt

        if axes is None:
            from mpl_toolkits.mplot3d.axes3d import Axes3D

            axes = plt.gca()
            if not isinstance(axes, Axes3D):
                axes = plt.axes(projection="3d")
        self._draw_scene(axes)

    def _draw_scene(self, axes):
        static_drawn = getattr(axes, "_tro2022_static_drawn", False)
        scene_id = getattr(axes, "_tro2022_scene_id", None)
        if static_drawn and scene_id == id(self._scene):
            self._redraw_robot(axes)
        else:
            self._clear_data_artists(axes)          # stale static/robot artists
            _draw_points(axes, self._scene, self)
            _draw_planes(axes, self._scene)
            _draw_lines(axes, self._scene)
            axes._tro2022_static_drawn = True
            axes._tro2022_scene_id = id(self._scene)
            self._redraw_robot(axes)

    def _redraw_robot(self, axes):
        """Clear the previous robot artists and redraw the robot (tagged)."""
        self._clear_data_artists(axes, robot_only=True)
        before = {id(o) for o in axes.get_children()}
        _draw_robot(axes, self._scene, self)
        for o in axes.get_children():               # tag the new (robot) artists
            if id(o) not in before:
                o._tro2022_robot = True

    def _clear_data_artists(self, axes, robot_only: bool = False):
        """Remove the artists drawn by ``_draw_robot`` (or all data artists)."""
        from matplotlib.collections import LineCollection, PolyCollection
        from matplotlib.lines import Line2D

        for o in list(axes.get_children()):
            # Line2D also covers 3d lines (Line3D); LineCollection covers the
            # quivers (Line3DCollection); PolyCollection covers plot_surface.
            if isinstance(o, (Line2D, LineCollection, PolyCollection)) and (
                not robot_only or getattr(o, "_tro2022_robot", False)
            ):
                o.remove()

    def animation(self, q, frames_per_step: int = 1, save_as: Optional[str] = None,
                  **anim_kwargs):
        """Animate robot configurations with matplotlib's ``FuncAnimation``.

        ``q`` is a single 1-D configuration, a list/tuple of configurations, or
        a 2-D array of shape ``(n, dim)`` (one configuration per row);
        ``frames_per_step`` repeats each configuration for that many animation
        frames (default 1).

        The static scene is drawn once and each frame only redraws the robot
        (see :meth:`draw_scene`), which is far cheaper than redrawing the whole
        scene per frame and keeps the artist count constant.

        If ``save_as`` is given (e.g. ``"traj.gif"`` or ``"traj.mp4"``) the
        animation is written to that file: GIF always works (PillowWriter),
        other containers use ffmpeg when it is on PATH (FFMpegWriter). ``fps``
        (used only when saving; defaults to ``1000 / interval``) and the
        remaining ``anim_kwargs`` are forwarded to ``FuncAnimation``
        (``interval``, ``blit`` (default False), ...). Returns the
        ``FuncAnimation`` object.
        """
        if self._scene is None:
            raise RuntimeError("no scene loaded; call load_scene(path) first")
        import matplotlib.pyplot as plt
        import matplotlib.animation as manim

        configs = _as_configuration_sequence(q)
        fpp = max(1, int(frames_per_step))
        fps = anim_kwargs.pop("fps", None)        # save()-only keyword

        fig = plt.figure()
        axes = plt.axes(projection="3d")
        self.draw_scene(axes)                       # static scene + first robot

        def _update(frame):
            i = (frame // fpp) % len(configs)
            self.set_configuration_space_positions(configs[i])
            self._redraw_robot(axes)
            return []

        anim_kwargs.setdefault("blit", False)
        anim_kwargs.setdefault("interval", 100)
        anim = manim.FuncAnimation(fig, _update, frames=len(configs) * fpp,
                                   **anim_kwargs)
        if save_as is not None:
            if fps is None:
                fps = 1000.0 / anim_kwargs["interval"]
            anim.save(save_as, writer=_animation_writer(save_as, fps))
        return anim
