---
kernelspec:
  name: python3
  display_name: 'Python 3'
---

# Adaptive Constrained Kinematic Control (Python example)

*License: MIT — see the [LICENSE](../LICENSE) of this repository*

*Author: Murilo M. Marinho (murilo.marinho@manchester.ac.uk)*

## Prerequisites for the learner

This notebook is an **advanced application**. The reader is expected to have a
comfortable working knowledge of

- Forward kinematics of a serial-link manipulator (configuration space
  $\mathbb{R}^n$, task space, dual-quaternion poses);
- The first-order (Jacobian) mapping between joint-space and task-space
  velocities;
- A comfortable use of `numpy` and of the `dqrobotics` package (dual
  quaternions, $SE(3)$ poses).

It implements, and executes in Python, the sample code of

> M. M. Marinho and B. V. Adorno, "Adaptive Constrained Kinematic Control Using
> Partial or Complete Task-Space Measurements," *IEEE Transactions on Robotics*,
> vol. 38, no. 6, pp. 3498--3513, Dec. 2022, doi: 10.1109/TRO.2022.3181047.

## I found an issue

Thank you! Please report it at https://github.com/mmmarinho/tro2022_adaptivecontrol/issues

# What this notebook does

A manipulator is usually controlled with a kinematic model that has to be
**known**. In this notebook we instead start with an *estimated* model whose
parameters are deliberately wrong, and a control law that

1. drives the end-effector to the task-space references $x_d$, and
2. simultaneously **adapts** the estimated model $\hat a$ towards the true
   model, using task-space measurements, while
3. enforcing **collision avoidance** through vector-field inequalities (VFIs).

Everything is self-contained and runs *headlessly* on an in-memory stand-in
simulator (`M3_SimulatorDummy`) that reconstructs the nominal reference scene:
a 6-DoF VS050-style robot, a box workspace (4 walls, 2 tubes) and two task
targets. No external simulator, robot, or ROS is required.

# A note on the dependencies

The example is implemented in this repository (`mmmarinho/tro2022_adaptivecontrol`,
branch `no_vrep`). Unlike most pure-Python packages, this one is **compiled**
(it contains a C++/pybind11 core), so the cells below need it to be built and
installed first. The cell in the next section takes care of it **best-effort**
and never fails:

- it always installs the lightweight dependencies (`numpy`, `matplotlib`);
- it builds and installs the compiled `no_vrep` package when a C++ toolchain
  is available — from **this repository's checkout** (so it always matches the
  version of this notebook), cloning from GitHub only if no checkout is found;
- if the package is already installed, or if the build is not possible
  (for example, when this notebook is being built for a book's website, which
  deliberately skips the multi-minute source build), it simply reports so and
  the example cells are skipped **gracefully** — the notebook still renders and
  executes without errors.

So: locally, following this notebook runs the real example; on a website build
the same notebook is shown with a short notice that the compiled example is not
available there. To run the full example, build the package once as shown in
the *Build the compiled example* section below.

# Installing the dependencies

````{code-cell}
%%capture
%pip install numpy matplotlib
````

Build the compiled example. This is **best-effort** and never raises: it
builds and installs the package only when the toolchain is present and we are
not inside a CI build (website builds of this notebook skip it to stay fast).
It builds **this repository's checkout** (falling back to a shallow clone of
the `no_vrep` branch if no checkout is found). The C++/pybind11 core build
needs a C++ toolchain (`g++`), `cmake`, `ninja`, and the Eigen headers
(`libeigen3-dev` on Debian/Ubuntu).

````{code-cell}
import os
import shutil
import subprocess
import sys
from pathlib import Path

def _in_ci():
    return any(os.environ.get(v) for v in ("CI", "GITHUB_ACTIONS", "JENKINS_URL"))

def _find_repo_root():
    """Locate this repository's checkout (has setup.py + submodules/dqrobotics)."""
    if os.environ.get("TRO2022_ADAPTIVECONTROL_ROOT"):
        return os.environ["TRO2022_ADAPTIVECONTROL_ROOT"]
    try:
        import marinholab.papers.tro2022.adaptive_control as _pkg
        p = Path(_pkg.__file__).resolve().parent
        for _ in range(12):
            if (p / "setup.py").is_file() and (p / "submodules").is_dir():
                return str(p)
            p = p.parent
    except Exception:
        pass
    here = Path.cwd()
    for _ in range(12):
        if (here / "book" / "adaptive_control_tutorial.md").is_file():
            return str(here)
        here = here.parent
    return None

try:
    import marinholab.papers.tro2022.adaptive_control  # noqa: F401
    print("Compiled example package already available; nothing to build.")
    _skip = False
except Exception:
    _skip = True

if _skip and _in_ci():
    print("Detected a CI build: skipping the source build of the compiled example.")
    print("The example cells below will be skipped gracefully.")
    print("To run the example locally, build the package as shown in the next section.")
elif _skip:
    missing = [tool for tool in ("g++", "cmake", "ninja") if shutil.which(tool) is None]
    if missing:
        print(f"Could not build the compiled example: missing {missing}.")
        print("Install them (e.g. `sudo apt install g++ cmake ninja-build libeigen3-dev`) and re-run this cell.")
    else:
        _root = _find_repo_root()
        if _root is not None and Path(_root, "setup.py").is_file():
            _src, _msg = _root, "Building and installing this repository's checkout (a few minutes)..."
        else:
            print("No local checkout found; cloning mmmarinho/tro2022_adaptivecontrol (branch no_vrep)...")
            _src = Path.cwd() / "tro2022_adaptivecontrol_no_vrep"
            subprocess.run(["git", "clone", "--depth", "1", "--recurse-submodules",
                            "-b", "no_vrep",
                            "https://github.com/mmmarinho/tro2022_adaptivecontrol.git",
                            str(_src)], check=True)
            _msg = "Building and installing the cloned no_vrep checkout (a few minutes)..."
        try:
            print(_msg)
            subprocess.run([sys.executable, "-m", "pip", "install", _src], check=True)
            print("Compiled example installed.")
        except subprocess.CalledProcessError:
            print("The build failed. Install the toolchain (g++, cmake, ninja, libeigen3-dev) and try again.")
````

# Build the compiled example (manual)

If the previous cell was skipped (for example, when viewing this notebook on a
website), build the package yourself once, from **this repository's root**
(the directory containing `setup.py`, i.e. the parent of this `book/` folder):

```bash
git clone --recurse-submodules -b no_vrep \
    https://github.com/mmmarinho/tro2022_adaptivecontrol.git
cd tro2022_adaptivecontrol
python3 -m pip install .
```

On Debian/Ubuntu you may first need `sudo apt install g++ cmake ninja-build libeigen3-dev`.
After that, the cells below run unmodified.

# Imports

````{code-cell}
%matplotlib inline
import numpy as np
import matplotlib.pyplot as plt
from numpy import pi, cos, sin

example_available = True
try:
    from dqrobotics import *  # dual quaternions, SE(3)
    from marinholab.papers.tro2022.adaptive_control import *
    import marinholab.papers.tro2022.adaptive_control._core as _core
    # The parameter-space submodule is not re-exported by the package `__init__`,
    # so pull it straight from the compiled core.
    M3_ParameterSpaceEDH = _core._M3_ParameterSpaceEDH
    # The C++ helper deg2rad is not re-exported in Python.
    deg2rad = np.radians
except Exception as _e:  # pragma: no cover - depends on the environment
    example_available = False
    print("The compiled example package could not be imported.")
    print(f"  ({_e})")
    print("Run the 'Build the compiled example' section above and restart the kernel,")
    print("then re-run this and the remaining cells.")

# A fixed seed so the (randomized) initial parameter estimate is reproducible.
np.random.seed(0)
print("example_available =", example_available)
````

# [1] Simulation parameters

The behaviour is somewhat robust to the exact gains, and the values below are
chosen only for this example, not to be optimal in any general sense. The
sampling time is 80 ms (12.5 Hz), matching the physical VS050 default joint
control frequency.

````{code-cell}
if example_available:
    simulation_parameters = Example_SimulationParameters(
        M3_MeasureSpace.Pose,  # measure the full pose
        20.0,                  # proportional_gain
        5,                     # vfi_gain
        0.02,                  # vfi_weight
        0.01,                  # damping
        0.08,                  # sampling_time_sec
        60.0,                  # reference_timeout_sec
    )
    print("sampling time:", simulation_parameters.sampling_time_sec, "s")
    print("measure space:", simulation_parameters.measure_space)
````

# [2] Robot and models

We load the reference scene, then build two kinematic models:

- `real_robot`, with the *ideal* (true) parameters; it represents the real
  robot, and produces the task-space **measurements** $y$;
- `estimated_robot`, with the parameters we estimate and adapt; it is the
  model used to compute the end-effector pose $\hat x$ and the control signal.

Both share the same VS050 joint structure; only the parameters differ.

````{code-cell}
if example_available:
    vi = M3_SimulatorDummy()
    vi.load_reference_scene()
    print("scene objects:", ", ".join(sorted(vi.get_object_names())))

    # Initial configuration (a plausible pose), in radians.
    q_init = np.asarray(vi.get_configuration_space_positions(), dtype=float)
    print("q_init =", np.round(q_init, 3))

    # Ideal base and effector frames.
    real_base_frame = vi.get_object_pose("VS050_reference_frame")
    r = cos(-pi / 4.0) + i_ * sin(-pi / 4.0)
    effector_frame = r + 0.5 * E_ * k_ * 0.15688 * r

    # Two independent instances of the same kinematic structure.
    real_robot = M3_SimulatorDummy.vs050_raw_kinematics()
    real_robot.set_base_frame(real_base_frame)
    real_robot.set_effector_frame(effector_frame)

    estimated_robot = M3_SimulatorDummy.vs050_raw_kinematics()
    estimated_robot.set_base_frame(real_base_frame)
    estimated_robot.set_effector_frame(effector_frame)

    print("configuration space (joints):", estimated_robot.get_dim_configuration_space())
    print("parameter space (dimensions):", estimated_robot.get_dim_parameter_space())
````

# [3] Parameter-space confidence bounds

The adaptive controller needs, for every parameter, a confidence interval
(lower and upper bound) around its current value. We give the base and
effector frames a *wider* confidence than the link parameters, reflecting that
a base calibration error is usually larger.

````{code-cell}
if example_available:
    def set_parameter_space_boundaries(
            robot,
            base_linear_confidence_meters=0.1,
            base_angular_confidence_degrees=20,
            effector_linear_confidence_meters=0.01,
            effector_angular_confidence_degrees=5,
            other_parameters_linear_confidence_meters=0.001,
            other_parameters_angular_confidence_degrees=1):
        bl = base_linear_confidence_meters
        ba = base_angular_confidence_degrees
        el = effector_linear_confidence_meters
        ea = effector_angular_confidence_degrees
        opl = other_parameters_linear_confidence_meters
        opa = other_parameters_angular_confidence_degrees
        bp = robot.get_base_parameters()
        ep = robot.get_effector_parameters()
        P = M3_ParameterSpaceEDH.Example_ParameterType
        parameter_space = [
            M3_ParameterSpaceEDH.Example_Parameter(-1, P.base_x, bp[0].value_, bp[0].value_ - bl, bp[0].value_ + bl),
            M3_ParameterSpaceEDH.Example_Parameter(-1, P.base_y, bp[1].value_, bp[1].value_ - bl, bp[1].value_ + bl),
            M3_ParameterSpaceEDH.Example_Parameter(-1, P.base_z, bp[2].value_, bp[2].value_ - bl, bp[2].value_ + bl),
            M3_ParameterSpaceEDH.Example_Parameter(-1, P.base_alpha, bp[3].value_, bp[3].value_ - deg2rad(ba), bp[3].value_ + deg2rad(ba)),
            M3_ParameterSpaceEDH.Example_Parameter(-1, P.base_beta, bp[4].value_, bp[4].value_ - deg2rad(ba), bp[4].value_ + deg2rad(ba)),
            M3_ParameterSpaceEDH.Example_Parameter(-1, P.base_gamma, bp[5].value_, bp[5].value_ - deg2rad(ba), bp[5].value_ + deg2rad(ba)),
        ]
        for li in range(6):
            parameter_space += [
                M3_ParameterSpaceEDH.Example_Parameter(li, P.theta, robot.get_theta(li), robot.get_theta(li) - deg2rad(opa), robot.get_theta(li) + deg2rad(opa)),
                M3_ParameterSpaceEDH.Example_Parameter(li, P.d, robot.get_d(li), robot.get_d(li) - opl, robot.get_d(li) + opl),
                M3_ParameterSpaceEDH.Example_Parameter(li, P.a, robot.get_a(li), robot.get_a(li) - opl, robot.get_a(li) + opl),
                M3_ParameterSpaceEDH.Example_Parameter(li, P.alpha, robot.get_alpha(li), robot.get_alpha(li) - deg2rad(opa), robot.get_alpha(li) + deg2rad(opa)),
            ]
        parameter_space += [
            M3_ParameterSpaceEDH.Example_Parameter(6, P.eff_x, ep[0].value_, ep[0].value_ - el, ep[0].value_ + el),
            M3_ParameterSpaceEDH.Example_Parameter(6, P.eff_y, ep[1].value_, ep[1].value_ - el, ep[1].value_ + el),
            M3_ParameterSpaceEDH.Example_Parameter(6, P.eff_z, ep[2].value_, ep[2].value_ - el, ep[2].value_ + el),
            M3_ParameterSpaceEDH.Example_Parameter(6, P.eff_alpha, ep[3].value_, ep[3].value_ - deg2rad(ea), ep[3].value_ + deg2rad(ea)),
            M3_ParameterSpaceEDH.Example_Parameter(6, P.eff_beta, ep[4].value_, ep[4].value_ - deg2rad(ea), ep[4].value_ + deg2rad(ea)),
            M3_ParameterSpaceEDH.Example_Parameter(6, P.eff_gamma, ep[5].value_, ep[5].value_ - deg2rad(ea), ep[5].value_ + deg2rad(ea)),
        ]
        robot.set_parameter_space(parameter_space)

    set_parameter_space_boundaries(real_robot)
    set_parameter_space_boundaries(estimated_robot)
    parameter_boundaries = estimated_robot.get_parameter_space_boundaries()

    # A (simplified) joint-velocity limit, which also keeps the robot moving
    # slowly enough to be comfortable to watch.
    ROBOT_JOINT_VELOCITY_LIMIT = 0.1
    n_q = estimated_robot.get_dim_configuration_space()
    estimated_robot.set_upper_q_dot_limit(np.full(n_q, ROBOT_JOINT_VELOCITY_LIMIT))
    estimated_robot.set_lower_q_dot_limit(np.full(n_q, -ROBOT_JOINT_VELOCITY_LIMIT))

    print("parameter-space dimension:", estimated_robot.get_dim_parameter_space())
````

# [4] Vector-field inequalities (collision avoidance)

The box workspace is expressed as VFIs that the robot must satisfy. Each VFI
is a *sphere* attached to one of the robot's links (the `tool_sphere_i`
objects) kept a safe distance away from a *wall* (a plane) or a *tube* (a
line). We build 4 wall VFIs and 2 tube VFIs per sphere, i.e. 36 VFIs.

````{code-cell}
if example_available:
    # Keep the scene's end-effector reference and the tool spheres consistent
    # with the control robot's kinematics, so the VFI distances line up with
    # the real end-effector pose.
    x_hat = estimated_robot.fkm(q_init)
    vi.set_object_pose("x_hat", x_hat)
    for link_index in range(6):
        vi.set_object_pose(f"tool_sphere_{link_index + 1}",
                           estimated_robot.fkm(q_init, link_index))

    x_hat = vi.get_object_pose("x_hat")
    # (reference pose relative to the end-effector, sphere radius, name)
    vfi_reference_dqs = [
        (conj(x_hat) * vi.get_object_pose("tool_sphere_1"), 0.04, "tool_sphere_1"),
        (conj(x_hat) * vi.get_object_pose("tool_sphere_2"), 0.015, "tool_sphere_2"),
        (conj(x_hat) * vi.get_object_pose("tool_sphere_3"), 0.015, "tool_sphere_3"),
        (conj(x_hat) * vi.get_object_pose("tool_sphere_4"), 0.015, "tool_sphere_4"),
        (conj(x_hat) * vi.get_object_pose("tool_sphere_5"), 0.015, "tool_sphere_5"),
        (conj(x_hat) * vi.get_object_pose("tool_sphere_6"), 0.075, "tool_sphere_6"),
    ]

    vfis = []
    tube_distance = 0.02
    wall_distance = 0.02
    for (ref_dq, radius, sphere_name) in vfi_reference_dqs:
        for wall in ("cube_40x40_wall_1", "cube_40x40_wall_2",
                     "cube_40x40_wall_3", "cube_40x40_wall_4"):
            vfis.append(M3_VFI(wall, sphere_name, M3_Primitive.Plane, vi,
                               radius + wall_distance,
                               M3_VFI_Direction.FORBIDDEN_ZONE, 7, ref_dq, ""))
        for tube in ("cube_40x40_tube_1", "cube_40x40_tube_2"):
            vfis.append(M3_VFI(tube, sphere_name, M3_Primitive.Line, vi,
                               (radius + tube_distance) ** 2,
                               M3_VFI_Direction.FORBIDDEN_ZONE, 7, ref_dq, ""))
    for vfi in vfis:
        vfi.initialize()

    print("number of VFIs:", len(vfis))
````

# [5] Make the initial estimate wrong (but plausible)

The whole point of the example is that the estimated model starts out wrong.
We randomize the parameters uniformly inside their confidence bounds, and
reject any candidate that would already be penetrating an obstacle, so the
initial pose is safe.

````{code-cell}
import time

if example_available:
    def randomize_parameters(estimated_robot, parameter_boundaries, q, vfis, timeout=10.0):
        t0 = time.perf_counter()
        counter = 0
        while True:
            if time.perf_counter() - t0 > timeout:
                raise RuntimeError("Timeout in finding suitable initial parameters")
            counter += 1
            n_p = estimated_robot.get_dim_parameter_space()
            noise_weights = (np.random.rand(n_p) + 1.0) * 0.5
            lo = np.asarray(parameter_boundaries[0], dtype=float)
            hi = np.asarray(parameter_boundaries[1], dtype=float)
            noise_affected = noise_weights * lo + (1.0 - noise_weights) * hi
            estimated_robot.set_parameter_space_values(noise_affected)
            x_hat = estimated_robot.fkm(q)
            found = True
            for vfi in vfis:
                de = vfi.get_distance_error(x_hat)
                dt_ = vfi.get_distance_type()
                if dt_ == M3_VFI_DistanceType.EUCLIDEAN and de < -0.001:
                    found = False
                    break
                if dt_ == M3_VFI_DistanceType.EUCLIDEAN_SQUARED and de < -0.00001:
                    found = False
                    break
            print(f"  Finding suitable parameters {{t={time.perf_counter() - t0:.3f}s}}, tries={counter}.")
            if found:
                break

    randomize_parameters(estimated_robot, parameter_boundaries, q_init, vfis)
    a_hat_init = np.asarray(estimated_robot.get_parameter_space_values(), dtype=float)
    print("initial (wrong) parameter estimate has been set.")
````

# [6] and [7] Run the control loop, with and without adaptation

We run the *same* scenario twice:

- with `FULL` adaptation (task control **and** parameter adaptation);
- with `TASK_ONLY` (task control only, no adaptation).

The original example runs in real time for the full reference timeout. To keep
this notebook fast and, more importantly, to *see* what is happening, we instead
run a fixed number of sampling steps and record the trajectories. This is a
faithful, step-bounded version of the same loop.

````{code-cell}
if example_available:
    adaptive_controller = M3_AdaptiveController(estimated_robot, simulation_parameters)
    vi.start_simulation()

    xds = [vi.get_object_pose("xd0"),  # safe approach reference (going)
           vi.get_object_pose("xd1")]   # final (intentionally unreachable) target

    def translation_distance(a, b):
        """Euclidean distance between the translations of two unit poses."""
        d = translation(a) - translation(b)
        return float(np.linalg.norm(np.asarray(d.q[1:4])))

    def run_strategy(strategy, n_steps_per_target):
        """Run the closed loop for n_steps_per_target steps on each target and
        record the trajectories. No real-time sleeping, so it runs instantly."""
        q = np.array(q_init, dtype=float)
        vi.set_configuration_space_positions(q)
        estimated_robot.set_parameter_space_values(a_hat_init)
        a_hat = np.array(a_hat_init, dtype=float)
        dt = simulation_parameters.sampling_time_sec

        task_error = []      # || x_tilde ||  (task pose error)
        est_real_error = []  # | translation(x_hat) - translation(y) |
        parameter_change = []  # || a_hat - a_hat_init ||

        for xd in xds:
            for _ in range(n_steps_per_target):
                x_hat = estimated_robot.fkm(q)
                vi.set_object_pose("x_hat", x_hat)
                y = real_robot.fkm(q)
                vi.set_object_pose("x", y)

                uq, ua, x_tilde, y_tilde, y_partial = \
                    adaptive_controller.compute_setpoint_control_signal(
                        strategy, q, xd, y, vfis)

                a_hat = a_hat + np.asarray(ua, dtype=float) * dt
                estimated_robot.set_parameter_space_values(a_hat)
                q = q + np.asarray(uq, dtype=float) * dt
                vi.set_configuration_space_positions(q)

                task_error.append(float(np.linalg.norm(np.asarray(x_tilde))))
                est_real_error.append(
                    translation_distance(x_hat, y) if is_unit(y) else float("nan"))
                parameter_change.append(float(np.linalg.norm(a_hat - a_hat_init)))

        return (np.array(task_error), np.array(est_real_error),
                np.array(parameter_change), np.array(a_hat))

    N_STEPS_PER_TARGET = 400  # 400 * 0.08 s = 32 s of simulated time, per target

    full_result = run_strategy(Example_AdaptiveControlStrategy.FULL, N_STEPS_PER_TARGET)
    task_result = run_strategy(Example_AdaptiveControlStrategy.TASK_ONLY, N_STEPS_PER_TARGET)
    vi.stop_simulation()

    print(f"FULL       : final parameter change = "
          f"{full_result[2][-1]:.4g}   (adaptation is active)")
    print(f"TASK_ONLY  : final parameter change = "
          f"{task_result[2][-1]:.4g}   (no adaptation, estimate frozen)")
````

# [8] What did adaptation do?

The three panels compare the run **with** adaptation (FULL) against the run
**without** adaptation (TASK_ONLY). The vertical dashed line separates the two
task targets.

- **Parameter change** $\|\hat a - \hat a_0\|$: only the FULL run moves the
  estimated parameters, and it does so towards the true model.
- **Estimated vs. real end-effector translation error**: the gap between what
  the (estimated) model predicts and what the robot actually does shrinks under
  adaptation, because the estimate improves.
- **Task pose error**: both runs steer the robot; note the final target is
  *on purpose* unreachable (the robot cannot reach it), so this error does not
  go to zero. The value of the method is that the robot still moves without
  colliding and the model converges.

````{code-cell}
if example_available:
    T = np.arange(full_result[0].size) * simulation_parameters.sampling_time_sec

    fig, axes = plt.subplots(1, 3, figsize=(13, 3.6))
    panels = (
        (axes[0], full_result[0], task_result[0], "Task pose error"),
        (axes[1], full_result[1], task_result[1], "Estimated vs real EE translation error (m)"),
        (axes[2], full_result[2], task_result[2], "Parameter change magnitude"),
    )
    for ax, data_full, data_task, title in panels:
        ax.plot(T, data_full, label="FULL (adaptation)")
        ax.plot(T, data_task, label="TASK_ONLY (no adaptation)")
        ax.axvline(N_STEPS_PER_TARGET * simulation_parameters.sampling_time_sec,
                   color="grey", lw=0.7, ls="--")
        ax.set_title(title)
        ax.set_xlabel("simulated time (s)")
        ax.legend(fontsize=8)
    fig.tight_layout()
    plt.show()
````

# Summary

In this notebook we executed, in Python, the adaptive constrained kinematic
control example of Marinho & Adorno (TRO 2022):

- a 6-DoF robot is driven through a box to two task-space references by an
  estimated kinematic model;
- the model's parameters start out deliberately wrong and are **adapted**
  online from task-space measurements;
- collision avoidance is enforced with vector-field inequalities;
- comparing the run with and without adaptation makes it clear that adaptation
  moves the estimated model towards the true model and reduces the
  estimate-vs-reality error, while the robot avoids the obstacles.

Everything ran headlessly on the in-memory `M3_SimulatorDummy`; no simulator,
robot, or ROS was required.
