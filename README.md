# Adaptive Constrained Kinematic Control using Partial or Complete Task-Space Measurements

## Python

### venv

```commandline
    python3 -m venv venv
    source venv/bin/activate
    python3 -m pip install dqrobotics --pre
    python3 -m pip install marinholab-papers-tro2022-adaptivecontrol
```

### When you cannot use a venv (e.g. ROS2)

```commandline
    python3 -m pip install dqrobotics --pre --break-system-packages
    python3 -m pip install marinholab-papers-tro2022-adaptivecontrol --break-system-packages
```

## Python example (with a pyplot figure)

The example below runs the adaptive constrained kinematic control loop
end-to-end, headlessly, and draws a figure with `matplotlib.pyplot` that
compares the run **with** adaptation (`FULL`) against the run **without** it
(`TASK_ONLY`). It is a self-contained, step-bounded version of the same loop
the C++ example runs — no external simulator, robot, or ROS is required.

A longer, fully-commented, cell-by-cell version of this exact example (with
inline explanations and the dependency build steps) is the notebook in
[`book/adaptive_control_tutorial.md`](book/adaptive_control_tutorial.md).

What the example does:

- A 6-DoF VS050-style robot is driven through a box workspace toward two
  task-space target poses, using an *estimated* kinematic model.
- The estimated model's parameters start out **deliberately wrong** (they are
  randomized inside their confidence bounds) and are **adapted online** from
  task-space measurements.
- Collision avoidance is enforced with vector-field inequalities (VFIs).
- The final target pose is, on purpose, unreachable, to show that the robot
  still moves without colliding while the model converges.

Save the snippet as `example.py` and run `python3 example.py` (a
`matplotlib` installation is required; `dqrobotics` and this package must be
installed as above):

```python
import time
import numpy as np
from numpy import pi, cos, sin
import matplotlib
matplotlib.use("Agg")               # headless backend; drop this line to plot on screen
import matplotlib.pyplot as plt

from dqrobotics import *
from marinholab.papers.tro2022.adaptive_control import *
import marinholab.papers.tro2022.adaptive_control._core as _core

# The parameter-space submodule is not re-exported by the package `__init__`,
# so pull it straight from the compiled core.
ParameterSpaceEDH = _core._ParameterSpaceEDH
deg2rad = np.radians

# A fixed seed so the (randomized) initial parameter estimate is reproducible.
np.random.seed(0)

# --- [1] Simulation parameters ----------------------------------------------
# Sampling time 0.08 s (12.5 Hz), the VS050 default joint control frequency.
simulation_parameters = Example_SimulationParameters(
    MeasureSpace.Pose,   # measure the full pose (pose, not just rotation/translation)
    20.0,                # proportional_gain
    5,                   # vfi_gain
    0.02,                # vfi_weight
    0.01,                # damping
    0.08,                # sampling_time_sec
    60.0,                # reference_timeout_sec
)

# --- [2] Robot and models ----------------------------------------------------
# Load the in-memory reference scene (robot, box workspace, two targets).
vi = SimulatorDummy()
vi.load_reference_scene()
q_init = np.asarray(vi.get_configuration_space_positions(), dtype=float)

# Ideal base and effector frames (shared by both models).
real_base_frame = vi.get_object_pose("VS050_reference_frame")
r = cos(-pi / 4.0) + i_ * sin(-pi / 4.0)
effector_frame = r + 0.5 * E_ * k_ * 0.15688 * r

# `real_robot` has the *true* parameters and produces the task-space
# measurements `y`; `estimated_robot` is the model we adapt and use for the
# end-effector pose `x_hat` and the control signal.
real_robot = SimulatorDummy.vs050_raw_kinematics()
real_robot.set_base_frame(real_base_frame)
real_robot.set_effector_frame(effector_frame)
estimated_robot = SimulatorDummy.vs050_raw_kinematics()
estimated_robot.set_base_frame(real_base_frame)
estimated_robot.set_effector_frame(effector_frame)

# --- [3] Parameter-space confidence bounds -----------------------------------
# The adaptive controller needs a confidence interval (min/max) around every
# parameter. Base/effector frames get a wider confidence than the links.
def set_parameter_space_boundaries(robot,
        bl=0.1, ba=20, el=0.01, ea=5, opl=0.001, opa=1):
    bp = robot.get_base_parameters(); ep = robot.get_effector_parameters()
    P = ParameterSpaceEDH.Example_ParameterType
    ps = [
        ParameterSpaceEDH.Example_Parameter(-1, P.base_x,     bp[0].value_, bp[0].value_ - bl, bp[0].value_ + bl),
        ParameterSpaceEDH.Example_Parameter(-1, P.base_y,     bp[1].value_, bp[1].value_ - bl, bp[1].value_ + bl),
        ParameterSpaceEDH.Example_Parameter(-1, P.base_z,     bp[2].value_, bp[2].value_ - bl, bp[2].value_ + bl),
        ParameterSpaceEDH.Example_Parameter(-1, P.base_alpha, bp[3].value_, bp[3].value_ - deg2rad(ba), bp[3].value_ + deg2rad(ba)),
        ParameterSpaceEDH.Example_Parameter(-1, P.base_beta,  bp[4].value_, bp[4].value_ - deg2rad(ba), bp[4].value_ + deg2rad(ba)),
        ParameterSpaceEDH.Example_Parameter(-1, P.base_gamma, bp[5].value_, bp[5].value_ - deg2rad(ba), bp[5].value_ + deg2rad(ba)),
    ]
    for li in range(6):  # one link each: theta, d, a, alpha
        ps += [
            ParameterSpaceEDH.Example_Parameter(li, P.theta,  robot.get_theta(li),  robot.get_theta(li)  - deg2rad(opa), robot.get_theta(li)  + deg2rad(opa)),
            ParameterSpaceEDH.Example_Parameter(li, P.d,      robot.get_d(li),      robot.get_d(li)      - opl, robot.get_d(li)      + opl),
            ParameterSpaceEDH.Example_Parameter(li, P.a,      robot.get_a(li),      robot.get_a(li)      - opl, robot.get_a(li)      + opl),
            ParameterSpaceEDH.Example_Parameter(li, P.alpha,  robot.get_alpha(li),  robot.get_alpha(li)  - deg2rad(opa), robot.get_alpha(li)  + deg2rad(opa)),
        ]
    ps += [
        ParameterSpaceEDH.Example_Parameter(6, P.eff_x,      ep[0].value_, ep[0].value_ - el, ep[0].value_ + el),
        ParameterSpaceEDH.Example_Parameter(6, P.eff_y,      ep[1].value_, ep[1].value_ - el, ep[1].value_ + el),
        ParameterSpaceEDH.Example_Parameter(6, P.eff_z,      ep[2].value_, ep[2].value_ - el, ep[2].value_ + el),
        ParameterSpaceEDH.Example_Parameter(6, P.eff_alpha,  ep[3].value_, ep[3].value_ - deg2rad(ea), ep[3].value_ + deg2rad(ea)),
        ParameterSpaceEDH.Example_Parameter(6, P.eff_beta,   ep[4].value_, ep[4].value_ - deg2rad(ea), ep[4].value_ + deg2rad(ea)),
        ParameterSpaceEDH.Example_Parameter(6, P.eff_gamma,  ep[5].value_, ep[5].value_ - deg2rad(ea), ep[5].value_ + deg2rad(ea)),
    ]
    robot.set_parameter_space(ps)

set_parameter_space_boundaries(real_robot)
set_parameter_space_boundaries(estimated_robot)
parameter_boundaries = estimated_robot.get_parameter_space_boundaries()

# A (simplified) joint-velocity limit; also keeps the robot moving slowly.
n_q = estimated_robot.get_dim_configuration_space()
estimated_robot.set_upper_q_dot_limit(np.full(n_q, 0.1))
estimated_robot.set_lower_q_dot_limit(np.full(n_q, -0.1))

# --- [4] Vector-field inequalities (collision avoidance) ---------------------
# Each VFI keeps a "tool sphere" attached to a link a safe distance from a
# wall (plane) or a tube (line): 4 walls + 2 tubes per sphere => 36 VFIs.
x_hat = estimated_robot.fkm(q_init)
vi.set_object_pose("x_hat", x_hat)
for link_index in range(6):
    vi.set_object_pose(f"tool_sphere_{link_index + 1}",
                       estimated_robot.fkm(q_init, link_index))
x_hat = vi.get_object_pose("x_hat")
vfi_reference_dqs = [
    (conj(x_hat) * vi.get_object_pose("tool_sphere_1"), 0.04,  "tool_sphere_1"),
    (conj(x_hat) * vi.get_object_pose("tool_sphere_2"), 0.015, "tool_sphere_2"),
    (conj(x_hat) * vi.get_object_pose("tool_sphere_3"), 0.015, "tool_sphere_3"),
    (conj(x_hat) * vi.get_object_pose("tool_sphere_4"), 0.015, "tool_sphere_4"),
    (conj(x_hat) * vi.get_object_pose("tool_sphere_5"), 0.015, "tool_sphere_5"),
    (conj(x_hat) * vi.get_object_pose("tool_sphere_6"), 0.075, "tool_sphere_6"),
]
vfis = []
for ref_dq, radius, sphere_name in vfi_reference_dqs:
    for wall in ("cube_40x40_wall_1", "cube_40x40_wall_2",
                 "cube_40x40_wall_3", "cube_40x40_wall_4"):
        vfis.append(VFI(wall, sphere_name, Primitive.Plane, vi,
                        radius + 0.02, VFI_Direction.FORBIDDEN_ZONE, 7, ref_dq, ""))
    for tube in ("cube_40x40_tube_1", "cube_40x40_tube_2"):
        vfis.append(VFI(tube, sphere_name, Primitive.Line, vi,
                        (radius + 0.02) ** 2, VFI_Direction.FORBIDDEN_ZONE, 7, ref_dq, ""))
for vfi in vfis:
    vfi.initialize()

# --- [5] Make the initial estimate wrong (but collision-free) ----------------
# Randomize the parameters uniformly inside their confidence bounds, and
# reject any candidate that already penetrates an obstacle.
t0 = time.time(); tries = 0
while True:
    if time.time() - t0 > 10:
        raise RuntimeError("Timeout in finding suitable initial parameters")
    tries += 1
    n_p = estimated_robot.get_dim_parameter_space()
    nw = (np.random.rand(n_p) + 1.0) * 0.5
    lo = np.asarray(parameter_boundaries[0], dtype=float)
    hi = np.asarray(parameter_boundaries[1], dtype=float)
    estimated_robot.set_parameter_space_values(nw * lo + (1.0 - nw) * hi)
    x_hat = estimated_robot.fkm(q_init)
    ok = True
    for vfi in vfis:
        de = vfi.get_distance_error(x_hat); dt_ = vfi.get_distance_type()
        if dt_ == VFI_DistanceType.EUCLIDEAN and de < -0.001: ok = False; break
        if dt_ == VFI_DistanceType.EUCLIDEAN_SQUARED and de < -0.00001: ok = False; break
    if ok:
        break
a_hat_init = np.asarray(estimated_robot.get_parameter_space_values(), dtype=float)
print(f"initial (wrong) parameter estimate set after {tries} tries")

# --- [6]/[7] Control loop, with and without adaptation -----------------------
adaptive_controller = AdaptiveController(estimated_robot, simulation_parameters)
vi.start_simulation()
xds = [vi.get_object_pose("xd0"),   # safe approach reference (going)
       vi.get_object_pose("xd1")]   # final (intentionally unreachable) target

def translation_distance(a, b):
    """Euclidean distance between the translations of two unit poses."""
    d = translation(a) - translation(b)
    return float(np.linalg.norm(np.asarray(d.q[1:4])))

def run_strategy(strategy, n_steps_per_target):
    """Run the closed loop for n_steps_per_target steps on each target and
    record the trajectories (no real-time sleeping, so it runs instantly)."""
    q = np.array(q_init, dtype=float)
    vi.set_configuration_space_positions(q)
    estimated_robot.set_parameter_space_values(a_hat_init)
    a_hat = np.array(a_hat_init, dtype=float)
    dt = simulation_parameters.sampling_time_sec
    task_error, est_real_error, parameter_change = [], [], []
    for xd in xds:
        for _ in range(n_steps_per_target):
            x_hat = estimated_robot.fkm(q); vi.set_object_pose("x_hat", x_hat)
            y = real_robot.fkm(q);         vi.set_object_pose("x", y)
            # (joint velocity, parameter update, task error, full meas err, partial)
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
    return np.array(task_error), np.array(est_real_error), np.array(parameter_change)

N_STEPS_PER_TARGET = 400   # 400 * 0.08 s = 32 s of simulated time, per target
full = run_strategy(Example_AdaptiveControlStrategy.FULL, N_STEPS_PER_TARGET)
task = run_strategy(Example_AdaptiveControlStrategy.TASK_ONLY, N_STEPS_PER_TARGET)
vi.stop_simulation()

print(f"FULL       : final parameter change = {full[2][-1]:.4g}   (adaptation is active)")
print(f"TASK_ONLY  : final parameter change = {task[2][-1]:.4g}   (no adaptation, estimate frozen)")

# --- [8] What did adaptation do? ---------------------------------------------
# Three panels compare the FULL run against the TASK_ONLY run. The dashed
# vertical line separates the two task targets.
T = np.arange(full[0].size) * simulation_parameters.sampling_time_sec
fig, axes = plt.subplots(1, 3, figsize=(13, 3.6))
panels = (
    (axes[0], full[0], task[0], "Task pose error"),
    (axes[1], full[1], task[1], "Estimated vs real EE translation error (m)"),
    (axes[2], full[2], task[2], "Parameter change magnitude"),
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
```

**Reading the figure** (with the fixed seed above it is deterministic):

- **Task pose error** — both runs steer the robot. The final target is *on
  purpose* unreachable, so this error does **not** go to zero; that is expected.
- **Estimated vs real EE translation error** — the gap between what the
  (estimated) model predicts and what the robot actually does. It shrinks under
  adaptation because the estimate improves.
- **Parameter change magnitude** $\|\hat a - \hat a_0\|$ — only the `FULL` run
  moves the estimated parameters (the `TASK_ONLY` line stays flat at 0), and it
  does so toward the true model. That is the whole point of the example.

## Reference

Sample code and minimal example for [our TRO2022 paper](https://doi.org/10.1109/TRO.2022.3181047).

```bib
@Article{marinhoandadorno2022adaptive,
  author       = {Marinho, M. M. and Adorno, B. V.},
  title        = {Adaptive Constrained Kinematic Control using Partial or Complete Task-Space Measurements},
  journal      = {IEEE Transactions on Robotics (T-RO)},
  year         = {2022},
  month        = dec,
  doi          = {10.1109/TRO.2022.3181047},
  volume       = {38},
  number       = {6},
  pages        = {3498--3513}
}
```

## C++ code layout & namespaces

All C++ declarations (headers under `include/marinholab/papers/tro2022/adaptive_control/`
and sources under `src/example/`) live in the namespace

```cpp
marinholab::papers::tro2022::adaptive_control
```

which mirrors the Python package path `marinholab.papers.tro2022.adaptive_control`.
The identifiers no longer carry the legacy `M3_` prefix; each class is named
after the package path instead:

| `marinholab::papers::tro2022::adaptive_control::` | File |
|---|---|
| `AdaptiveController` | `AdaptiveController.{h,cpp}` |
| `SerialManipulatorEDH` (with nested `ParameterSpaceEDH`) | `SerialManipulatorEDH.{h,cpp}` |
| `SimulatorDummy` (headless, in-memory scene) | `SimulatorDummy.{h,cpp}` |
| `VFI` (with `Primitive`, `VFI_Direction`, `VFI_DistanceType`) | `VFI.{h,cpp}` |
| `MeasureSpace` (with `get_measure_space_dimension`) | `MeasurementSpace.{h,cpp}` |

The free helpers the example relies on (`get_variable_boundary_inequalities`,
`closest_invariant_error`, `get_example_scene_vfis`, `randomize_parameters`,
`set_parameter_space_boundaries`) are in the same namespace; the standalone
example (`src/adaptive_control_example.cpp`) pulls it in with
`using namespace marinholab::papers::tro2022::adaptive_control;`.

**The Python API uses the same names as the C++ classes.** The pybind11
module exposes `SerialManipulatorEDH`, `SimulatorDummy`, `AdaptiveController`,
`VFI`, `MeasureSpace`, `Primitive`, `VFI_Direction`, `VFI_DistanceType`
(and the `_ParameterSpaceEDH` submodule with `Example_Parameter` /
`Example_ParameterType`), so there is a 1:1 match between the C++ and Python
layers. Note: this dropped the historical `M3_*` Python names — consumers
that used them need a mechanical `M3_` removal (the `book/` tutorial and
`adaptive_control_import_eval.py` have been updated accordingly).

## Standalone Example

- The estimated robot model starts out **on purpose** very wrong, to evaluate the adaptation.
- The estimation usually converges within a few seconds using measurements from a simulated sensor.
- Simultaneously, the robot proceeds through the box toward the target poses, without collisions.
- The example runs **headless** on the in-memory stand-in simulator (`SimulatorDummy`), so no external simulator is required.
- You can change the pose of the `xd0` and `xd1` target objects (see `SimulatorDummy::load_reference_scene`), as long as you do it **before** the simulation starts.

The paper's original demonstration (recorded with the robot model in the GUI):

https://github.com/mmmarinho/tro2022_adaptivecontrol/assets/46012516/2abe0b0b-6e48-46e9-9a86-061ba013b355

## Usage

> **Note (stale):** the pre-compiled download below is release
> `v23.05.1` from **2023-05-26**, which predates the 2025.05 external-simulator
> removal, the 2026.08 headless `SimulatorDummy` change, and the 2026.09
> `M3_`/namespace refactor. It is not current with the repository, and
> **Build from source** is the supported way to run the example as it stands
> today.

### Download & extract the standalone version (only do this once)
```bash
cd ~
sudo apt install curl jq -y
wget $(curl -sL https://api.github.com/repos/mmmarinho/tro2022_adaptivecontrol/releases/latest | jq -r '.assets[].browser_download_url')
tar -xvf tro2022_adaptivecontrol_example.tar.xz
```
### Running

```bash
cd ~/tro2022_adaptivecontrol_example
./run_example.sh
```
### Troubleshooting

If the pre-compiled example fails with a `GLIBC`/`GLIBCXX` version error, please use `Ubuntu 22.04` or later, or build from source (below).

## Known limitations *of this example*/*TODO* list/*Extra info*

- The stopping criterion is elapsed time, so it might not converge for all initial parameters.
- The initial convergence to measurements mentioned in the paper *TODO* for this example.
- The estimated model is randomized so it might start in an implausible zone. Fixing this is *TODO* for this example.
- Sample code for partial measurements is included, but they have not been tested in this example, only in the physical robot.
The adaptation is supposed to move the parameters of the `estimated_robot` towards the ideal kinematic model defined by `real_robot` in the code.
- A different solver was used in the paper's experiments; in this example we use an open-source solver, so the behavior might be somewhat different.
- The final target position is, **ON PURPOSE**, chosen as somewhere the robot cannot reach. It serves to show that even in such case the robot does not collide with the environment.

## Build from source

### Ubuntu

```bash
sudo apt install g++ cmake git libeigen3-dev
```

### `macos`
```bash
brew install cmake eigen
```

### Download the repo

```bash
cd ~
mkdir git
cd git
git clone https://github.com/mmmarinho/tro2022_adaptivecontrol.git --recursive
```

### Build

With all dependencies correctly configured,

```bash
cd ~/git/tro2022_adaptivecontrol
chmod +x .build.sh
./.build.sh
```

## Running

The example runs headless on the in-memory stand-in simulator (`SimulatorDummy`):

```bash
cd ~/git/tro2022_adaptivecontrol
chmod +x .run.sh
./.run.sh
```

## Example console output of the results

Running on an 8-core Ubuntu VM. The example runs the scenario **twice** — once
with full adaptation (`[6]`) and once without it (`[7]`) — so you can compare
the final task errors. The exact numbers vary from run to run: the initial
parameter estimate is randomized and there is **no fixed seed** (unlike the
seeded Python example above). What is stable is the *structure*, shown below
(the per-attempt `Finding suitable parameters …` and occasional `… estimated
penetration: …` lines are elided with `...`):

```console
[1] Loading the reference scene (in-memory simulator)...
[2] Initializing robot and models...
[3] Initializing xd...
[4] Initializing VFIs...
[5] Making our initial parameter estimate wrong, but plausible...
...
[6] Running with full adaptation.
...
Reference timeout for xd0
  Average computational time = 0.00223711 seconds.
  Clock overruns = 0 (Too many, i.e. hundreds, indicate that the sampling time is too low for this CPU).
  Final task pose error norm = 0.793335 (Dual quaternion norm).
  Final task translation error norm = 0.563789 (in meters).
  Final measurement error norm = 0.240959 (Dual quaternion norm).
  Final measurement translation error norm = 0.159095 (in meters).
Reference timeout for xd1
  Average computational time = 0.00247141 seconds.
  Clock overruns = 0 (Too many, i.e. hundreds, indicate that the sampling time is too low for this CPU).
  Final task pose error norm = 0.742961 (Dual quaternion norm).
  Final task translation error norm = 0.130314 (in meters).
  Final measurement error norm = 0.231114 (Dual quaternion norm).
  Final measurement translation error norm = 0.061223 (in meters).
[7] Running WITHOUT adaptation.
...
Reference timeout for xd0
  Average computational time = 0.000304637 seconds.
  Clock overruns = 0 (Too many, i.e. hundreds, indicate that the sampling time is too low for this CPU).
  Final task pose error norm = 0.775077 (Dual quaternion norm).
  Final task translation error norm = 0.725812 (in meters).
  Final measurement error norm = 0 (Dual quaternion norm).
  Final measurement translation error norm = 0.123291 (in meters).
Reference timeout for xd1
  Average computational time = 0.000312849 seconds.
  Clock overruns = 0 (Too many, i.e. hundreds, indicate that the sampling time is too low for this CPU).
  Final task pose error norm = 0.685748 (Dual quaternion norm).
  Final task translation error norm = 0.213954 (in meters).
  Final measurement error norm = 0 (Dual quaternion norm).
  Final measurement translation error norm = 0.122333 (in meters).
```

Notes:

- The final task errors are **not** zero by design: the last target (`xd1`) is
  intentionally unreachable (see *Known limitations*).
- `Clock overruns = 0` means the sampling time was adequate on that machine; a
  large (hundreds+) number means it was not.

## Tested on

- Ubuntu 22.04 `5.19.0-41-generic #42~22.04.1-Ubuntu SMP PREEMPT_DYNAMIC Tue Apr 18 17:40:00 UTC 2 x86_64 x86_64 x86_64 GNU/Linux`
- g++ --version `g++ (Ubuntu 11.3.0-1ubuntu1~22.04.1) 11.3.0`
- DQ Robotics cpp as shown in the submodule information.
- DQ Robotics cpp-interface-qpoases as shown in the submodule information.
- qpOASES as shown in the submodule information.
- sas_core as shown in the submodule information.

## Changelog

- 2026.09. README: added a self-contained Python + pyplot example (the adaptive control loop run end-to-end, headlessly, with a comparison figure); refreshed the C++ console-output sample to match the current two-run (with/without adaptation) example; and flagged the pre-compiled `v23.05.1` download as a 2023 snapshot that predates the headless and `M3_`-refactor changes.
- 2026.09. Dropped the legacy `M3_` prefix from the C++ identifiers and moved all C++ code into the `marinholab::papers::tro2022::adaptive_control` namespace (headers/sources renamed to `AdaptiveController`, `SerialManipulatorEDH`, `SimulatorDummy`, `VFI`, `MeasurementSpace`). The Python API was aligned with the C++ classes in the same step: the pybind11 module now exposes `SerialManipulatorEDH`, `SimulatorDummy`, `AdaptiveController`, `VFI`, `MeasureSpace`, `Primitive`, `VFI_Direction`, `VFI_DistanceType` (and the `_ParameterSpaceEDH` submodule) — the historical `M3_*` Python names are gone, and the `book/` tutorial + `adaptive_control_import_eval.py` were updated to the new names.
- 2026.08. Removed the dependency on the external robot simulator and its network interface: the example now runs headless on the in-memory stand-in simulator `SimulatorDummy` (dry testing), so no external simulator is required to build or run it.
- 2025.06. Removed Python wrapper instructions now that it's available via PyPI.
- 2025.05. Updating code to work with an external-simulator-based interface.
