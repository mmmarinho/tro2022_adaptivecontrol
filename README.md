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

Running on an 8 core Ubuntu VM.

*Not considering the setup step prints*

```console
Reference timeout for xd0
  Average computational time = 0.00126314 seconds.
  Clock overruns =7 (Too many, i.e. hundreds, indicate that the sampling time is too low for this CPU).
  Final task pose error norm 2.37699e-15 (Dual quaternion norm).
  Final task translation error norm 0 (in meters).
  Final measurement error norm 9.3756e-16 (Dual quaternion norm).
  Final measurement translation error norm 0 (in meters).
Reference timeout for xd1
  Average computational time = 0.000902905 seconds.
  Clock overruns =7 (Too many, i.e. hundreds, indicate that the sampling time is too low for this CPU).
  Final task pose error norm 0.0225817 (Dual quaternion norm).
  Final task translation error norm 0.044178 (in meters).
  Final measurement error norm 0.000940036 (Dual quaternion norm).
  Final measurement translation error norm 0.001836 (in meters).
```

## Tested on

- Ubuntu 22.04 `5.19.0-41-generic #42~22.04.1-Ubuntu SMP PREEMPT_DYNAMIC Tue Apr 18 17:40:00 UTC 2 x86_64 x86_64 x86_64 GNU/Linux`
- g++ --version `g++ (Ubuntu 11.3.0-1ubuntu1~22.04.1) 11.3.0`
- DQ Robotics cpp as shown in the submodule information.
- DQ Robotics cpp-interface-qpoases as shown in the submodule information.
- qpOASES as shown in the submodule information.
- sas_core as shown in the submodule information.

## Changelog

- 2026.09. Dropped the legacy `M3_` prefix from the C++ identifiers and moved all C++ code into the `marinholab::papers::tro2022::adaptive_control` namespace (headers/sources renamed to `AdaptiveController`, `SerialManipulatorEDH`, `SimulatorDummy`, `VFI`, `MeasurementSpace`). The Python API was aligned with the C++ classes in the same step: the pybind11 module now exposes `SerialManipulatorEDH`, `SimulatorDummy`, `AdaptiveController`, `VFI`, `MeasureSpace`, `Primitive`, `VFI_Direction`, `VFI_DistanceType` (and the `_ParameterSpaceEDH` submodule) — the historical `M3_*` Python names are gone, and the `book/` tutorial + `adaptive_control_import_eval.py` were updated to the new names.
- 2026.08. Removed the dependency on the external robot simulator and its network interface: the example now runs headless on the in-memory stand-in simulator `M3_SimulatorDummy` (dry testing), so no external simulator is required to build or run it.
- 2025.05. Updating code to work with an external-simulator-based interface.
- 2025.06. Removed Python wrapper instructions now that it's available via PyPI.
