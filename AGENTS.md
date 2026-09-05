# AGENTS.md — tro2022_adaptivecontrol

Notes for future sessions (repo layout + hard constraints).

## Layout (as of commit 671bfd5, PR #10)
Single-tree layout, consolidated from the old `python_wrapper/` design:

- One top-level `CMakeLists.txt` builds everything:
  `adaptive_control` (static lib), `adaptive_control_cpp` (headless exe),
  and `_core` (pybind11 module, gated behind `BUILD_PYTHON_WRAPPER`).
- Python package at repo root: `marinholab/papers/tro2022/adaptive_control/`
  (only that package dir has an `__init__.py`; upper levels are namespace
  packages). Binding source: `src/adaptive_control_example_py.cpp`.
- All C++ declarations live in the namespace
  `marinholab::papers::tro2022::adaptive_control` (see below). The C++
  identifiers dropped the legacy `M3_` prefix; headers/sources are now
  `AdaptiveController`, `SerialManipulatorEDH`, `SimulatorDummy`, `VFI`,
  `MeasurementSpace` (in `include/.../adaptive_control/` and `src/example/`).
- `setup.py` / `pyproject.toml` at the repo root. Wheel is built from the
  repo root: `pip install .` (or `pip wheel . -w dist/`).
  - The `_core` extension must keep its default pybind11 name
    (`_core.cpython-XY-ARCH.so`) and be built with `CMAKE_POSITION_INDEPENDENT_CODE ON`
    (the static libs link into the shared module).
- `adaptive_control_import_eval.py` at the repo root; it self-registers the
  installed `_core` `.so` when run from the repo root (the source tree
  shadows the installed package there).
- `.build.sh` = pure-C++ build (`-DBUILD_PYTHON_WRAPPER=OFF`), produces
  `./adaptive_control_cpp` in the repo root (no `bin/`).
- `test_python_wrapper.sh` = CI Python flow: install dqrobotics from PyPI,
  `pip install .`, run the import eval.

## C++ namespace & naming
- Every C++ declaration is in `marinholab::papers::tro2022::adaptive_control`
  (mirrors the Python package path). The legacy `M3_` prefix is gone from C++.
- C++ names: `AdaptiveController`, `SerialManipulatorEDH` (nested
  `ParameterSpaceEDH` with `Example_Parameter`/`Example_ParameterType`),
  `SimulatorDummy`, `VFI` (`Primitive`, `VFI_Direction`, `VFI_DistanceType`),
  `MeasureSpace`, plus free helpers `get_measure_space_dimension`,
  `get_variable_boundary_inequalities`, `closest_invariant_error`.
- **The pybind11 API uses the same names as the C++ classes** — a 1:1
  match: `SerialManipulatorEDH`, `SimulatorDummy`, `AdaptiveController`,
  `VFI`, `MeasureSpace`, `Primitive`, `VFI_Direction`, `VFI_DistanceType`
  (and the `_ParameterSpaceEDH` submodule with `Example_Parameter` /
  `Example_ParameterType`). The historical `M3_*` Python names are gone;
  keep the Python-visible strings in `src/adaptive_control_example_py.cpp`
  (2nd arg of `py::enum_<T>(m, "...")` / `py::class_<T>(m, "...")` and the
  `def_submodule("_ParameterSpaceEDH", ...)` name) in sync with the C++
  class/enum names. If you rename a C++ class, rename the binding string and
  every Python consumer (`adaptive_control_import_eval.py`, `book/`
  tutorial) in the same change.
- Global-scope consumers (`src/adaptive_control_example.cpp` and the binding)
  bring the namespace in with `using namespace marinholab::papers::tro2022::adaptive_control;`.

## Hard constraints
- Keep the `sas_core` submodule and use of `sas::Clock` (required for a
  future step). `sas_object.cpp` must also be compiled because `sas::Clock`
  depends on `sas::Object` (link error otherwise).
- `AdaptiveController` (C++) uses `DQ_QPOASESSolver`, so `qpOASES` and
  `cpp-interface-qpoases` are required (cannot be removed).
- Simulation is headless via `SimulatorDummy` (C++) (16-object reference
  scene built in code; box half-size 0.2 m in x/y; robot base at identity
  `DQ(1,0,0,0,0,0,0,0)`). No CoppeliaSim/V-REP/ZMQ.
- DQ pose convention is 8-component (quaternion + translation).
- `dqrobotics` + wrapper share `__pybind11_internals_v11`
  (pybind11 v3.0 branch pinned in `dqrobotics/python/.gitmodules`).

## Verified commands
- C++: `./build/adaptive_control_cpp` → steps [1]-[7], adaptation error
  ~0.13 m final.
- Python: `/usr/local/bin/python adaptive_control_import_eval.py` → exit 0.
- Wheel: `pip wheel . -w /tmp/whl --no-deps` (works with isolation).

## CI gotcha (2026-08, PR #10)
macOS CI failed with `sas_core.hpp:35: fatal error: 'eigen3/Eigen/Dense'
file not found`: `sas_core.hpp` includes Eigen as `<eigen3/Eigen/Dense>`,
so the include path must contain the **parent** dir of `eigen3`
(`/opt/homebrew/include` on homebrew), not just the `eigen3` subdir.
Eigen has no CMake config; the root CMakeLists therefore uses
directory-scope `include_directories()` (APPLE-gated) so the path reaches
every target (lib, exe, `_core`). `find_package(Eigen3)` is unreliable and
was removed. Linux is unaffected (`/usr/include/eigen3` via libeigen3-dev).
