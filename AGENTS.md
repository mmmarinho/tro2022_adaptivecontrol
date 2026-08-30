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

## Hard constraints
- Keep the `sas_core` submodule and use of `sas::Clock` (required for a
  future step). `sas_object.cpp` must also be compiled because `sas::Clock`
  depends on `sas::Object` (link error otherwise).
- `M3_AdaptiveController` uses `DQ_QPOASESSolver`, so `qpOASES` and
  `cpp-interface-qpoases` are required (cannot be removed).
- Simulation is headless via `M3_SimulatorDummy` (16-object reference
  scene built in code; box half-size 0.2 m in x/y; robot base at identity
  `DQ(1,0,0,0,0,0,0,0)`). No CoppeliaSim/V-REP/ZMQ.
- DQ pose convention is 8-component (quaternion + translation).
- `dqrobotics` + wrapper share `__pybind11_internals_v11`
  (pybind11 v3.0 branch pinned in `dqrobotics/python/.gitmodules`).

## Code style
- Prefer the `dqrobotics` functions over numpy wherever they exist (in Python:
  `dot`, `cross`, `norm`, `translation`, `rotation`, `Ad`, `DQ.normalize()`,
  and the `DQ()` constructor — `DQ([x,y,z])` builds a pure quaternion from a
  3-vector). Keep vectors as `DQ` end-to-end instead of bouncing through
  numpy. Note: `DQ.normalize()` returns a **new** normalized DQ (not in-place);
  the dual unit satisfies `E_*E_ == 0` (nilpotent, not `-1`).

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

## matplotlib / animation gotchas (3.x)
- To detect a 3d axes use `isinstance(ax, Axes3D)` — the old
  `hasattr(ax, 'proj3d')` check is **always False** on mpl 3.x (the `proj3d`
  property was removed); the buggy check made every `draw_scene` call create
  a NEW 3d axes and leak one per call.
- `plt.axes(projection='3d')` always creates a NEW axes even if a 3d one is
  current — never a reuse. Capture the returned object or reuse the existing
  one explicitly.
- `M3_PyPlotSimulator.draw_scene` is idempotent (static scene drawn once per
  axes, only the robot is redrawn afterwards) and
  `M3_PyPlotSimulator.animation(q, ...)` builds a `FuncAnimation` on top of
  that (static drawn once, robot-only redraw per frame). GIF always works via
  `PillowWriter`; mp4 etc. need ffmpeg on PATH.
