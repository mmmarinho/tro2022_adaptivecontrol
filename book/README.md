# Python example: Adaptive Constrained Kinematic Control

This folder contains an executable [MyST text notebook](https://mystmd.org/guide/notebooks-with-markdown)
that runs, in Python, the adaptive constrained kinematic control example of
Marinho & Adorno (IEEE T-RO 2022) implemented by the package in this
repository.

> M. M. Marinho and B. V. Adorno, "Adaptive Constrained Kinematic Control Using
> Partial or Complete Task-Space Measurements," *IEEE Transactions on Robotics*,
> vol. 38, no. 6, pp. 3498--3513, Dec. 2022, doi: 10.1109/TRO.2022.3181047.

## Notebooks

| Title | Content |
|-------|---------|
| [Adaptive Constrained Kinematic Control (Python example)](./adaptive_control_tutorial.ipynb) | Runs the example headlessly on the in-memory `SimulatorDummy`, with a *deliberately wrong* initial model that is **adapted** online from task-space measurements, and compares a run **with** adaptation (`FULL`) against one **without** (`TASK_ONLY`). |

## Running the notebook

1. Build and install the package from the **repository root** (the parent of this
   `book/` folder) — it is compiled, so it needs `g++`, `cmake`, `ninja` and the
   Eigen headers (`libeigen3-dev` on Debian/Ubuntu):

   ```bash
   cd ..  # repository root
   git submodule update --init --recursive
   python3 -m pip install .
   ```

2. Open `adaptive_control_tutorial.md` (or the generated `.ipynb`) in a Jupyter
   environment and run all cells.

The notebook is self-contained about its own setup: its first cells install the
lightweight dependencies (`numpy`, `matplotlib`) and, if the compiled package is
not importable, build it **best-effort** from this repository's checkout
(cloning the `no_vrep` branch only if no checkout is found). In a CI/website
build it skips the multi-minute source build and the example cells degrade
gracefully — the notebook still renders and executes without errors.

## Generating the `.ipynb` (downloadable notebook)

The `.md` is the source of truth. To regenerate the `.ipynb` (a build artifact,
git-ignored here):

```bash
pip install jupytext pyyaml
python convert_to_ipynb.py
```

This expands the custom LaTeX macros (defined in `myst.yml`) so the `.ipynb`
renders in any notebook viewer, then converts with jupytext.

## Building a website from this folder

The `myst.yml` here is a minimal, self-contained config. It can be used
standalone (e.g. `python -m jupyter-book build --execute`) or dropped into a
larger book such as
[OpenExecutableBooksRobotics](https://github.com/MarinhoLab/OpenExecutableBooksRobotics).
