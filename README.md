# Adaptive Constrained Kinematic Control using Partial or Complete Task-Space Measurements

Sample code and minimal example for [our TRO2022 paper](https://doi.org/10.1109/TRO.2022.3181047): 

```bib
@Article{marinhoandadorno2022adaptive,
  author       = {Marinho, M. M. and Adorno, B. V.},
  title        = {Adaptive Constrained Kinematic Control using Partial or Complete Task-Space Measurements},
  journal      = {IEEE Transactions on Robotics (T-RO)},
  year         = {2022},
  month        = dec,
  doi          = {10.1109/TRO.2022.3181047},
  url          = {https://arxiv.org/pdf/2109.06375.pdf},
  url_video    = {https://youtu.be/WntmyJe53gY},
  custom_type  = {1. Journal Paper},
  volume       = {38},
  number       = {6},
  pages        = {3498--3513}
}
```
# Example showcase

- The red object represents the estimated robot, initially very wrong.
- In seconds, the estimation converges by using measurements from a simulated sensor.
- The robot then proceeds through the box, reaching the target poses, without collisions. 

https://github.com/mmmarinho/tro2022_adaptivecontrol/assets/46012516/2abe0b0b-6e48-46e9-9a86-061ba013b355

# Tested on

- Ubuntu 22.04 `5.19.0-41-generic #42~22.04.1-Ubuntu SMP PREEMPT_DYNAMIC Tue Apr 18 17:40:00 UTC 2 x86_64 x86_64 x86_64 GNU/Linux`
- CoppeliaSim EDU 5.4.1 (rev4)
- DQ Robotics cpp [`commit 77acf9a42875ffb69e9f48f98f3950f9d7242c0e`](https://github.com/dqrobotics/cpp/commit/77acf9a42875ffb69e9f48f98f3950f9d7242c0e)
- DQ Robotics cpp-interface-vrep [`commit 67a5839074243e262de3a6c83439dc3a59492913`](https://github.com/dqrobotics/cpp-interface-vrep/commit/67a5839074243e262de3a6c83439dc3a59492913)
- DQ Robotics cpp-interface-qpoases [`commit cdc2d6cbc6d67074267c38227b85bb1f14df8b14`](https://github.com/dqrobotics/cpp-interface-qpoases/commit/cdc2d6cbc6d67074267c38227b85bb1f14df8b14)
- qpOASES [`commit 0b86dbf00c7fce34420bedc5914f71b176fe79d3`](https://github.com/coin-or/qpOASES/commit/0b86dbf00c7fce34420bedc5914f71b176fe79d3)
- sas_core [`commit 696b2019c30d62e322030eec8a0c2bb2f3f7b3c8`](https://github.com/SmartArmStack/sas_core/commit/696b2019c30d62e322030eec8a0c2bb2f3f7b3c8)

# Known limitations, *TODO* list

- The stopping criterium is elapsed time, so it might not converge for all initial parameters.
- The initial convergence to measurements implemented in the experiments is *TODO* for this example.
- The estimated model is randomized so it might start in an implausible zone. Fixing this it *TODO* for this example.
- Sample code for partial measurements is included, but they have not been tested in this example, only in the physical robot.

# Extra info

- The adaptation is supposed to move the parameters of the `estimated_robot` towards ideal kinematic model, defined by `real_robot` in the code. 
The robot model in CoppeliaSim is for visualization only.
- A different solver was used in the paper's experiments, in this example we use an open-source solver, so the behavior might be somewhat different.
- The final target position is, **ON PURPOSE**, chosen as somewhere the robot cannot reach. It serves to show that even in such case the robot does not collide with the environment.

# Installation

## Just in case

```bash
sudo apt install g++ cmake git
```

## DQ Robotics Development 

```bash
sudo add-apt-repository ppa:dqrobotics-dev/development
sudo apt-get update
sudo apt-get install libdqrobotics*
```

## qpOASES

```bash
cd ~/Downloads
git clone https://github.com/coin-or/qpOASES.git
cd qpOASES
sed -i -e 's/option(BUILD_SHARED_LIBS "If ON, build shared library instead of static" OFF)/option(BUILD_SHARED_LIBS "If ON, build shared library instead of static" ON)/g' CMakeLists.txt
mkdir build
cd build
cmake ..
make -j16
sudo make install
sudo ldconfig
```

## Download the repo

```bash
cd ~
mkdir git
cd git
git clone https://github.com/mmmarinho/tro2022_adaptivecontrol.git --recursive
```

# Build

With all dependencies correctly configured,

```bash
cd ~/git/tro2022_adaptivecontrol
mkdir build
cd build
cmake ..
make -j16
```

# Running

1. Open the example scene, namely `TRO2022_MarinhoAdorno_ReferenceScene.ttt` on CoppeliaSim.
2. Run `~/git/tro2022_adaptivecontrol/build/adaptive_control_example`.

