# Adaptive Constrained Kinematic Control using Partial or Complete Task-Space Measurements

Sample code and minimal example for [our TRO2022 paper](https://doi.org/10.1109/TRO.2022.3181047).

# Video on YouTube

<a href="http://www.youtube.com/watch?feature=player_embedded&v=WntmyJe53gY" target="_blank">
 <img src="http://img.youtube.com/vi/WntmyJe53gY/hqdefault.jpg" alt="Watch the video" width="480" height="360" border="10" />
</a>

# Citation

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
# Standalone Example

- The red object represents the estimated robot, initially very wrong.
- In seconds, the estimation converges by using measurements from a simulated sensor.
- The robot then proceeds through the box, reaching the target poses, without collisions. 
- Using a different solver, we obtain much faster calculation when compared with the experiments in the original paper.

https://github.com/mmmarinho/tro2022_adaptivecontrol/assets/46012516/2abe0b0b-6e48-46e9-9a86-061ba013b355

## Known limitations, *TODO* list

- The stopping criterium is elapsed time, so it might not converge for all initial parameters.
- The initial convergence to measurements implemented in the experiments is *TODO* for this example.
- The estimated model is randomized so it might start in an implausible zone. Fixing this it *TODO* for this example.
- Sample code for partial measurements is included, but they have not been tested in this example, only in the physical robot.

## Extra info

- The adaptation is supposed to move the parameters of the `estimated_robot` towards ideal kinematic model, defined by `real_robot` in the code. 
The robot model in CoppeliaSim is for visualization only.
- A different solver was used in the paper's experiments, in this example we use an open-source solver, so the behavior might be somewhat different.
- The final target position is, **ON PURPOSE**, chosen as somewhere the robot cannot reach. It serves to show that even in such case the robot does not collide with the environment.

# Run the binary



# Build from source

## Just in case

```bash
sudo apt install g++ cmake git libeigen3-dev
```

## Download the repo

```bash
cd ~
mkdir git
cd git
git clone https://github.com/mmmarinho/tro2022_adaptivecontrol.git --recursive
```

## Build

With all dependencies correctly configured,

```bash
cd ~/git/tro2022_adaptivecontrol.git
chmod +x .build.sh
./.build.sh
```

# Running

1. Open the example scene, namely `TRO2022_MarinhoAdorno_ReferenceScene.ttt` on CoppeliaSim.
2. Run
```console
cd ~/git/tro2022_adaptivecontrol.git
chmod +x .run.sh
./.run.sh
```

# Example console output of the results

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

# Tested on

- Ubuntu 22.04 `5.19.0-41-generic #42~22.04.1-Ubuntu SMP PREEMPT_DYNAMIC Tue Apr 18 17:40:00 UTC 2 x86_64 x86_64 x86_64 GNU/Linux`
- CoppeliaSim EDU 5.4.1 (rev4)
- DQ Robotics cpp as shown in the submodule information.
- DQ Robotics cpp-interface-vrep as shown in the submodule information.
- DQ Robotics cpp-interface-qpoases as shown in the submodule information.
- qpOASES as shown in the submodule information.
- sas_core as shown in the submodule information.

