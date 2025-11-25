# psbmpc

This repository contains branches of different Collision Avoidance (COLAV) algorithms, based on the Probabilistic Scenario-based Model Predictive Control Algorithm (PSB-MPC), having its origins from [[1]](#1).

The **psbmpc_cxx** implements the Probabilistic Scenario-based MPC [[2]](#2) in C/C++, which is an extended and improved version of the original SB-MPC, with more focus on probabilistic risk assessment, and which allows any given number of avoidance maneuvers in the prediction horizon. Here, one version is implemented for the CPU (used for prototyping and testing mainly) and another for the GPU (which is meant to be used in real-time). Here, **psbmpc_ros_package** is a ROS1 package for using the PSB-MPC in an autonomous ship.

The **sbmpc_catkin_ws** contains the ROS-based colav implemented through the Autosea project (with added robustness against obstacle track loss etc. [[3]](#3)), where the original SB-MPC is implemented, in addition to a velocity obstacle algorithm.

## Citation
If you are using the `PSB-MPC` in your work, please use the following citation:
```
@article{Tengesdal2024fr,
  author  = {Tengesdal, Trym and Rothmund, Sverre V. and Basso, Erlend A. and Schmidt-Didlaukies, Henrik and Johansen, Tor A.},
  journal = {Field Robotics},
  title   = {Obstacle Intention Awareness in Automatic Collision Avoidance: Full Scale Experiments in Confined Waters},
  volume={4},
  number={1},
  pages={211--245},
  year={2024},
  doi     = {10.55417/fr.2024007},
}
```

## Git Workflow

All cooperators are obligated to follow the methods outlined in <https://www.atlassian.com/git/tutorials/comparing-workflows/feature-branch-workflow> for ensuring a pain-free workflow with thecolavrepo.

If you're unfamiliar with git, check out <https://try.github.io/> to get familiar, and use <https://learngitbranching.js.org/> for trying out topics on your own.

## References

<a id="1">[1]</a>  Johansen, T. A., Perez, T., and Cristofaro, A., "Ship collision avoidance and COLREGS compliance using simulation-based control behavior selection with predictive hazard assessment" IEEE
Transactions on Intelligent Transportation Systems, vol. 17, no. 12, pp. 3407-3422, Dec. 2016.

<a id="2">[2]</a>  Tengesdal, Trym and Rothmund, Sverre V. and Basso, Erlend A. and Johansen, Tor A. and Schmidt-Didlaukies, Henrik (2024). "Obstacle Intention Awareness in Automatic Collision Avoidance: Full Scale Experiments in Confined Waters." Field Robotics, vol. 4, no. 1, pp. 211-245, DOI: 10.55417/fr.2024007.

<a id="3">[3]</a> Kufoalor, D. K. M., Wilthil, E., Hagen, I. B., Brekke E. F. and Johansen, T. A. (2019). "Autonomous COLREGSs-Compliant Decision Making using Maritime Radar Tracking and Model Predictive Control" 2019 18th European Control Conference (ECC).

Trym Tengesdal, 17th of June 2024.
