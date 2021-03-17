# thecolavrepo
<p> This repository contains branches of different Collision Avoidance (COLAV) algorithms, mainly based on the Scenario-based Model Predictive Control Algorithm [[1]](#1). 

The **sbmpc_catkin_ws** contains the ROS-based colav implemented by Inger Hagen and Giorgio Kufoalor through the Autosea project (with added robustness against obstacle track loss etc. [[3]](#3)), where the original SB-MPC is implemented, in addition to a velocity obstacle algorithm. "Insert more information here.."

The **psbmpc_cxx** implements the Probabilistic Scenario-based MPC [[2]](#2) in C/C++, which is an extended and improved version of the original SB-MPC, with more focus on probabilistic risk assessment, and which allows any given number of avoidance maneuvers in the prediction horizon. Here, one version is implemented for the CPU and another experimental for the GPU. </p>

## Git Workflow

All cooperators are obligated to follow the methods outlined in <https://www.atlassian.com/git/tutorials/comparing-workflows/feature-branch-workflow> for ensuring a pain-free workflow with thecolavrepo. 

If you're unfamiliar with git, check out <https://try.github.io/> to get familiar, and use <https://learngitbranching.js.org/> for trying out topics on your own.


## References

<a id="1">[1]</a>  Johansen, T. A., Perez, T., and Cristofaro, A., "Ship collision avoidance and COLREGS compliance using simulation-based control behavior selection with predictive hazard assessment" IEEE
Transactions on Intelligent Transportation Systems, vol. 17, no. 12, pp. 3407-3422, Dec. 2016.

<a id="2">[2]</a>  Tengesdal, T., Johansen, T. A., and Brekke, E. (2020). "Risk-based Maritime Autonomous Collision Avoidance Considering Obstacle Intentions." 2020 23rd International Conference on Information Fusion (FUSION), South Africa.

<a id="3">[3]</a> Kufoalor, D. K. M., Wilthil, E., Hagen, I. B., Brekke E. F. and Johansen, T. A. (2019). "Autonomous COLREGSs-Compliant Decision Making using Maritime Radar Tracking and Model Predictive Control" 2019 18th European Control Conference (ECC).



<p> Trym Tengesdal, 17. March 2021.  </p>
