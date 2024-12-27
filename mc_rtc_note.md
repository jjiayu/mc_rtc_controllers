# Introduction

This documents list: 
- What we can do with mc-rtc at BIGAI (tested) 

- Issues we find

- Resources Collection

# Achieved Simulations and Demos (Converte to table with Demo name, robot name, ROS version, and Simulator names)

- CoM, End-Effector, and Posture Controller on JRVC1 robot (ROS1, ROS2)

- Door Opening Example (ROS1)

# Issues

### Communication Interface and ROS1 v.s. ROS2

Mc-rtc supports ROS2 (currently needs to build from source - using superbuild). The benefit (potentially) of using ROS2 is to have low-latency communication with the robot, e.g., using mc-rtc ROS2 interface to directly send joint command to the robot and retrieve sensor readings. 

However, many mc-rtc application packages (walking, multi-contact, loco-manipulation) built on ROS1 and requiring many dependencies that ROS2 currently does not support. To enable those features in ROS2, we need to work on code migration. 

Alternatively, we can think about communication in ROS1:
- Communications through ROS1 interface: The key issue here is how much the latency will affect the control performance, especially for the case involves force interactions. 

- Communication to the motors directly: This requires development of a package like other interfaces, e.g., mc_pepper, mc_mujoco, mc_franka. It sounds complicated, but in fact it is just sending motor command after control.run(), and retrieving sensor reading before control.run().

# Resources Collection
- Tutorials: https://jrl.cnrs.fr/mc_rtc/tutorials.html (ROS2 installation needs to follow the superbuild option)

- Interfaces to robots and simulators: https://jrl.cnrs.fr/mc_rtc/interfaces.html

- Mujoco interface: https://github.com/rohanpsingh/mc_mujoco (Replace Complicated Choroenoid/OpenRTM interface)

- List of Existing Supported Robot Modules: https://jrl.cnrs.fr/mc_rtc/robots.html

- BaselineWalkingController: https://github.com/isri-aist/BaselineWalkingController

- LocomanipController: https://github.com/isri-aist/LocomanipController

- Multi-Contact Contollers: https://github.com/isri-aist/MultiContactController

