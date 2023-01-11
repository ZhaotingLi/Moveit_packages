# Franka Emika Panda MoveIt Config Package

The Panda robot is the flagship MoveIt integration robot used in the MoveIt tutorials.
Any changes to MoveIt need to be propagated into this config fast, so this package
is co-located under the ``ros-planning`` Github organization here.


## Usage connecting with real robot

```
roslaunch panda_moveit_config franka_control.launch robot_ip:=172.16.0.2 load_gripper:=true
```