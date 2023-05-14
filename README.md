# Moveit Package

This moveit package is built from the original moveit package, with three main modifications: 

1. Introduce a new cost considering the length of the elastic band, see `moveit/moveit_planners/ompl/ompl_interface/src/model_based_planning_context.cpp` 

2. Implememnt a planning framework in `moveit_tutorials/doc/move_group_interface/src/move_group_cpp_trajectory_generation.cpp`

3. The controller used in real robot is changed to contact-aware controller, see `panda_moveit_config/launch/franka_control.launch`

## Install 

1. Follow the instruction here: https://moveit.ros.org/install/source/  (Note that use the code of this package, intead of the official moveit source code.)

2. Blacklist: catkin config --blacklist     moveit_commander     moveit_setup_assistant     moveit_ros_robot_interaction     moveit_ros_visualization     moveit_ros_benchmarks     moveit_controller_manager_example  

3. One curl error can be solved by https://github.com/ros-planning/moveit/issues/697 However, remember to delete the build and devel files before rebuild. 

4. Run the rviz launch demo, there may be a bug to load the motion planning visualization: [ERROR] [1671728392.408823536]: PluginlibFactory: The plugin for class 'moveit_rviz_plugin/MotionPlanning' failed to load. Can be solved by install sudo apt-get install ros-noetic-moveit-ros-visualization  https://blog.csdn.net/u011304078/article/details/102723477  


## Usage in simulation

Start the robot
```
roslaunch panda_moveit_config demo_gazebo.launch

```

Then plan a path by running
```
roslaunch moveit_tutorials move_group_cpp_trajectory_generation.launch 
```


## Usage connecting with real robot

Connect to the real robot
```
roslaunch panda_moveit_config franka_control.launch robot_ip:=172.16.0.2 load_gripper:=true
```
Then plan a path by running
```
roslaunch moveit_tutorials move_group_cpp_trajectory_generation.launch 
```
