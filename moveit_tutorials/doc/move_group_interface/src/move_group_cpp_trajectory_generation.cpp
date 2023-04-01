/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta, Dave Coleman, Mike Lautman */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_pipeline/planning_pipeline.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include "geometry_msgs/Wrench.h"
#include <Eigen/Core>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>

#include <moveit/kinematic_constraints/utils.h>

#include <moveit_msgs/ApplyPlanningScene.h>

#include <moveit/robot_state/conversions.h>

#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include "contact_msgs/contact_loop_status.h"

// The circle constant tau = 2*pi. One tau is one rotation in radians.
const double tau = 2 * M_PI;

// to be modified, this should also be included in the low level control part
// to do: add a .h file and a class to store the obstacles info
Eigen::Vector3d ext_force;
Eigen::Vector3d contact_position;
bool is_contact_happening = false;
bool is_previous_contact_happening = false;
bool is_contact_saved = false;

// need to check whether the obstacle has already in this set before saving the new one
std::vector<Eigen::Vector3d> obstacle_set;  // contain the estimated center of detected obstacles 


void callback(const geometry_msgs::Wrench& msg){
  ext_force << msg.force.x, msg.force.y, msg.force.z;
  contact_position << msg.torque.x, msg.torque.y, msg.torque.z;
  is_previous_contact_happening = is_contact_happening;
  if(ext_force.norm() > 2){
    is_contact_happening = true;
  }else{
    is_contact_happening = false;
  }
  if(ext_force.norm() > 10){
    if(!is_contact_saved){
      obstacle_set.push_back(contact_position);
      is_contact_saved = true;
    }
    // also publish to the topic "effort_joint_trajectory_controller/command" to stop current plan excuation, can be done in another node
  }

  // contact just happen
  if(!is_previous_contact_happening && is_contact_happening){
    is_contact_saved = false;
  }
}

bool finished = false; // whether the task is finished
int current_state_index = 1; // 1: reach into the goal   0: return to the start   2: waiting
double pin1_x = 0;
double pin1_z = 0;

void contact_status_callback(const contact_msgs::contact_loop_status& msg){
  finished = msg.is_finished;
  current_state_index = msg.current_state_index;
  pin1_x = msg.pin1_x;
  pin1_z = msg.pin1_z;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;

  // ROS spinning must be running for the MoveGroupInterface to get information
  // about the robot's state. One way to do this is to start an AsyncSpinner
  // beforehand.

  ros::Subscriber sub_haptic = node_handle.subscribe("/contact_loop_status", 100, contact_status_callback);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  // BEGIN_TUTORIAL
  //
  // Setup
  // ^^^^^
  //
  // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
  // the `JointModelGroup`. Throughout MoveIt the terms "planning group" and "joint model group"
  // are used interchangeably.
  static const std::string PLANNING_GROUP = "panda_arm";

  // The :planning_interface:`MoveGroupInterface` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);

  // We will use the :planning_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const moveit::core::JointModelGroup* joint_model_group =
      move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);


  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor;
  planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
  if (!planning_scene_monitor->getPlanningScene())
  {
    ROS_ERROR_STREAM_NAMED("tutorial", "Error in setting up the PlanningSceneMonitor.");
    exit(EXIT_FAILURE);
  }

  planning_scene_monitor->startSceneMonitor();
  planning_scene_monitor->startWorldGeometryMonitor(
      planning_scene_monitor::PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC,
      planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC,
      false /* skip octomap monitor */);
  planning_scene_monitor->startStateMonitor();


  robot_model_loader::RobotModelLoaderPtr robot_model_loader(
      new robot_model_loader::RobotModelLoader("robot_description"));
  moveit::core::RobotModelPtr robot_model = robot_model_loader->getModel();

  planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));

  // create a planning pipeline
  // planning_pipeline::PlanningPipelinePtr planning_pipeline =
  //     std::make_shared<planning_pipeline::PlanningPipeline>(robot_model, node_handle, "ompl_interface/OMPLPlanner");
  planning_pipeline::PlanningPipelinePtr planning_pipeline(
      new planning_pipeline::PlanningPipeline(robot_model, node_handle, "planning_plugin", "request_adapters"));
  // get the current planner configurations
  planning_interface::PlannerConfigurationMap planner_configs =
      planning_pipeline->getPlannerManager()->getPlannerConfigurations();
    
  const std::string planner_id = "geometric::BiTRRT";
  planning_interface::PlannerConfigurationSettings planner_config;
  planner_config.name = "my_planner";
  planner_config.group = PLANNING_GROUP;
  planner_config.config["type"] = planner_id;
  planner_config.config["pin1_x"] = "0";
  planner_config.config["pin1_z"] = "0";
  planner_configs[PLANNING_GROUP] = planner_config;
  planning_pipeline->getPlannerManager()->setPlannerConfigurations(planner_configs);

  // print planner_configs[i].name, also print the config
  // print planner_configs[PLANNING_GROUP].config
  // print planner_configs[PLANNING_GROUP].config[planner_id]
  for(auto j = planner_configs[PLANNING_GROUP].config.begin(); j != planner_configs[PLANNING_GROUP].config.end(); j++){
    std::cout << j->first << std::endl;
    std::cout << j->second << std::endl;
  }
  
  // getPlanningAlgorithms
  std::vector<std::string> algorithms;
  planning_pipeline->getPlannerManager()->getPlanningAlgorithms(algorithms);
  for(auto i = algorithms.begin(); i != algorithms.end(); i++){
    std::cout << *i << std::endl;
  }

  // Visualization
  // ^^^^^^^^^^^^^
  //
  // The package MoveItVisualTools provides many capabilities for visualizing objects, robots,
  // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script.
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
  visual_tools.deleteAllMarkers();

  // Remote control is an introspection tool that allows users to step through a high level script
  // via buttons and keyboard shortcuts in RViz
  visual_tools.loadRemoteControl();

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  // visual_tools.trigger();

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());

  // We can get a list of all the groups in the robot:
  ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
  std::copy(move_group_interface.getJointModelGroupNames().begin(),
            move_group_interface.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

  // Start the demo
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

  double obs_pos_x_bias = 0.15;
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = move_group_interface.getPlanningFrame();
    // add second obstacle for cabinet
  // The id of the object is used to identify it.
  collision_object.id = "box1";

  // Define a box to add to the world.
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_Y] = 0.02;
  primitive.dimensions[primitive.BOX_X] = 0.3;
  primitive.dimensions[primitive.BOX_Z] = 0.8;

  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.5 + obs_pos_x_bias;
  box_pose.position.y = 0.3;
  box_pose.position.z = 0.4;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  // add third 
  // The id of the object is used to identify it.
  collision_object.id = "box1";
  // Define a pose for the box (specified relative to frame_id)
  // geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.5 + obs_pos_x_bias;
  box_pose.position.y = -0.3;
  box_pose.position.z = 0.4;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  // std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  // // add third "elastic band"
  // // The id of the object is used to identify it.
  // primitive.type = primitive.BOX;
  // primitive.dimensions.resize(3);
  // primitive.dimensions[primitive.BOX_Y] = 0.6;
  // primitive.dimensions[primitive.BOX_X] = 0.01;
  // primitive.dimensions[primitive.BOX_Z] = 0.03;   // 0.03

  // collision_object.id = "box1";
  // box_pose.orientation.w = 1.0;
  // box_pose.position.y = 0;
  // box_pose.position.x = 0.35 + obs_pos_x_bias;
  // box_pose.position.z = 0.45 - primitive.dimensions[primitive.BOX_Z]/2;
  // // box_pose.position.z = 0.4 - primitive.dimensions[primitive.BOX_Z]/2;

  // collision_object.primitives.push_back(primitive);
  // collision_object.primitive_poses.push_back(box_pose);
  // collision_object.operation = collision_object.ADD;
  // collision_objects.push_back(collision_object);

    // add 4th-- upper borad of the cabinet 
  // The id of the object is used to identify it.
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_Y] = 0.6;
  primitive.dimensions[primitive.BOX_X] = 0.3;
  primitive.dimensions[primitive.BOX_Z] = 0.03;

  collision_object.id = "box1";
  // Define a pose for the box (specified relative to frame_id)
  // geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.y = 0;
  box_pose.position.x = 0.5 + obs_pos_x_bias;
  box_pose.position.z = 0.77;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  // std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

    // add 5th-- ground
  // The id of the object is used to identify it.
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_Y] = 1;
  primitive.dimensions[primitive.BOX_X] = 1;
  primitive.dimensions[primitive.BOX_Z] = 0.01;

  collision_object.id = "box1";
  // Define a pose for the box (specified relative to frame_id)
  // geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.y = 0;
  box_pose.position.x = 0.5;
  box_pose.position.z = -0.05;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  // std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

     // add 6th-- back borad of the cabinet 
  // The id of the object is used to identify it.
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_Y] = 0.6;
  primitive.dimensions[primitive.BOX_X] = 0.02;
  primitive.dimensions[primitive.BOX_Z] = 0.8;

  collision_object.id = "box1";
  // Define a pose for the box (specified relative to frame_id)
  // geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.y = 0;
  box_pose.position.x = 0.65 + obs_pos_x_bias;
  box_pose.position.z = 0.4;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  // std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);


  // Now, let's add the collision object into the world
  // (using a vector that could contain additional objects)
  ROS_INFO_NAMED("tutorial", "Add an object into the world");
  planning_scene_interface.addCollisionObjects(collision_objects);
  // Show text in RViz of status and wait for MoveGroup to receive and process the collision object message
  visual_tools.publishText(text_pose, "Add object", rvt::WHITE, rvt::XLARGE);

  ros::ServiceClient planning_scene_diff_client =
      node_handle.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
  planning_scene_diff_client.waitForExistence();
  // and send the diffs to the planning scene via a service call

  moveit_msgs::PlanningScene planning_scene_msg; 
  collision_detection::AllowedCollisionMatrix& acm = planning_scene_monitor->getPlanningScene()->getAllowedCollisionMatrixNonConst();
  acm.setEntry("panda_link0", "box1", true);
  acm.setEntry("panda_link1", "box1", true);
  acm.setEntry("panda_link2", "box1", true);
  acm.setEntry("panda_link3", "box1", true);
  acm.setEntry("panda_link4", "box1", true);
  acm.setEntry("panda_link5", "box1", true);
  // acm.setEntry("panda_link6", "box1", true);

  acm.getMessage(planning_scene_msg.allowed_collision_matrix);
  planning_scene_msg.world.collision_objects = collision_objects;
  planning_scene_msg.is_diff = true;
  moveit_msgs::ApplyPlanningScene srv;
  srv.request.scene = planning_scene_msg;
  planning_scene_diff_client.call(srv);

  


  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  
  // (3) retrun to start pose
  moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState();
  // //
  // // Next get the current set of joint values for the group.
  std::vector<double> joint_start_state;
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
  current_state->copyJointGroupPositions(joint_model_group, joint_start_state);

  // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
  // joint_group_positions[0] = -48 * 3.1415926 / 180;
  // joint_group_positions[1] = -17 * 3.1415926/180;
  // joint_group_positions[2] =  34 * 3.1415926/180;
  // joint_group_positions[3] =  -126 * 3.1415926/180;
  // joint_group_positions[4] =  10 * 3.1415926 / 180;
  // joint_group_positions[5] =  110 * 3.1415926/180;
  // joint_group_positions[6] =  26 *3.1415926/180;
  // joint_group_positions[0] = 16 * 3.1415926 / 180;
  // joint_group_positions[1] = 25 * 3.1415926/180;
  // joint_group_positions[2] =  -25 * 3.1415926/180;
  // joint_group_positions[3] =  -106 * 3.1415926/180;
  // joint_group_positions[4] =  13 * 3.1415926 / 180;
  // joint_group_positions[5] =  204 * 3.1415926/180;
  // joint_group_positions[6] =  39 *3.1415926/180;
  
  // reach into the cabinet and collide with the top board
  // joint_group_positions[0] = -87 * 3.1415926 / 180;
  // joint_group_positions[1] = -2 * 3.1415926/180;
  // joint_group_positions[2] =  11 * 3.1415926/180;
  // joint_group_positions[3] =  -107 * 3.1415926/180;
  // joint_group_positions[4] =  0 * 3.1415926 / 180;
  // joint_group_positions[5] =  106 * 3.1415926/180;
  // joint_group_positions[6] =  59 *3.1415926/180;

  // Motion 1: reach  and collide into the top and left board of the cabient
  // joint_group_positions[0] = -78 * 3.1415926 / 180;
  // joint_group_positions[1] = 38 * 3.1415926/180;
  // joint_group_positions[2] =  10 * 3.1415926/180;
  // joint_group_positions[3] =  -92 * 3.1415926/180;
  // joint_group_positions[4] =  -8 * 3.1415926 / 180;
  // joint_group_positions[5] =  129 * 3.1415926/180;
  // joint_group_positions[6] =  70 *3.1415926/180;

  //Motion2 [Not good]: reach  and collide into the left board of the cabient, but also collide into the back board, 
  // joint_group_positions[0] = -74 * 3.1415926 / 180;
  // joint_group_positions[1] = 36 * 3.1415926/180;
  // joint_group_positions[2] =  6 * 3.1415926/180;
  // joint_group_positions[3] =  -93 * 3.1415926/180;
  // joint_group_positions[4] =  -10 * 3.1415926 / 180;
  // joint_group_positions[5] =  123 * 3.1415926/180;
  // joint_group_positions[6] =  -13 *3.1415926/180;

  // MOtion 3 [Good traj]: env: cabinet_crl, reach and collide into the left board of the cabient, tested for rigid contact qp solver 
  // joint_group_positions[0] = -74 * 3.1415926 / 180;
  // joint_group_positions[1] = 36 * 3.1415926/180;
  // joint_group_positions[2] =  6 * 3.1415926/180;
  // joint_group_positions[3] =  -93 * 3.1415926/180;
  // joint_group_positions[4] =  -10 * 3.1415926 / 180;
  // joint_group_positions[5] =  123 * 3.1415926/180;
  // joint_group_positions[6] =  -13 *3.1415926/180;

  // // Motion 4: env: cabinet_crl2
  // joint_group_positions[0] = -93 * 3.1415926 / 180;
  // joint_group_positions[1] = 50 * 3.1415926/180;
  // joint_group_positions[2] =  0 * 3.1415926/180;
  // joint_group_positions[3] =  -85 * 3.1415926/180;
  // joint_group_positions[4] =  0 * 3.1415926 / 180;
  // joint_group_positions[5] =  135 * 3.1415926/180;
  // joint_group_positions[6] =  41 *3.1415926/180;

  // Motion 5: env: cabinet_crl2, end effecotr a little bit lower along z axis than Motion 4
  // joint_group_positions[0] = -92 * 3.1415926 / 180;
  // joint_group_positions[1] = 54 * 3.1415926/180;
  // joint_group_positions[2] =  0 * 3.1415926/180;
  // joint_group_positions[3] =  -84 * 3.1415926/180;
  // joint_group_positions[4] =  0 * 3.1415926 / 180;
  // joint_group_positions[5] =  137 * 3.1415926/180;
  // joint_group_positions[6] =  42 *3.1415926/180;

  // // Motion 6: env: cabinet_crl2, end effecotr a little bit lower along z axis than Motion 5
  joint_group_positions[0] = 0 * 3.1415926 / 180;
  joint_group_positions[1] = 52 * 3.1415926/180;
  joint_group_positions[2] =  0 * 3.1415926/180;
  joint_group_positions[3] =  -90 * 3.1415926/180;
  joint_group_positions[4] =  0 * 3.1415926 / 180;
  joint_group_positions[5] =  137 * 3.1415926/180;
  joint_group_positions[6] =  44 *3.1415926/180;

  // Motion 6.2: env: cabinet_crl2, self motion of Motion 6
  // joint_group_positions[0] = 0 * 3.1415926 / 180;
  // joint_group_positions[1] = 46 * 3.1415926/180;
  // joint_group_positions[2] =  0 * 3.1415926/180;
  // joint_group_positions[3] =  -109 * 3.1415926/180;
  // joint_group_positions[4] =  0 * 3.1415926 / 180;
  // joint_group_positions[5] =  167 * 3.1415926/180;
  // joint_group_positions[6] =  44 *3.1415926/180;


  // moition 7: avoid the elastic band when reaching into the cabinet
  // joint_group_positions[0] = 0 * 3.1415926 / 180;
  // joint_group_positions[1] = -15 * 3.1415926/180;
  // joint_group_positions[2] =  0 * 3.1415926/180;
  // joint_group_positions[3] =  -130 * 3.1415926/180;
  // joint_group_positions[4] =  0 * 3.1415926 / 180;
  // joint_group_positions[5] =  168 * 3.1415926/180;
  // joint_group_positions[6] =  45 *3.1415926/180;

  moveit::core::RobotStatePtr robot_state_start(
    new moveit::core::RobotState(planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor)->getCurrentState()));

  bool success = false;  // whether a single plan finds a feasible trajectory solution

  // need to update via other ros nodes: (1) the joint goal positions (2) planner interal parameter (pins state) (3) current state (4) the position of the elastic band (ready to be removed if not accurate)
  while(!finished){
    /*[Begin] Method 01 use move group interface, planner is executed in the move_group node launched by demo/real robot*/
    // move_group_interface.setJointValueTarget(joint_group_positions);

    // // We lower the allowed maximum velocity and acceleration to 5% of their maximum.
    // // The default values are 10% (0.1).
    // // Set your preferred defaults in the joint_limits.yaml file of your robot's moveit_config
    // // or set explicit factors in your code if you need your robot to move faster.
    // move_group_interface.setMaxVelocityScalingFactor(0.06);
    // move_group_interface.setMaxAccelerationScalingFactor(0.1);
    // move_group_interface.setPlanningTime(10.0);

    // success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    // ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");
    /*[End] use move group interface*/

    /*[Begin] Method02 use planning pipeline, planner is executed in this node, so we can modify the planner config in real time*/
    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res; 
    if(current_state_index == 1){
      if(pin1_z!= 0){
        std::cout<<"put the elastic band into planning process"<<std::endl;
        // update the pins state for ompl planner
        planner_configs =
        planning_pipeline->getPlannerManager()->getPlannerConfigurations();

        planner_config.name = "my_planner";
        planner_config.group = PLANNING_GROUP;
        planner_config.config["type"] = planner_id;
        planner_config.config["pin1_x"] = std::to_string(pin1_x);
        planner_config.config["pin1_z"] = std::to_string(pin1_z);
        planner_configs[PLANNING_GROUP] = planner_config;
        planning_pipeline->getPlannerManager()->setPlannerConfigurations(planner_configs);

        // add elastic band obstacle in moveit
        moveit_msgs::CollisionObject collision_object_eb;
        collision_object_eb.header.frame_id = move_group_interface.getPlanningFrame();

        // Define a box to add to the world.
        shape_msgs::SolidPrimitive primitive_band;
        primitive_band.type = primitive_band.BOX;
        primitive_band.dimensions.resize(3);
        primitive_band.dimensions[primitive_band.BOX_Y] = 0.6;
        primitive_band.dimensions[primitive_band.BOX_X] = 0.01;
        primitive_band.dimensions[primitive_band.BOX_Z] = 0.05;   // 0.03

        collision_object_eb.id = "elastic_band";
        geometry_msgs::Pose eb_pose;
        eb_pose.orientation.w = 1.0;
        eb_pose.position.y = 0;
        eb_pose.position.x = 0.37 + obs_pos_x_bias;
        eb_pose.position.z = pin1_z;
        // box_pose.position.z = 0.4 - primitive.dimensions[primitive.BOX_Z]/2;

        collision_object_eb.primitives.push_back(primitive_band);
        collision_object_eb.primitive_poses.push_back(eb_pose);
        collision_object_eb.operation = collision_object_eb.ADD;

        std::vector<moveit_msgs::CollisionObject> collision_objects_eb;
        collision_objects_eb.push_back(collision_object_eb);
        // planning_scene_interface.addCollisionObjects(collision_objects_eb);
        auto collision_objects_all = collision_objects;
        collision_objects_all.push_back(collision_object_eb);
        planning_scene_interface.addCollisionObjects(collision_objects_all);

        // set to ignore the collision between some links and the band
        planning_scene_diff_client.waitForExistence();
        moveit_msgs::PlanningScene planning_scene_msg2; 
        collision_detection::AllowedCollisionMatrix& acm2 = planning_scene_monitor->getPlanningScene()->getAllowedCollisionMatrixNonConst();
        acm2.setEntry("panda_link0", "elastic_band", true);
        acm2.setEntry("panda_link1", "elastic_band", true);
        acm2.setEntry("panda_link2", "elastic_band", true);
        acm2.setEntry("panda_link3", "elastic_band", true);
        acm2.setEntry("panda_link4", "elastic_band", true);
        acm2.setEntry("panda_link5", "elastic_band", true);
        // acm.setEntry("panda_link6", "box1", true);

        acm2.getMessage(planning_scene_msg2.allowed_collision_matrix);
        planning_scene_msg2.world.collision_objects = collision_objects_all;
        planning_scene_msg2.is_diff = true;
        moveit_msgs::ApplyPlanningScene srv2;
        srv2.request.scene = planning_scene_msg2;
        planning_scene_diff_client.call(srv2);
      }


      // robot_state = planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor)->getCurrentStateUpdated(response.trajectory_start);
      moveit::core::RobotStatePtr robot_state(
          new moveit::core::RobotState(planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor)->getCurrentState()));
      moveit::core::robotStateToRobotStateMsg(*robot_state, req.start_state);
      moveit::core::RobotState goal_state(*robot_state);
      goal_state.setJointGroupPositions(joint_model_group, joint_group_positions);
      moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);

      req.goal_constraints.clear();
      req.group_name = "panda_arm";
      req.goal_constraints.push_back(joint_goal);
      current_state_index = 2;
      
    }else if(current_state_index == 0){
      moveit::core::RobotStatePtr robot_state(
          new moveit::core::RobotState(planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor)->getCurrentState()));
      moveit::core::robotStateToRobotStateMsg(*robot_state, req.start_state);
      moveit::core::RobotState goal_state(*robot_state);
      goal_state.setJointGroupPositions(joint_model_group, joint_start_state);
      moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);

      req.goal_constraints.clear();
      req.group_name = "panda_arm";
      req.goal_constraints.push_back(joint_goal);
      current_state_index = 2;
    }

    planning_pipeline->generatePlan(planning_scene, req, res);
    /* Check that the planning was successful */
    if (res.error_code_.val != res.error_code_.SUCCESS)
    {
      ROS_ERROR("Could not compute plan successfully");
      return 0;
    }else{
      success = true;
    }
  
    // Visualize the plan in RViz
    moveit_msgs::MotionPlanResponse response;
    res.getMessage(response);

    // time parameterization
    moveit_msgs::RobotTrajectory trajectory;
    robot_trajectory::RobotTrajectory rt(move_group_interface.getRobotModel(), move_group_interface.getName());
    rt.setRobotTrajectoryMsg(*move_group_interface.getCurrentState(), response.trajectory);
    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    success =
        iptp.computeTimeStamps(rt, 0.2, 0.1); // velocity scale + acceleration scale
    rt.getRobotTrajectoryMsg(trajectory);

    ros::Publisher display_publisher =
        node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;
    display_trajectory.trajectory_start = response.trajectory_start;
    display_trajectory.trajectory.push_back(response.trajectory);
    display_publisher.publish(display_trajectory);


    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
    
    // visual_tools.trigger();
    // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to excuate the trajectoroy");

    ROS_INFO_NAMED("tutorial", "excuate the trajectory to the start_pose");
    // ros::WallDuration(3.0).sleep();

    // (4) excuate the planned trajectory
    if(success){
      moveit::planning_interface::MoveGroupInterface::Plan my_plan;
      my_plan.trajectory_ = trajectory;
      my_plan.start_state_ = response.trajectory_start;
      my_plan.planning_time_ = 10;
      move_group_interface.execute(my_plan);

      // remove current belief of the band, we will add it before planning
      if(current_state_index == 0 && pin1_z!=0){
        std::vector<std::string> to_removed_list;
        std::string to_removed_object = "elastic_band";
        to_removed_list.push_back(to_removed_object);
        planning_scene_interface.removeCollisionObjects(to_removed_list);
      }
    }
    // // move_group_interface.move();

  }

  ROS_INFO_NAMED("tutorial", "finish planning");
  // ros::WallDuration(3.0).sleep();


  // (5) add obstacles
  // Now let's define a collision object ROS message for the robot to avoid.
  // moveit_msgs::CollisionObject collision_object;
  // std::cout<<"size of saved obstacles: " << obstacle_set.size() << std::endl;
  // // The id of the object is used to identify it.
  // collision_object.id = "box1";

  // // Define a box to add to the world.
  // // shape_msgs::SolidPrimitive primitive;
  // primitive.type = primitive.BOX;
  // primitive.dimensions.resize(3);
  // primitive.dimensions[primitive.BOX_X] = 0.02;
  // primitive.dimensions[primitive.BOX_Y] = 0.1;
  // primitive.dimensions[primitive.BOX_Z] = 0.02;

  // if(obstacle_set.size() > 0){
  //   box_pose.orientation.w = 1.0;
  //   box_pose.position.x = obstacle_set[0][0] + primitive.dimensions[primitive.BOX_X] + 0.01;
  //   box_pose.position.y = obstacle_set[0][1];
  //   box_pose.position.z = obstacle_set[0][2] +  primitive.dimensions[primitive.BOX_Z]/2;
  // }else{
  //   box_pose.orientation.w = 1.0;
  //   box_pose.position.x = 0.4;
  //   box_pose.position.y = 0.0;
  //   box_pose.position.z = 0.75;
  // }


  // collision_object.primitives.push_back(primitive);
  // collision_object.primitive_poses.push_back(box_pose);
  // collision_object.operation = collision_object.ADD;

  // std::vector<moveit_msgs::CollisionObject> collision_objects_estiamted;
  // collision_objects_estiamted.push_back(collision_object);
  // ROS_INFO_NAMED("tutorial", "Add an object into the world");
  // planning_scene_interface.addCollisionObjects(collision_objects_estiamted);

  // // remove obstacles example
  // // Now, let's remove the objects from the world.
  // // ROS_INFO_NAMED("tutorial", "Remove the objects from the world");
  // // std::vector<std::string> object_ids;
  // // object_ids.push_back(collision_object.id);
  // // object_ids.push_back(object_to_attach.id);
  // // planning_scene_interface.removeCollisionObjects(object_ids);

  // // Show text in RViz of status and wait for MoveGroup to receive and process the collision object message
  // visual_tools.publishText(text_pose, "Add object", rvt::WHITE, rvt::XLARGE);
  // // visual_tools.trigger();
  // // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object appears in RViz");
  // ROS_INFO_NAMED("tutorial", "Plan the path");
  // ros::WallDuration(1.0).sleep();

  // (6) Plan a trajectory to avoid the obstacles
  // .. _move_group_interface-planning-to-pose-goal:
  //
  // Planning to a Pose goal
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // We can plan a motion for this group to a desired pose for the
  // end-effector.
  // Important! read the current state and send it to moveit. otherwise there will be the error Invalid Trajectory: start point deviates from current robot state more than 0.01 joint 'panda_joint2': expected: -0.569261, current: -0.554642
  // robot_state::RobotState current_state2(*move_group_interface.getCurrentState());
  // move_group_interface.setStartState(current_state2);

  // geometry_msgs::Pose target_pose1;
  // target_pose1.orientation.x=-0.9238795;
  // target_pose1.orientation.y = 0.3826834;
  // target_pose1.position.x = 0.5;
  // target_pose1.position.y = 0.0;
  // target_pose1.position.z = 0.49;
  // move_group_interface.setPoseTarget(target_pose1);

  // // Now, we call the planner to compute the plan and visualize it.
  // // Note that we are just planning, not asking move_group_interface
  // // to actually move the robot.
  // // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  // move_group_interface.setMaxVelocityScalingFactor(0.075);
  // move_group_interface.setMaxAccelerationScalingFactor(0.075);
  // success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  // ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  // // Visualizing plans
  // // ^^^^^^^^^^^^^^^^^
  // // We can also visualize the plan as a line with markers in RViz.
  // ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  // visual_tools.publishAxisLabeled(target_pose1, "pose1");
  // visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  // visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  // // visual_tools.trigger();
  // // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to execute the trajectory");
  // ROS_INFO_NAMED("tutorial", "excuate the  final path");
  // ros::WallDuration(6.0).sleep();


  //(7) excuate the plan
  // if(success){
  //   move_group_interface.execute(my_plan);
  // }
  // move_group_interface.move();

  // Moving to a pose goal
  // ^^^^^^^^^^^^^^^^^^^^^
  //
  // If you do not want to inspect the planned trajectory,
  // the following is a more robust combination of the two-step plan+execute pattern shown above
  // and should be preferred. Note that the pose goal we had set earlier is still active,
  // so the robot will try to move to that goal.

  // move_group_interface.move();

  
  // END_TUTORIAL

  ros::shutdown();
  return 0;
}
