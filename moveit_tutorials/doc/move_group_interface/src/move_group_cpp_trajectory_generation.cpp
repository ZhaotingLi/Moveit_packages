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

// parameters for the current status of the state machine
bool finished = false; // whether the task is finished
int current_state_index = 1; // 1: reach into the goal   0: return to the start   2: waiting

// parameters of the elastic band
// double pin1_x = 0;     // should set to zero during the whole loop test
// double pin1_z = 0;
double pin1_x = 0.5;      // in the case where only planning is tested
double pin1_z = 0.38;

// valude of the optimized joint goal, which is published by the contact_detection_node_simplifiedModel node
std::vector<double> optimized_joint_goal;

/**
 * @brief Callback function for the contact status
 */
void contact_status_callback(const contact_msgs::contact_loop_status& msg){
  finished = msg.is_finished;
  current_state_index = msg.current_state_index;
  pin1_x = msg.pin1_x;
  pin1_z = msg.pin1_z;
  optimized_joint_goal.clear();
  for(int i = 0; i < 7; i++){
    optimized_joint_goal.push_back(msg.optimized_q_goal[i]);
  }
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


  static const std::string PLANNING_GROUP = "panda_arm";
  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);


  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

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
  // planner_config.config["pin1_x"] = "0.5";
  // planner_config.config["pin1_z"] = "0.4";
  planner_config.config["pin1_x"] = "0.0";
  planner_config.config["pin1_z"] = "0.0";
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
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
  visual_tools.deleteAllMarkers();

  // Remote control is an introspection tool that allows users to step through a high level script via buttons and keyboard shortcuts in RViz
  visual_tools.loadRemoteControl();

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

  /* -------------------- add obstalces [Begin]-------------------------------------------------------------------*/
  double obs_pos_x_bias = 0.13;
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
  primitive.dimensions[primitive.BOX_Z] = 0.72;

  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.5 + obs_pos_x_bias;
  box_pose.position.y = 0.29;
  box_pose.position.z = 0.36;

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
  box_pose.position.y = -0.29;
  box_pose.position.z = 0.36;

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
  primitive.dimensions[primitive.BOX_Y] = 0.58;
  primitive.dimensions[primitive.BOX_X] = 0.3;
  primitive.dimensions[primitive.BOX_Z] = 0.03;

  collision_object.id = "box1";
  // Define a pose for the box (specified relative to frame_id)
  // geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.y = 0;
  box_pose.position.x = 0.5 + obs_pos_x_bias;
  box_pose.position.z = 0.72;

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
  primitive.dimensions[primitive.BOX_Z] = 0.72;

  collision_object.id = "box1";
  // Define a pose for the box (specified relative to frame_id)
  // geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.y = 0;
  box_pose.position.x = 0.65 + obs_pos_x_bias;
  box_pose.position.z = 0.36;

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

  /* -------------------- add obstalces [end]-------------------------------------------------------------------*/

  // moveit_msgs::PlanningScene planning_scene_msg; 
  // collision_detection::AllowedCollisionMatrix& acm = planning_scene_monitor->getPlanningScene()->getAllowedCollisionMatrixNonConst();
  // acm.setEntry("panda_link0", "box1", true);
  // acm.setEntry("panda_link1", "box1", true);
  // acm.setEntry("panda_link2", "box1", true);
  // acm.setEntry("panda_link3", "box1", true);
  // acm.setEntry("panda_link4", "box1", true);
  // acm.setEntry("panda_link5", "box1", true);
  // // acm.setEntry("panda_link6", "box1", true);

  // acm.getMessage(planning_scene_msg.allowed_collision_matrix);
  // planning_scene_msg.world.collision_objects = collision_objects;
  // planning_scene_msg.is_diff = true;
  // moveit_msgs::ApplyPlanningScene srv;
  // srv.request.scene = planning_scene_msg;
  // planning_scene_diff_client.call(srv);

  


  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  
  // (3) retrun to start pose
  moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState();
  // //
  // // Next get the current set of joint values for the group.
  std::vector<double> joint_start_state;
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
  current_state->copyJointGroupPositions(joint_model_group, joint_start_state);

  // joint goal postion if the optimized_goal_q is not available
  // standard joint state without optimization
  // joint_group_positions[0] = 0 * 3.1415926 / 180;
  // joint_group_positions[1] = 52 * 3.1415926/180;
  // joint_group_positions[2] =  0 * 3.1415926/180;
  // joint_group_positions[3] =  -90 * 3.1415926/180;
  // joint_group_positions[4] =  0 * 3.1415926 / 180;
  // joint_group_positions[5] =  137 * 3.1415926/180;
  // joint_group_positions[6] =  44 *3.1415926/180;


  // optimized joint state in mode 1
  // -0.08979270079669528, 0.8643770043246566, 0.08706532937078018, -1.7446617106558027, -0.14915937795513548, 2.764733060796357, 0.8636804622796009
  joint_group_positions[0] = -0.08979270079669528;
  joint_group_positions[1] = 0.8643770043246566;
  joint_group_positions[2] =  0.08706532937078018;
  joint_group_positions[3] =  -1.7446617106558027;
  joint_group_positions[4] =  -0.14915937795513548;
  joint_group_positions[5] =  2.764733060796357;
  joint_group_positions[6] =  0.8636804622796009;


  moveit::core::RobotStatePtr robot_state_start(
    new moveit::core::RobotState(planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor)->getCurrentState()));

  bool success = false;  // whether a single plan finds a feasible trajectory solution

  /*  Core function of this file: plan a path according to current status  */
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
    if(current_state_index == 1){  // if the current state is 1, then we need to plan to the goal
      if(pin1_z!= 0){   // if the pin1_z is not zero, then we need to put the elastic band into the planning process
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
      if(optimized_joint_goal.size() != 7){
        goal_state.setJointGroupPositions(joint_model_group, joint_group_positions);
      }else{
        goal_state.setJointGroupPositions(joint_model_group, optimized_joint_goal);
      }
      moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);

      req.goal_constraints.clear();
      req.group_name = "panda_arm";
      req.goal_constraints.push_back(joint_goal);
      current_state_index = 2;
      
    }else if(current_state_index == 0){  // if the current state is 0, then we need to plan to the start
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
    }else if(current_state_index == 2){
      success = false;
      continue;
    }

    req.allowed_planning_time = 10.0;
    // planning_pipeline->generatePlan(planning_scene, req, res); // [IMPORTANT]this line is wrong, the planning_scene is not updated
    planning_pipeline->generatePlan(planning_scene_monitor->getPlanningScene(), req, res);
    /* Check that the planning was successful */
    if (res.error_code_.val != res.error_code_.SUCCESS)
    {
      ROS_ERROR("Could not compute plan successfully");
      // return 0;
      success = false;
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
      move_group_interface.move();

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



  ros::shutdown();
  return 0;
}
