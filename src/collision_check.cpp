#include "collision_check.h"


Collision_Checker::Collision_Checker(){}
//Collision_Checker

double Collision_Checker::getDist(array<double, 6> config1, array<double, 6> config2)
{
  double sq_sum=0;

  for (int i=0; i < config1.size(); ++i){
    sq_sum += pow(config1[i]-config2[i],2);
  }
  
  return sqrt(sq_sum);
}

double Collision_Checker::getDist(vector<double> config1, vector<double> config2)
{
  double sq_sum=0;

  for (int i=0; i < config1.size(); ++i){
    sq_sum += pow(config1[i]-config2[i],2);
  }
  
  return sqrt(sq_sum);
}

bool Collision_Checker::nodeCollisionCheck(array<double, 6> config, planning_scene::PlanningScene *planning_scene, robot_state::RobotState& current_state, const robot_model::JointModelGroup* joint_model_group)
{
  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
  
  std::vector<double> joint_values;
  for (double elem : config) joint_values.push_back(elem);

  current_state.setJointGroupPositions(joint_model_group, joint_values);  
  planning_scene->checkCollision(collision_request,collision_result);

 // ROS_INFO_STREAM("Current state is " << (collision_result.collision ? "in" : "not in") << " collision");
  return collision_result.collision;
}

bool Collision_Checker::nodeCollisionCheck(vector<double> config, planning_scene::PlanningScene *planning_scene, robot_state::RobotState& current_state, const robot_model::JointModelGroup* joint_model_group)
{
  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
  
  std::vector<double> joint_values;
  for (double elem : config) joint_values.push_back(elem);

  current_state.setJointGroupPositions(joint_model_group, joint_values);  
  planning_scene->checkCollision(collision_request,collision_result);

  //ROS_INFO_STREAM("Current state is " << (collision_result.collision ? "in" : "not in") << " collision");
  return collision_result.collision;
}

bool Collision_Checker::edgeCollisionCheck(array<double, 6> config1, array<double, 6> config2,double edge_resol, planning_scene::PlanningScene *planning_scene, robot_state::RobotState& current_state, const robot_model::JointModelGroup* joint_model_group)
{

  array<double, 6> tmp_config;
  double edge_dist = getDist(config1,config2);
  //printf("%f\n",edge_dist);
  int num_divided_edge= ceil(edge_dist/edge_resol);
  //printf("%d\n",num_divided_edge);
  if (num_divided_edge>1){
    for (int i=0; i < num_divided_edge+1; ++i){
  for (int j=0; j < config1.size(); ++j) 
    tmp_config[j] = ((double) i * config1[j] + ((double)num_divided_edge-(double)i) * config2[j]) /(double)num_divided_edge;
        if (nodeCollisionCheck(tmp_config, planning_scene, current_state, joint_model_group)) return true;
    }     
  }
  return false;
}

bool Collision_Checker::edgeCollisionCheck(vector<double> config1, vector<double> config2,double edge_resol, planning_scene::PlanningScene *planning_scene, robot_state::RobotState& current_state, const robot_model::JointModelGroup* joint_model_group)
{

  array<double, 6> tmp_config;
  double edge_dist = getDist(config1,config2);
  //printf("%f\n",edge_dist);
  int num_divided_edge= ceil(edge_dist/edge_resol);
  //printf("%d\n",num_divided_edge);
  if (num_divided_edge>1){
    for (int i=0; i < num_divided_edge+1; ++i){
  for (int j=0; j < config1.size(); ++j) 
    tmp_config[j] = ((double) i * config1[j] + ((double)num_divided_edge-(double)i) * config2[j]) /(double)num_divided_edge;
        if (nodeCollisionCheck(tmp_config, planning_scene, current_state, joint_model_group)) return true;
    }     
  }
  return false;
}