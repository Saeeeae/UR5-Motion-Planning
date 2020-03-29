#pragma once
#include "using_lib.h"

class Collision_Checker{
private:

public:
    Collision_Checker();
    //Collision_Checker
    double getDist(array<double, 6> config1, array<double, 6> config2);

    double getDist(vector<double> config1, vector<double> config2);

    bool nodeCollisionCheck(array<double, 6> config, planning_scene::PlanningScene *planning_scene, robot_state::RobotState& current_state, const robot_model::JointModelGroup* joint_model_group);

    bool nodeCollisionCheck(vector<double> config, planning_scene::PlanningScene *planning_scene, robot_state::RobotState& current_state, const robot_model::JointModelGroup* joint_model_group);

    bool edgeCollisionCheck(array<double, 6> config1, array<double, 6> config2,double edge_resol, planning_scene::PlanningScene *planning_scene, robot_state::RobotState& current_state, const robot_model::JointModelGroup* joint_model_group);

    bool edgeCollisionCheck(vector<double> config1, vector<double> config2,double edge_resol, planning_scene::PlanningScene *planning_scene, robot_state::RobotState& current_state, const robot_model::JointModelGroup* joint_model_group);

};