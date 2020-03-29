#pragma once
#include <iostream>
#include <stdlib.h>
#include <vector>
#include <ctime>
#include <queue>
#include <fstream>
#include <limits>
#include <time.h>
#include <eigen3/Eigen/Dense>

////ros
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>

#include <moveit/kinematic_constraints/utils.h>
#include <eigen_conversions/eigen_msg.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
////

using namespace std;
using namespace Eigen;
#define PI 3.14159265358979323846
#define INF std::numeric_limits<double>::infinity()
#define NumV 100
#define JointNum 6
#define Restriction 0.6 // add kNN 
// notation of ur5
const double a[6] = {0,-0.425,-0.39225,0,0,0};
const double al[6] = {PI/2,0,0,PI/2,-PI/2,0};
const double d[6] = {0.08916,0,0,0.10915,0.09456,0.0823};


typedef struct Joint3dPosition{
     double x;
     double y;
     double z;  
};