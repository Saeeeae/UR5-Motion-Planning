#ifndef ROBOTARM_H
#define ROBOTARM_H
#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <vector>
#include <array>

using namespace std;
typedef actionlib::SimpleActionClient< control_msgs::FollowJointTrajectoryAction > TrajClient;

class RobotArm
{
private:

  TrajClient* traj_client_;

public:

  RobotArm();   //robotarm init

  ~RobotArm();  //robotarm delete

  //send goal to ur5
  void startTrajectory(control_msgs::FollowJointTrajectoryGoal goal);

  //get joint val by vector and set goal
  control_msgs::FollowJointTrajectoryGoal armExtensionTrajectory(vector < array < double , 6 > > vt);

  //get action state
  actionlib::SimpleClientGoalState getState();
  
};
#endif