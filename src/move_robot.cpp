#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <vector>
#include <array>
#include "robotArm.h"
using namespace std;
typedef actionlib::SimpleActionClient< control_msgs::FollowJointTrajectoryAction > TrajClient;


vector< vector < double > > getVT()
{
  vector< vector < double > > tmp(5);
  tmp[0].push_back(0.0);
  tmp[0].push_back(0.0);
  tmp[0].push_back(0.0);
  tmp[0].push_back(0.0);
  tmp[0].push_back(0.0);
  tmp[0].push_back(0.0);


  tmp[1].push_back(1.0);
  tmp[1].push_back(0.0);
  tmp[1].push_back(0.0);
  tmp[1].push_back(0.0);
  tmp[1].push_back(0.0);
  tmp[1].push_back(0.0);


  tmp[2].push_back(0.0);
  tmp[2].push_back(0.0);
  tmp[2].push_back(-1.0);
  tmp[2].push_back(0.0);
  tmp[2].push_back(0.0);
  tmp[2].push_back(0.0);


  tmp[3].push_back(0.0);
  tmp[3].push_back(0.0);
  tmp[3].push_back(0.0);
  tmp[3].push_back(0.0);
  tmp[3].push_back(0.0);
  tmp[3].push_back(-1.0);


  tmp[4].push_back(0.0);
  tmp[4].push_back(-1.0);
  tmp[4].push_back(0.0);
  tmp[4].push_back(0.0);
  tmp[4].push_back(0.0);
  tmp[4].push_back(0.0);

  return tmp;
}

vector<array < double , 6  > > getAT()
{
  vector<array< double , 6 > > tmp(5);
  tmp[0][0]=0.0;
  tmp[0][1]=0.0;
  tmp[0][2]=0.0;
  tmp[0][3]=0.0;
  tmp[0][4]=0.0;
  tmp[0][5]=0.0;

  tmp[1][0]=2.0;
  tmp[1][1]=0.0;
  tmp[1][2]=0.0;
  tmp[1][3]=0.0;
  tmp[1][4]=0.0;
  tmp[1][5]=0.0;

  tmp[2][0]=0.0;
  tmp[2][1]=0.0;
  tmp[2][2]=-2.0;
  tmp[2][3]=0.0;
  tmp[2][4]=0.0;
  tmp[2][5]=0.0;

  tmp[3][0]=0.0;
  tmp[3][1]=0.0;
  tmp[3][2]=0.0;
  tmp[3][3]=0.0;
  tmp[3][4]=0.0;
  tmp[3][5]=-2.0;

  tmp[4][0]=0.0;
  tmp[4][1]=-1.0;
  tmp[4][2]=0.0;
  tmp[4][3]=0.0;
  tmp[4][4]=0.0;
  tmp[4][5]=0.0;

  return tmp;
}

int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "robot_driver");

  vector < array < double , 6 > > vt = getAT();

  RobotArm arm;
  // Start the trajectory
  arm.startTrajectory(arm.armExtensionTrajectory(vt));
  // Wait for trajectory completion
  while(!arm.getState().isDone() && ros::ok())
  {
    usleep(50000);
  }
  return 0;
}

