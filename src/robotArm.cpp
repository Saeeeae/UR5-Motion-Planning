#include "robotArm.h"

using namespace std;

  RobotArm::RobotArm() 
  {
    // tell the action client that we want to spin a thread by default
    traj_client_ = new TrajClient("arm_controller/follow_joint_trajectory", true);

    // wait for action server to come up
    while(!traj_client_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the joint_trajectory_action server");
    }
  }

  RobotArm::~RobotArm()
  {
    delete traj_client_;
  }

  void RobotArm::startTrajectory(control_msgs::FollowJointTrajectoryGoal goal)
  {
    goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    traj_client_->sendGoal(goal);
  }
  control_msgs::FollowJointTrajectoryGoal RobotArm::armExtensionTrajectory(vector < array < double , 6 > > vt)
  {
    // vector< vector< double > >vt(4);
    
    // 컴파일시 vector<vector<double>>vt(4); 다 붙여서 컴파일하면 오류뜰 경우 존재

    //목적한 trajectoryGoal
    control_msgs::FollowJointTrajectoryGoal goal;
    
    //set joints name
    goal.trajectory.joint_names.push_back("shoulder_pan_joint");
    goal.trajectory.joint_names.push_back("shoulder_lift_joint");
    goal.trajectory.joint_names.push_back("elbow_joint");
    goal.trajectory.joint_names.push_back("wrist_1_joint");
    goal.trajectory.joint_names.push_back("wrist_2_joint");
    goal.trajectory.joint_names.push_back("wrist_3_joint");    


    //vt 크기만큼 size조정해서 point에 joint값 삽입
    goal.trajectory.points.resize(vt.size());

    for (int i = 0; i < vt.size(); i++)
    {
      goal.trajectory.points[i].positions.resize(6);    // position 6개만큼 공간할당
      goal.trajectory.points[i].velocities.resize(6);   // 각 position velocity 할당
      for (int j = 0; j < 6; j++)
      {
        goal.trajectory.points[i].positions[j] = vt[i][j];  // set position joints
        goal.trajectory.points[i].velocities[j] = 0.0;  // vel = 0 setting
      }
      goal.trajectory.points[i].time_from_start = ros::Duration(i+3);
    }
    //오류뜰시 Duration 설정해주기 sequentially
    
    return goal;
  }

  //! Returns the current state of the action
  actionlib::SimpleClientGoalState RobotArm::getState()
  {
    return traj_client_->getState();
  }
