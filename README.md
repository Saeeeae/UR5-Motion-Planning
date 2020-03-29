# UR5 Motion Planning
It's about Probabilistic Roadmap Method (PRM). Using a UR5 Robot Arm. The PRM planner uses a graph of task poses to find a path between the start and goal poses.


# Installation
1. Setting ROS workspace
2. Setting gazebo and moveit
3. Install ur5.launch file in ROS workspace

# Usage
1. `catkin make`
2. `roslaunch ur_gazebo ur5.launch`
3. `roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch sim:=true`
4. `rosrun prm prm_prm`
5. `rosrun prm prm_astar` 

