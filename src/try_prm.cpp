#include "make_roadmap.h"

//writing roadmap

int main(int argc, char **argv)
{

    ros::init (argc, argv, "UR5_APSC");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    planning_scene::PlanningScene planning_scene(kinematic_model);

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    robot_state::RobotState &current_state = planning_scene.getCurrentStateNonConst();

    const robot_model::JointModelGroup* joint_model_group = current_state.getJointModelGroup("manipulator");

    //current_state.setToRandomPositions();
    std::vector<double> joint_values;
    current_state.copyJointGroupPositions(joint_model_group, joint_values);

    Collision_Checker chk;

    Node vertex[NumV];
    Graph g;
    
    srand((unsigned int)time(0));

    for (int i = 0; i < NumV; i++)
    {
        vertex[i].set_Id(i);
        vertex[i].set_JointValue();
        while(chk.nodeCollisionCheck(vertex[i].getJointValue(),&planning_scene,current_state,joint_model_group))
        {
            //ROS_INFO_STREAM("Still Checking Node Collision...");
            vertex[i].delete_JointValue();
            vertex[i].set_JointValue();
        }
    }

    // vector< double > start_Joint;
    // vector< double > final_Joint;
    // // 
    // vertex[0].set_JointValue(start_Joint);         //
    // vertex[NumV-1].set_JointValue(final_Joint);    //   

    for (int i = 0; i < NumV; i++)
    {
        vertex[i].getFK();
    }

    ROS_INFO_STREAM("Vertex Created!");
    
    writeFile_Node(vertex);

    ROS_INFO_STREAM("Vertex File Written!");
    
    bool map[NumV][NumV] = {0,};


    for (int i = 0; i < NumV; i++)
    {
        for (int j = i; j < NumV; j++)
        {
            if(i==j)continue;

            // if(chk.edgeCollisionCheck(vertex[i].getJointValue(),vertex[j].getJointValue(), 0.1, &planning_scene, current_state, joint_model_group));
            //     continue;
            if(chk.edgeCollisionCheck(vertex[i].getJointValue(),vertex[j].getJointValue(), 0.1, &planning_scene, current_state, joint_model_group)==1)
            {continue;}
            
            ROS_INFO_STREAM("Edge Collision Checked!");

            map[i][j]=true;
            map[j][i]=true;
        }
    }

    g.makeGraph(vertex,map);    //edge collision chk
    //g.extension();
    ROS_INFO_STREAM("Graph Created!");

    writeFile_Graph(g);

    ROS_INFO_STREAM("Graph File Written!");
    
}