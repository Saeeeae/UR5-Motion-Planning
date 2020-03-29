#pragma once
#include "collision_check.h"

class Node
{
protected:
    int node_id;
    vector<double> JointValue;           // double q1,q2,q3,q4,q5,q6;
    Joint3dPosition joint3dPostition[JointNum]; // double x1,y1,z1,x2,y2,z2,x3,y3,z3,x4,y4,z4,x5,y5,z5,x6,y6,z6;
    double manip;
    bool node_Val;  //node validation check
public:
    
    Node();
    Node(int id);

    double setRnd(double low, double high);

    void printJoint();

    void set_Id(int id);

    void set_JointValue();

    void delete_JointValue();
    void getFK();
    
    void setNeighbors();

    int getId(){return node_id;}

    void setId(int id){this->node_id = id;}

    vector<double> getJointValue()
    {
        return this->JointValue;
    }

    void setJointValue(vector<double> vt)
    {
        this->JointValue = vt;
    }

    Joint3dPosition* getJoint3dPosition()
    {
        return this->joint3dPostition;
    }

    void setJoint3dPosition(Joint3dPosition j[])
    {
        for(int i = 0 ; i< JointNum; i++)
        {
            this->joint3dPostition[i].x=j[i].x;
            this->joint3dPostition[i].y=j[i].y;
            this->joint3dPostition[i].z=j[i].z;
        }
    }

    double getManip()
    {
        return this->manip;
    }

    void setManip(double m)
    {
        this->manip = m;
    }

    bool getNodeVal()
    {
        return this->node_Val;
    }

    void setNodeVal(bool val )
    {
        this->node_Val = val;
    }

    friend class Graph;
};
