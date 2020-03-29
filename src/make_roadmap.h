#pragma once
#include "create_Graph.h"

void writeFile_Node(Node vertex[])
{
    ofstream writeFile;
    writeFile.open("/home/sae/catkin_ws/src/prm/Node.txt");

    for (int i = 0; i < NumV; i++)
    {
        writeFile<< vertex[i].getId()<<endl;
        for (int j = 0; j < JointNum; j++)
        {
            writeFile << vertex[i].getJointValue()[j]<<" ";
        }
        writeFile<<endl;

        for (int j = 0; j < JointNum; j++)
        {
                writeFile << vertex[i].getJoint3dPosition()[j].x<<" "<<vertex[i].getJoint3dPosition()[j].y<<" "<<vertex[i].getJoint3dPosition()[j].z<<" ";
        }
        writeFile<<endl<<vertex[i].getManip()<<endl;
        writeFile<<vertex[i].getNodeVal()<<endl;
    }
    writeFile.close();
}

void writeFile_Graph(Graph g)
{
    ofstream writeFileG;
    writeFileG.open("/home/sae/catkin_ws/src/prm/Graph.txt");

    vector<vector< pair < int, double > > > gph = g.getGraph();
    vector<vector< pair < int, bool > > > eVal = g.getEdgeVal();
    for (int i = 0; i < gph.size(); i++)
    {
        for (int j = 0; j < gph[i].size(); j++)
        {
            writeFileG << i << " " << gph[i][j].first << " " << gph[i][j].second << " " << eVal[i][j].second << endl;
        }
    }
    
    writeFileG.close();
}

void readFile_Node(Node vertex[])
{
    
    int id;
    double q;
    double x,y,z;
    double heur;
    double manip;
    bool val;
    int num = NumV;
    ifstream in;
    in.open("/home/sae/catkin_ws/src/prm/Node.txt");
    if(in.is_open()==false)
    {
        cout<< "file open failed!\n";
    }
    else
    {
        while(in>>id)
        {
            
            vector<double> j;
            Joint3dPosition jp[JointNum];
            for (int i = 0; i < JointNum; i++)
            {
                in>>q;
                j.push_back(q);
                //cout<<q<<" q ";
            }

            for (int i = 0; i < JointNum; i++)
            {
                in>>x>>y>>z;
                jp[i].x=x;
                jp[i].y=y;
                jp[i].z=z;
                
            }
            
            in >> manip;
            in >> val;
            vertex[id].setId(id);
            vertex[id].setJointValue(j);
            vertex[id].setJoint3dPosition(jp);
            vertex[id].setManip(manip);
            vertex[id].setNodeVal(val);
        }
    }
    if(in.is_open()==true)in.close();

    cout<<"NODE FINSIHED\n";
}

Graph readFile_Graph()
{
    Graph g;
    ifstream ing;
    vector<vector< pair < int, double > > > gph;
    vector<vector< pair < int, bool > > > eVal;
    gph.resize(NumV);
    eVal.resize(NumV);
    int from,to;
    double dist;
    bool edge_Val;
    ing.open("/home/sae/catkin_ws/src/prm/Graph.txt");
    if(ing.is_open()==false)
    {
        cout<<"file open failed!\n";
    }
    else
    {
        while(ing >> from >> to >> dist >> edge_Val)
        {
            gph[from].push_back({to,dist});
            eVal[from].push_back({to,edge_Val});
        }
    }
    if(ing.is_open()==true)ing.close();
    cout<<"SetGraph start\n";
    g.setGraph(gph);
    cout<<"SetEdge start\n";
    g.setEdgeVal(eVal);
    return g;
}