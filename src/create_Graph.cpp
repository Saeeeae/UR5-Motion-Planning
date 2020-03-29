#include "create_Graph.h"

Graph::Graph(){}

void Graph::makeGraph(Node node[],bool map[][NumV]){     

    graph.resize(NumV);
    edge_Val.resize(NumV);

    for (int i = 0; i < NumV; i++)
    {
        for (int j = i; j < NumV; j++)
        {
            if(i==j)continue;

            // if(chk.edgeCollisionCheck(node[i].getJointValue(),node[j].getJointValue(), 0.1, planning_scene, current_state, joint_model_group));
            //     continue;

            double dist = calDist_WS(node[i],node[j]);

            //double dist = calDist_WS(node[i],node[j]);
            //double dist = calDist_Mid(node[i],node[j]);
            //cout<<dist<<"***\n";
            if(dist<Restriction && map[i][j]) //노드 1,2사이 거리가 0.5보다작으면 간선연결해줌
            {
                graph[i].push_back({j, dist});
                graph[j].push_back({i, dist});
                edge_Val[i].push_back({j, true});
                edge_Val[j].push_back({i, true});
            } 
                //여기서 g[i][j]=g[j][i]처리하면 그래프형성 2배빨라짐
                //거리를얼마나주느냐에따라 간선형성하거나 안하거나
        }
    }

}

double Graph::calDist_CS(Node node1,Node node2)
{   
    //q1~q6 거리합
    double dist_tmp = 0;
    for (int i = 0; i < JointNum; i++)
    {
        dist_tmp += pow((node1.JointValue[i]-node2.JointValue[i]),(double)2);
    }
    double dist = sqrt(dist_tmp);

    return dist;
}

double Graph::calDist_WS(Node node1,Node node2)
{
    double dist_tmp = -1;
    for (int i = 0; i < JointNum; i++)
    {
        double tmp = pow((node1.joint3dPostition[i].x-node2.joint3dPostition[i].x),2)+pow((node1.joint3dPostition[i].y-node2.joint3dPostition[i].y),2)+pow((node1.joint3dPostition[i].z-node2.joint3dPostition[i].z),2);
        dist_tmp = dist_tmp > tmp ? dist_tmp : tmp;
    }
    double dist = sqrt(dist_tmp);

    return dist;
}

double Graph::calDist_Mid(Node node1,Node node2)
{   //q1~q6 거리합
    Node midPointNode;

    for (int i = 0; i < JointNum; i++)
    {
        midPointNode.JointValue.push_back((node1.JointValue[i]+node2.JointValue[i])/2);
    }
//           startt=clock();
    midPointNode.getFK();
//        endd=clock();
//        cout<<"#####"<<(double)(endd-startt)/CLOCKS_PER_SEC<<endl;
    double dist_tmp = -1;

    for (int i = 0; i < JointNum; i++)
    {
        double tmp = pow((node1.joint3dPostition[i].x-midPointNode.joint3dPostition[i].x),2)+pow((node1.joint3dPostition[i].y-midPointNode.joint3dPostition[i].y),2)+pow((node1.joint3dPostition[i].z-midPointNode.joint3dPostition[i].z),2)
        + pow((midPointNode.joint3dPostition[i].x-node2.joint3dPostition[i].x),2)+pow((midPointNode.joint3dPostition[i].y-node2.joint3dPostition[i].y),2)+pow((midPointNode.joint3dPostition[i].z-node2.joint3dPostition[i].z),2);
        dist_tmp = dist_tmp > tmp ? dist_tmp : tmp;
    }

    double dist = sqrt(dist_tmp);

    return dist;
}

void Graph::printGraph()
{
    for (int i = 0; i < graph.size(); i++)
    {
        cout<<i<<"번째 노드와 연결되어있는 노드들 : \n";
        for(int j=0; j<graph[i].size();j++)
            cout<<graph[i][j].first<<"노드 거리: " <<graph[i][j].second <<endl;
    }
}

double Graph::calHuristic(Node current, Node goal)
{    //this->heuristic= final node x,y,z와 현재노드간 거리
    double dist = calDist_CS(current,goal);
    return dist;
}

void Graph::setGraph(vector<vector< pair < int, double > > > g)
{
    this->graph = g;
}

vector<vector< pair < int, double > > > Graph::getGraph()
{
    return this->graph;
}

void Graph::setEdgeVal(vector<vector< pair < int, bool > > >edg)
{
    this->edge_Val = edg;
}

vector<vector< pair < int, bool > > > Graph::getEdgeVal()
{
    return this->edge_Val;
}

void Graph::aStar(Node node[], int startNodeId,int finalNodeId) //astar로변경
{
    bool Closed_Set[NumV];
    double gScore[NumV];
    double fScore[NumV];

    int cameFrom[NumV];
    for (int i = 0; i < NumV; i++) // 시작시 모든노드들이 방문되지않았으니 false 방문되면 true
    {    
        gScore[i]=INF;
        fScore[i]=INF;
        Closed_Set[i]=false;
        cameFrom[i]=-1;
    }
    gScore[startNodeId]=0;
    fScore[startNodeId]=calHuristic(node[startNodeId],node[finalNodeId]);
    priority_queue< pair<double,int> > Open_Set; 
    
    Open_Set.push( { -fScore[startNodeId], startNodeId } );
    //operator 정의해주야댐

    // edge, node validation chk

    while (!Open_Set.empty())
    {
        double current_Fcost = -Open_Set.top().first;   //start~현재 노드 distance값
        int current_Node_Id= Open_Set.top().second;     //현재 노드 id값
        Node current_Node = node[current_Node_Id];               //현재 노드

        if(current_Node_Id==finalNodeId)         //현재노드가 목적지라면 종료
        {
            cout<<"Connected!\n"<<current_Fcost<<endl;
            break;
        }
        
        Open_Set.pop();
        Closed_Set[current_Node_Id]=false;


        for (int i = 0; i < graph[current_Node_Id].size(); i++) //현재노드에서의 이웃 노드들
        {
            int Neighbor_Node_Id = graph[current_Node_Id][i].first;
            Node* Neighbor_Node = &node[Neighbor_Node_Id];
            // 포인터로 실제 저장된 노드들 배열에 접근해주어야함
            // 포인터로 접근하지않으면 실제 저장된 노드들 배열에는 parent Id가 -1로 되어있음
            // ex) 포인터 swap과 같은 개념

            if(!Neighbor_Node->node_Val||!edge_Val[current_Node_Id][i].second)
                continue;

            double tentative_gScore = gScore[current_Node_Id] + graph[current_Node_Id][i].second;
            //double Next_Fcost = graph[Neighbor_Node_Id][i].second + current_Fcost;// + calHuristic; //fcost

            if(tentative_gScore<gScore[Neighbor_Node_Id])
            {
                cameFrom[Neighbor_Node_Id]=current_Node_Id;
                gScore[Neighbor_Node_Id]=tentative_gScore;
                fScore[Neighbor_Node_Id]=gScore[Neighbor_Node_Id]+calHuristic(node[Neighbor_Node_Id],node[finalNodeId]);
                if(Closed_Set[Neighbor_Node_Id]==false)
                {
                    Closed_Set[Neighbor_Node_Id]=true;
                    Open_Set.push({-fScore[Neighbor_Node_Id],Neighbor_Node_Id});
                }
            }

        }
    }


    int tmp = finalNodeId;
    while(cameFrom[tmp]!=-1)
    {
        cout<<tmp<<" ";
        tmp = cameFrom[tmp];
    }
    cout<<tmp<<endl;
    cout<<gScore[finalNodeId]<<endl;
}    
