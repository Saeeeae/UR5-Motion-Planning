#pragma once
#include "create_Node.h"


class Graph{
private:
    vector<vector< pair < int, double > > > graph;
    vector<vector< pair < int, bool > > > edge_Val; //edge validation chk (node i-> node j = true/false)
public:
    Graph();

    void makeGraph(Node node[],bool map[][NumV]);

    double calDist_CS(Node node1,Node node2);

    double calDist_WS(Node node1,Node node2);

    double calDist_Mid(Node node1,Node node2);

    void printGraph();

    double calHuristic(Node current, Node goal);

    void setGraph(vector<vector< pair < int, double > > > g);

    vector<vector< pair < int, double > > > getGraph();

    void setEdgeVal(vector<vector< pair < int, bool > > >edg);

    vector<vector< pair < int, bool > > > getEdgeVal();

    void aStar(Node node[], int startNodeId,int finalNodeId);

};