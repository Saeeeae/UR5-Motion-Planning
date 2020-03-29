#include "make_roadmap.h"

int main()
{
    Node vertex[NumV];
    readFile_Node(vertex);
    Graph g = readFile_Graph();
    
    //g.printGraph();
    cout<<"Astar start\n";

    g.aStar(vertex,0,NumV-1);
    cout<<"Finished!";
}