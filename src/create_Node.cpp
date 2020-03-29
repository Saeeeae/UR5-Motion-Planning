#include "create_Node.h"
    
    Node::Node(){node_Val=true;}
    Node::Node(int id)
    {   
        node_id = id;
        node_Val=true;
        // for(int i=0; i<JointNum;i++)
        //     JointValue.push_back(setRnd(-3.14,3.14));
    }

    double Node::setRnd(double low, double high)
    {
        return ((high-low)*((double)rand()/RAND_MAX))+low;
    }

    void Node::printJoint()
    {
        for (int i = 0; i < JointValue.size(); i++)
            cout<< JointValue[i]<<" ";
    }

    void Node::set_Id(int id)
    {
        this->node_id = id;
    }

    void Node::set_JointValue()
    {
        //끝노드와 현재노드 xyz값 dist 계산해서 저장하기
        //랜덤범위 -3.14 ~ 3.14
        for(int i=0; i<JointNum;i++)
            JointValue.push_back(setRnd(-3.14,3.14));
    }

    void Node::delete_JointValue()
    {
        for (int i = 0; i < JointNum; i++)
            JointValue.pop_back();   
    }

    void Node::getFK()
    {

        MatrixXd m1 = MatrixXd(4,4);
        MatrixXd m2 = MatrixXd(4,4);
        MatrixXd m3 = MatrixXd(4,4);
        MatrixXd m4 = MatrixXd(4,4);
        MatrixXd m5 = MatrixXd(4,4);
        MatrixXd m6 = MatrixXd(4,4);
        MatrixXd m7 = MatrixXd(4,4);

        m1 << cos(JointValue[0]),-sin(JointValue[0])*cos(al[0]),sin(JointValue[0])*sin(al[0]),a[0]*cos(JointValue[0]),sin(JointValue[0]),cos(JointValue[0])*cos(al[0]),-cos(JointValue[0])*sin(al[0]),a[0]*sin(JointValue[0]),0,sin(al[0]),cos(al[0]),d[0],0,0,0,1;
        m2 << cos(JointValue[1]),-sin(JointValue[1])*cos(al[1]),sin(JointValue[1])*sin(al[1]),a[1]*cos(JointValue[1]),sin(JointValue[1]),cos(JointValue[1])*cos(al[1]),-cos(JointValue[1])*sin(al[1]),a[1]*sin(JointValue[1]),0,sin(al[1]),cos(al[1]),d[1],0,0,0,1;
        m3 << cos(JointValue[2]),-sin(JointValue[2])*cos(al[2]),sin(JointValue[2])*sin(al[2]),a[2]*cos(JointValue[2]),sin(JointValue[2]),cos(JointValue[2])*cos(al[2]),-cos(JointValue[2])*sin(al[2]),a[2]*sin(JointValue[2]),0,sin(al[2]),cos(al[2]),d[2],0,0,0,1;
        m4 << cos(JointValue[3]),-sin(JointValue[3])*cos(al[3]),sin(JointValue[3])*sin(al[3]),a[3]*cos(JointValue[3]),sin(JointValue[3]),cos(JointValue[3])*cos(al[3]),-cos(JointValue[3])*sin(al[3]),a[3]*sin(JointValue[3]),0,sin(al[3]),cos(al[3]),d[3],0,0,0,1;
        m5 << cos(JointValue[4]),-sin(JointValue[4])*cos(al[4]),sin(JointValue[4])*sin(al[4]),a[4]*cos(JointValue[4]),sin(JointValue[4]),cos(JointValue[4])*cos(al[4]),-cos(JointValue[4])*sin(al[4]),a[3]*sin(JointValue[4]),0,sin(al[4]),cos(al[4]),d[4],0,0,0,1;
        m5 << cos(JointValue[5]),-sin(JointValue[5])*cos(al[5]),sin(JointValue[5])*sin(al[5]),a[5]*cos(JointValue[5]),sin(JointValue[4]),cos(JointValue[5])*cos(al[5]),-cos(JointValue[5])*sin(al[5]),a[3]*sin(JointValue[5]),0,sin(al[5]),cos(al[5]),d[5],0,0,0,1;

        MatrixXd T2 = MatrixXd(4,4);
        MatrixXd T3 = MatrixXd(4,4);
        MatrixXd T4 = MatrixXd(4,4);
        MatrixXd T5 = MatrixXd(4,4);
        MatrixXd T6 = MatrixXd(4,4);
        
        T2=m1*m2;
        T3=T2*m3;
        T4=T3*m4;
        T5=T4*m5;
        T6=T5*m6;

        Vector3d z1(0,0,1); Vector3d z2; Vector3d z3; Vector3d z4; Vector3d z5; Vector3d z6;
        Vector3d p0(0,0,0); Vector3d p1; Vector3d p2; Vector3d p3; Vector3d p4; Vector3d p5; Vector3d P;
        
        for(int i=0; i<3;i++)
        {
            z2[i]=m1(i,2);
            z3[i]=T2(i,2);
            z4[i]=T3(i,2);
            z5[i]=T4(i,2);
            z6[i]=T5(i,2);

            p1[i]=m1(i,3);
            p2[i]=T2(i,3);
            p3[i]=T3(i,3);
            p4[i]=T4(i,3);
            p5[i]=T5(i,3);
            P[i]=T6(i,3);
        }

        //debug
        // cout<<"  x: "<<P[0]<<" y: "<<P[1]<<" z: "<<P[2]<<endl;
        // cout<<T6<<endl;
        // cout<<endl;

        joint3dPostition[0].x=p1[0]; joint3dPostition[0].y=p1[1]; joint3dPostition[0].z=p1[2];
        joint3dPostition[1].x=p2[0]; joint3dPostition[1].y=p2[1]; joint3dPostition[1].z=p2[2];
        joint3dPostition[2].x=p3[0]; joint3dPostition[2].y=p3[1]; joint3dPostition[2].z=p3[2];
        joint3dPostition[3].x=p4[0]; joint3dPostition[3].y=p4[1]; joint3dPostition[3].z=p4[2];
        joint3dPostition[4].x=p5[0]; joint3dPostition[4].y=p5[1]; joint3dPostition[4].z=p5[2];
        joint3dPostition[5].x=P[0]; joint3dPostition[5].y=P[1]; joint3dPostition[5].z=P[2];

        Vector3d c1; Vector3d c2; Vector3d c3; Vector3d c4; Vector3d c5; Vector3d c6;
        
        c1 = z1.cross(P-p0);
        c2 = z2.cross(P-p1);
        c3 = z3.cross(P-p2);
        c4 = z4.cross(P-p3);
        c5 = z5.cross(P-p4);
        c6 = z6.cross(P-p5);

        MatrixXd aa = MatrixXd(6,6);
        aa << c1,c2,c3,c4,c5,c6,z1,z2,z3,z4,z5,z6;

        double manip = abs(aa.determinant());

        this->manip = manip;
        
        //debug
        // cout<<aa<<endl<<endl;
        // cout<<c1<<endl<<z1<<endl<<endl;
    }
    
    void Node::setNeighbors()
    {

    }

//bool operator<(Node a,Node b){return a.getManip()<b.getManip();}