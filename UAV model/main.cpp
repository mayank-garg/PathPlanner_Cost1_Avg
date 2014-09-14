
#ifdef __APPLE__
#include <OpenGL/OpenGL.h>
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include <stdlib.h>
#include<iostream>
#include<stdio.h>
#include <time.h>

#define pi 3.142857
#define PI 3.14159265

#include <fstream>
#include "quad.hpp"
#include "terrain.hpp"
#include "functions.hpp"
#include "openGlStuff.hpp"

#include "simulator.hpp"

using namespace std;
bool noCollision(int xIn, int yIn,float hIn, int xFin, int yFin, float hFin);
float valAssign(float start, float diff, float inc_factor);
float incFactor(float del, float diff);
bool inRange(float a, float b, float c, float d, float e, float f);
void pathCalculate(int x, int y,float z);
float eucl_distance(float dX0, float dY0, float dZ0, float dX1, float dY1,float dZ1);
float eucl_distane_2d(float dX0, float dY0, float dX1, float dY1);

int path_count = 0;

float maxHeight = 100;
float tilt = 30;
vector<struct pathSim> TempUavPath;

int main(int argc, char** argv)
{
    /*
    double W0[2] = {0,0};
    double W1[2] = {3000,3000};

    return simulation(W0,W1,-pi);
    */

    int x;
    int y;
    float z;
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(400, 400);

	glutCreateWindow("Terrain");
	initRendering();

	_terrain = loadTerrain("E:/STUDY/6th Sem/BTP/quadtree/images/him_nepal.bmp", maxHeight);

    cout<<"\nTerrain width"<<_terrain->width();
    cout<<"\nTerrain Length"<<_terrain->length();


    Node * rootNode = new Node;
    setnode(rootNode, _terrain->width(), _terrain->length());
    buildQuadTree(rootNode);
    landingSite(rootNode);
    printSite(rootNode);

    cout<<"\nLanding Site : "<<landSite;
    cout<<"\nSuccess";

    cout<<"\nProvide your Location (x,y,z) : ";
    cin>>x;
    cin>>y;
    cin>>z;

    z = _terrain->getHeight(x,y);// +60;
    cout<<"\nHeight of UAV: "<<z;

    time_t timer1;
    time_t timer2;
    double seconds;

    time(&timer1);
    pathCalculate(x,y,z);
    time(&timer2);
    seconds = difftime(timer2,timer1);

    cout<<"\nSeconds : "<<seconds;

	glutDisplayFunc(drawScene);
	glutKeyboardFunc(handleKeypress);
	glutReshapeFunc(handleResize);
	glutTimerFunc(25, update, 0);

	glutMainLoop();

	return 0;

}

void pathCalculate(int x, int y,float z)
{

    float random_x;  //To generate random functions
    float random_y;
    float random_z;
    float node_dist;       //Distance of random point from nodes in tree
    float temp_dist;       //Store temporary distance
    float del_q = 100;            //Delta distance to be moved in the direction of random point
    float inc_factorX = 0;       //inc_factor = del_q/(distance between random point and nearest point)
    float inc_factorY = 0;
    float node_count = 0;
    int pos = 0;
    int temp_id;
    int avoid_obs = 0;
    float diffX = 0;
    float diffY = 0;
    float diffZ = 0;

    float base = 0;
    float hypot = 0; //To find tilt/elevation angle from one point to another
    float height_angle = 0;

    float slope = 0;
    int flag = 0;

    float diffRange = 0;


    for(int num =landSite-2; num<landSite-1;num++)
    {
        path_count = 0;
        flag = 0;
        node_count = 0;

        //Adding 1st Node
        temp.node_id = 1;
        temp.self_x = x;
        temp.self_y = y;
        temp.self_z = z;
        temp.node_parent_x = 0;
        temp.node_parent_y = 0;
        temp.node_parent_z = 0;
        temp.parent_id = 0;
        rrt_tree.push_back(temp);
        //1st Node Added

        node_count++;

        int flag2 = 0;
        int localCount = 0;    // To keep track of for how many points,Local Sampling has been performed
        int maxLocalCount = 1000;
        int fail = 1;
        int pointCount = 0;
        int maxPointCount = 30000;

        while(flag == 0)
        {

            if(flag2 == 0)
            {
            random_x = (int)((_terrain->width()-1) * ((float)rand()/(float)RAND_MAX));
            random_y = (int)((_terrain->length()-1) * ((float)rand()/(float)RAND_MAX));
            }
            else
            {
                if(localCount < maxLocalCount)
                {
                    localCount++;
                    diffRange = goodPoint[num].endPointX - goodPoint[num].startPointX + 20;
                    random_x = goodPoint[num].startPointX -10 + (int)(diffRange * ((float)rand()/(float)RAND_MAX));

                    diffRange = goodPoint[num].endPointY - goodPoint[num].startPointY + 20;
                    random_y = goodPoint[num].startPointY -10 + (int)(diffRange * ((float)rand()/(float)RAND_MAX));

                }
                else
                {
                    flag2 = 0;
                    cout<<"\n"<<maxLocalCount<<" points checked!";
                    cout<<"\n Node Count "<<node_count;
                }

            }

            node_dist = sqrt(_terrain->width()*_terrain->width() + _terrain->length()*_terrain->length());

            for(int i=0;i<node_count;i++)
            {
                temp_dist = eucl_distance(rrt_tree[i].self_x,rrt_tree[i].self_y,rrt_tree[i].self_z,random_x,random_y,random_z);
                if(temp_dist<node_dist)
                {
                    node_dist = temp_dist;
                    temp_id = rrt_tree[i].node_id;
                    pos = i;
                }
            }

            diffX = random_x - rrt_tree[pos].self_x;
            diffY = random_y - rrt_tree[pos].self_y;

            inc_factorX = incFactor(del_q,diffX);
            inc_factorY = incFactor(del_q,diffY);

            temp.self_x = (int)valAssign(rrt_tree[pos].self_x, diffX, inc_factorX);
            temp.self_y = (int)valAssign(rrt_tree[pos].self_y, diffY, inc_factorY);

            height_angle = ((rand() % ((int)(2*tilt))) - tilt)*PI/180;
            base = eucl_distance(temp.self_x,temp.self_y,0,rrt_tree[pos].self_x,rrt_tree[pos].self_y,0);

            temp.self_z =  tan(height_angle)*base + rrt_tree[pos].self_z;//(int)(maxHeight * ((float)rand()/(float)RAND_MAX));

            if((temp.self_z < _terrain->getHeight(temp.self_x,temp.self_y)))
            {
                avoid_obs = 1;
                fail++;
            }
/*
            if(fail%1000 == 0)
            {
                cout<<"\nFailed";
                fail = 1;
                cout<<"\tNode Count"<<node_count;
            }
*/
            if(avoid_obs == 1)
            {
                if(inRange(temp.self_x, goodPoint[num].startPointX-5, goodPoint[num].endPointX+5,
                   temp.self_y, goodPoint[num].startPointY-5, goodPoint[num].endPointY+5))
                {
                    if(inRange(temp.self_x, goodPoint[num].startPointX, goodPoint[num].endPointX,
                    temp.self_y, goodPoint[num].startPointY, goodPoint[num].endPointY))
                    {
                        if(abs(temp.self_z - _terrain->getHeight(temp.self_x,temp.self_y)) <5)
                        {
                            avoid_obs = 0;
                            cout<<"\n Obtained Height = "<<temp.self_z <<" and Avg Height = "<<goodPoint[num].avgHeight;
                        }
                    }
                    else if(flag2 == 0)
                    {
                        flag2 = 1;
                        localCount = 0;
                        cout<<"\nLocal Sampling Started, Node Count = "<<node_count;
                    }
                }
            }

            if(avoid_obs == 0)
            {
                double X[2] = {rrt_tree[pos].self_x, rrt_tree[pos].self_y};
                double Y[2] = {temp.self_x, temp.self_y};
                float ang = rrt_tree[pos].headingAngle;


                node_count++;
                temp.node_id = node_count;
                temp.parent_id = temp_id;
                temp.node_parent_x = rrt_tree[pos].self_x;
                temp.node_parent_y = rrt_tree[pos].self_y;
                temp.node_parent_z = rrt_tree[pos].self_z;
                rrt_tree.push_back(temp);

                if(inRange(temp.self_x, goodPoint[num].startPointX, goodPoint[num].endPointX,
                           temp.self_y, goodPoint[num].startPointY, goodPoint[num].endPointY))
                {
                    if(abs(temp.self_z - _terrain->getHeight(temp.self_x,temp.self_y)) <5)
                    {
                        cout<<"\nDestination has arrived..!!!";
                        flag =1;
                    }
                }

            }
            avoid_obs = 0;
        }

//newPath = rrt_tree;
//newPathCount = node_count;

        int node_temp = node_count-1;
        temp.node_id = 0;
        path.push_back(temp);
        path_count++;


        while(rrt_tree[node_temp].node_id != 1)
        {

            path.push_back(rrt_tree[node_temp]);
            node_temp = rrt_tree[node_temp].parent_id-1;
            path_count++;
        }
        cout<<"\nPath count : "<<path_count;

        if(newPathCount == -1)
        {
            newPathCount = path_count;
            newPath = path;

        }

        else
        {
            if(path_count < newPathCount)
            {
                newPath.clear();

                newPathCount = path_count;
                newPath = path;
                cout<<"\nNew Path Count : "<<newPathCount;
            }
        }
        rrt_tree.clear();
        path.clear();

    }

    int start = 0;
    temp = newPath[start];
    finalPath.push_back(temp);

    for(int i=start+2; i<newPathCount;i++)
    {

        if(!(noCollision(finalPath[start].self_x, finalPath[start].self_y, finalPath[start].self_z,
                         newPath[i].self_x, newPath[i].self_y, newPath[i].self_z)))
        {
            //cout<<"\nX : "<<finalPath[start].self_x;
            //cout<<"\nY : "<<finalPath[start].self_y;
            temp.self_x = newPath[i-1].self_x;
            temp.self_y = newPath[i-1].self_y;
            temp.self_z = newPath[i-1].self_z;
            temp.headingAngle = 0;
            temp.node_id = start+1;
            temp.node_parent_x = finalPath[start].self_x;
            temp.node_parent_y = finalPath[start].self_y;
            temp.node_parent_z = finalPath[start].self_z;
            temp.parent_id = start;
            finalPath.push_back(temp);
            start++;
        }
    }
    temp = newPath[newPathCount-1];
    finalPath.push_back(temp);
    start++;

    cout<<"\nStart : "<<start;

    //newPath.clear();
    //newPath = finalPath;
    record = start+2;
    //record = newPathCount;
    //newPathCount = start+1;


/*
    record = 0;
    FILE *fx,*fy;
    fx = fopen("x.txt","w");
    fy = fopen("y.txt","w");
    double pointsArr[newPathCount][2], heightArr[newPathCount];
    int j = 0;
    for(int i =newPathCount-1;i>=0;i--)
    {

    fprintf(fx,"\n%f",newPath[i].self_x);
    fprintf(fy,"\n%f",newPath[i].self_y);
    pointsArr[j][0] = newPath[i].self_x;
    pointsArr[j][1] = newPath[i].self_y;

    heightArr[j] = newPath[i].self_z;
    j++;
    }
    fclose(fx);
    fclose(fy);

    TempUavPath = simulation(pointsArr,heightArr,newPathCount,record);

    FILE *fx1,*fy1;
    fx1 = fopen("xSim.txt","w");
    fy1 = fopen("ySim.txt","w");
    //double pointsArr[newPathCount][2], heightArr[newPathCount];
    newPath.clear();

    for(int i =0;i<record;i++)
    {
        temp.self_x = TempUavPath[i].x;
        temp.self_y = TempUavPath[i].y;
        temp.self_z = TempUavPath[i].z;
    //newPath[i].self_x = TempUavPath[i].x;
    //newPath[i].self_y = TempUavPath[i].y;
    //newPath[i].self_z = 100;
        newPath.push_back(temp);
    fprintf(fx1,"\n%f",newPath[i].self_x);
    fprintf(fy1,"\n%f",newPath[i].self_y);
    //TempUavPath[i].z = 100;
    }
    fclose(fx1);
    fclose(fy1);

*/

}

bool noCollision(int xIn, int yIn,float hIn, int xFin, int yFin, float hFin)
{
    float heightSum = 0;
    float avgHeight = 0;
    int len = abs(xFin - xIn);
    int width = abs(yFin - yIn);

    for(int x = min(xIn,xFin);x<= max(xFin,xIn);x++)
    {
        for(int y = min(yIn,yFin) ;y<= max(yFin,yIn) ; y++)
            heightSum = heightSum + _terrain->getHeight(x,y);
    }

    avgHeight = heightSum/(len*width);

    if(avgHeight < hIn && avgHeight < hFin)
        return true;
    else
        return false;

}
float valAssign(float start, float diff, float inc_factor)
{
    if(diff == 0)
        return start;
    else
        return (start + (diff*inc_factor));

}

float incFactor(float del, float diff)
{
    if(abs(diff) > del)
        return  (del/abs(diff));
    else
        return 1;

}

bool inRange(float a, float b, float c, float d, float e, float f)
{
    if(a>=b and a<=c)
    {
        if(d>=e and d<=f)
            return true;
        else
            return false;
    }
    else
        return false;
}

float eucl_distance(float dX0, float dY0, float dZ0, float dX1, float dY1,float dZ1)
{
    return sqrt((dX1 - dX0)*(dX1 - dX0) + (dY1 - dY0)*(dY1 - dY0) + (dZ1 - dZ0)*(dZ1 - dZ0));
}

float eucl_distane_2d(float dX0, float dY0, float dX1, float dY1)
{
    return sqrt((dX1 - dX0)*(dX1 - dX0) + (dY1 - dY0)*(dY1 - dY0));
}


