#ifndef QuadDeclared

#define QuadDeclared

#include<vector>
#include "terrain.hpp"

struct Node
{
	int bottomLeft_X;
	int bottomLeft_Y;
	int topRight_X;
	int topRight_Y;
	Node *child[4]= {NULL, NULL, NULL, NULL};			//Changed to Node *child[4] rather than Node ** child[4]
	float height = 0;
	float variance = 0;
	float size = 0;
	int potential_land = 0;
};

struct goodLand
{
    int startPointX;
    int startPointY;
    int endPointX;
    int endPointY;
    float avgHeight;
}tempLand;

struct pathNode
{
  int node_id;
  float self_x;
  float self_y;
  float self_z;
  float headingAngle;

  //Parents of the node
  int parent_id;
  float node_parent_x;
  float node_parent_y;
  float node_parent_z;

}temp,temp1,temp2;

vector<goodLand> goodPoint;
vector<pathNode> rrt_tree; //RRT Declared
vector<pathNode> path;
vector<pathNode> newPath;
vector<pathNode> finalPath;

int newPathCount = -1;
int landSite = 0;
int record = 0;

float _angle = 60.0f;
float landArea = 100;
float threshold = 10;
float maxVariance = 0.05;

#endif // QuadDeclared

