#ifndef FUNCTIONS

#define FUNCTIONS

#include "terrain.hpp"
#include "quad.hpp"

void childInitialize(Node *xy)
{
    //Initialises child-nodes to NULL - better safe than sorry
	for (int i = 0; i < 4; i++)
		xy->child[i] = NULL;
}

void setnode(Node *xy, int w, int h)
{
	xy->bottomLeft_X = 5;
	xy->bottomLeft_Y = 5;
	xy->topRight_X = w-2;
	xy->topRight_Y =  h-2;
	xy->height =0;
	xy->variance = 0;

	childInitialize(xy);
	xy->size = w*h;

}

void heightCal(Node* parent)
{
    int width = parent->topRight_X - parent->bottomLeft_X + 1;
    int height = parent->topRight_Y - parent->bottomLeft_Y + 1;
    parent->height = 0;
    for(int i = parent->bottomLeft_Y; i<=parent->topRight_Y ;i++)
        {
            for(int j = parent->bottomLeft_X; j<=parent->topRight_X ;j++)
            {
                parent->height += _terrain->getHeight(j,i);
                //cout<<"\nHeight ("<< j<<","<<i<<") : "<<_terrain->getHeight(j,i);
            }
        }

        parent->height = parent->height/(width*height);
        //cout<<"\nHeight "<<parent->height;
}

void varCal(Node* parent)
{
    int width = parent->topRight_X - parent->bottomLeft_X + 1;
    int height = parent->topRight_Y - parent->bottomLeft_Y + 1;

    for(int i = parent->bottomLeft_Y; i<=parent->topRight_Y ;i++)
    {
        for(int j = parent->bottomLeft_X; j<=parent->topRight_X ;j++)
        {
                parent->variance += (parent->height - _terrain->getHeight(j,i))*(parent->height - _terrain->getHeight(j,i));
                //cout<<"\nTerrain Height : "<<_terrain->getHeight(j,i);
        }

    }
        //cout<<"\nMean Height : "<<parent->height;
    parent->variance = parent->variance/(width*height);
}

void buildQuadTree(Node *parent)
{
    int width = parent->topRight_X - parent->bottomLeft_X + 1;
    int height = parent->topRight_Y - parent->bottomLeft_Y + 1;

    int center_X = (int)(parent->bottomLeft_X + (width/2));
    int center_Y = (int)(parent->bottomLeft_Y + (height/2));

    if((height*width)>=landArea)
    {
    //Child 0
     parent->child[0] = new Node;
	 parent->child[0]->bottomLeft_X = center_X;
	 parent->child[0]->bottomLeft_Y = center_Y;
	 parent->child[0]->topRight_X = parent->topRight_X;
	 parent->child[0]->topRight_Y = parent->topRight_Y;
	 parent->child[0]->size = (parent->child[0]->topRight_X - parent->child[0]->bottomLeft_X)*(parent->child[0]->topRight_Y - parent->child[0]->bottomLeft_Y);
	 buildQuadTree(parent->child[0]);

	 parent->child[1] = new Node;
	 parent->child[1]->bottomLeft_X = parent->bottomLeft_X;
	 parent->child[1]->bottomLeft_Y = center_Y;
	 parent->child[1]->topRight_X = center_X;
	 parent->child[1]->topRight_Y = parent->topRight_Y;
	 parent->child[1]->size = (parent->child[1]->topRight_X - parent->child[1]->bottomLeft_X)*(parent->child[1]->topRight_Y - parent->child[1]->bottomLeft_Y);
	 buildQuadTree(parent->child[1]);

     parent->child[2] = new Node;
	 parent->child[2]->bottomLeft_X = parent->bottomLeft_X;
	 parent->child[2]->bottomLeft_Y = parent->bottomLeft_Y;
	 parent->child[2]->topRight_X = center_X;
	 parent->child[2]->topRight_Y = center_Y;
	 parent->child[2]->size = (parent->child[2]->topRight_X - parent->child[2]->bottomLeft_X)*(parent->child[2]->topRight_Y - parent->child[2]->bottomLeft_Y);
	 buildQuadTree(parent->child[2]);

     parent->child[3] = new Node;
	 parent->child[3]->bottomLeft_X = center_X;
	 parent->child[3]->bottomLeft_Y = parent->bottomLeft_Y;
	 parent->child[3]->topRight_X = parent->topRight_X;
	 parent->child[3]->topRight_Y = center_Y;
	 parent->child[3]->size = (parent->child[3]->topRight_X - parent->child[3]->bottomLeft_X)*(parent->child[3]->topRight_Y - parent->child[3]->bottomLeft_Y);
	 buildQuadTree(parent->child[3]);

    heightCal(parent);
    varCal(parent);
 }
    else
    {
        heightCal(parent);
        varCal(parent);
    }

}

void showAns(Node* parent, int i)
{
        cout<<"\nChild "<<i <<" : ";
        cout<<"\nBottom X : "<<parent->child[i]->bottomLeft_X;
        cout<<"\nBottom Y : "<<parent->child[i]->bottomLeft_Y;
        cout<<"\nTop X : "<<parent->child[i]->topRight_X;
        cout<<"\nTop Y : "<<parent->child[i]->topRight_Y;
        cout<<"\nheight : "<<parent->child[i]->height;
        cout<<"\t size : "<<parent->child[i]->size;
        cout<<"\tVariance : "<<parent->child[i]->variance;
        cout<<"\n";
}

void childCombination(Node *parent)
{
    //Check If parent's variance is less than all of it's children
    if((parent->child[0]->potential_land == 2) or (parent->child[1]->potential_land == 2)
       or (parent->child[2]->potential_land == 2) or (parent->child[3]->potential_land == 2))
    {
        int flag = 0;
        if(parent->child[0]->potential_land == 2 and flag == 0)
        {
            if(parent->variance > parent->child[0]->variance)
                flag = 1;
        }
        if(parent->child[1]->potential_land == 2 and flag == 0)
        {
            if(parent->variance > parent->child[1]->variance)
                flag = 1;
        }
        if(parent->child[2]->potential_land == 2 and flag == 0)
        {
            if(parent->variance > parent->child[2]->variance)
                flag = 1;
        }
        if(parent->child[3]->potential_land == 2 and flag == 0)
        {
            if(parent->variance > parent->child[3]->variance)
                flag = 1;
        }

        if(flag == 1)
            parent->potential_land = 1;
        else
        {
            parent->potential_land = 2;
            for(int i =0;i<4;i++)
                parent->child[i] = NULL;

        }

    }
    //When the size of landing site is less than potential landing site, try combination of children
    else if((parent->child[0]->potential_land == 0) or (parent->child[1]->potential_land == 0)
       or (parent->child[2]->potential_land == 0) or (parent->child[3]->potential_land == 0))
    {
        /***
        Combinations possible :
        | 0,1 | , | 1,2 | ,
        | 2,3 | , | 3,0 |
        ***/
        float tempHeight = 0;
        float tempVar = 0;

        int a =0;
        int b = 1;
        Node *tempData[4] = {NULL, NULL, NULL, NULL};

        for(int i =0;i<4;i++)
        {
            if (i == 1)
            {
                a = 1;
                b = 2;
            }
            if (i == 2)
            {
                a = 2;
                b = 3;
            }
            if (i == 3)
            {
                a = 3;
                b = 0;
            }

            tempData[i] = parent->child[a];
            if(i ==0 or i ==1)
                {
                tempData[i]->bottomLeft_X = parent->child[b]->bottomLeft_X;
                tempData[i]->bottomLeft_Y = parent->child[b]->bottomLeft_Y;
                }
            else
                {
                 tempData[i]->topRight_X = parent->child[b]->topRight_X;
                 tempData[i]->topRight_Y = parent->child[b]->topRight_Y;
                }

            heightCal(tempData[i]);
            varCal(tempData[i]);
            tempData[i]->size = parent->child[a]->size + parent->child[b]->size;

            if(tempData[i]->variance > maxVariance)
            {
             tempData[i] = NULL;
            }

        }
        for(int i =0;i<4;i++)
        {
            if(tempData[i] != NULL and tempData[i]->size >= (landArea-threshold))
            {
                parent->child[i] = tempData[i];
                parent->child[i]->potential_land = 2;
            }
            else
                parent->child[i]->potential_land = 1;
        }
       childCombination(parent);
    }
    //Children have sufficient size and are not potential landing site, then check if parent satisfy max variance criteria
    else
    {
    if((parent->child[0]->variance <= 1) or (parent->child[1]->variance <= 1)
       or (parent->child[2]->variance <= 1) or (parent->child[3]->variance <= 1))
       {

           Node *tempNode = new Node;
           tempNode->bottomLeft_X = parent->child[1]->bottomLeft_X + (parent->child[1]->topRight_X - parent->child[1]->bottomLeft_X)/2;
           tempNode->bottomLeft_Y = parent->child[1]->bottomLeft_Y + (parent->child[1]->topRight_Y - parent->child[1]->bottomLeft_Y)/2;
           tempNode->topRight_X = parent->child[1]->topRight_X + (parent->child[1]->topRight_X - parent->child[1]->bottomLeft_X)/2;
           tempNode->topRight_Y = parent->child[1]->topRight_Y + (parent->child[1]->topRight_Y - parent->child[1]->bottomLeft_Y)/2;

            if(tempNode->topRight_X > _terrain->width() or tempNode->topRight_Y > _terrain->length())
                return;
           heightCal(tempNode);
           varCal(tempNode);

           if(tempNode->variance <= maxVariance)
           {
                tempNode->potential_land = 2;
                tempNode->size = (tempNode->topRight_X - tempNode->bottomLeft_X+1) * (tempNode->topRight_Y - tempNode->bottomLeft_Y+1);
                for(int i =0;i<4;i++)
                    tempNode->child[i] = parent->child[0]->child[i];
                parent->child[0] = tempNode;
                childCombination(parent);
           }
           else if(parent->variance <= maxVariance)
            {
             parent->potential_land = 2;
             for(int i =0;i<4;i++)
                parent->child[i] = NULL;
            }
            else
            parent->potential_land = 1;
        }

        else if(parent->variance <= maxVariance)
         {
             parent->potential_land = 2;
             for(int i =0;i<4;i++)
                parent->child[i] = NULL;
         }
         else
            parent->potential_land = 1;
    }
    return;
}

void landingSite(Node *parent)
{

    if(parent->child[0] != NULL)
        landingSite(parent->child[0]);

    if(parent->child[1] != NULL)
        landingSite(parent->child[1]);

    if(parent->child[2] != NULL)
        landingSite(parent->child[2]);

    if(parent->child[3] != NULL)
        landingSite(parent->child[3]);

    if(parent->size <(landArea-threshold))
        return;

    else if(parent->size >=(landArea-(threshold)) and parent->size <(landArea+threshold))
    {
        if(parent->variance < maxVariance)
        {
          parent->potential_land = 2;
          for(int i = 0;i<4;i++)
                parent->child[i] = NULL;
        }
        else
            parent->potential_land = 1;
    }

    else if(parent->size >=(landArea+threshold))
    {
        childCombination(parent);
    }
    return;

}

void printSite(Node *parent)
{
    if(parent->child[0] != NULL)
        printSite(parent->child[0]);

    if(parent->child[1] != NULL)
        printSite(parent->child[1]);

    if(parent->child[2] != NULL)
        printSite(parent->child[2]);

    if(parent->child[3] != NULL)
        printSite(parent->child[3]);

   for(int i =0;i<4;i++)
   {
    if(parent->child[i] != NULL)
    {
        if(parent->child[i]->potential_land ==2 )
        {
            tempLand.startPointX = parent->child[i]->bottomLeft_X;
            tempLand.startPointY = parent->child[i]->bottomLeft_Y;
            tempLand.endPointX = parent->child[i]->topRight_X;
            tempLand.endPointY = parent->child[i]->topRight_Y;
            tempLand.avgHeight = parent->child[i]->height;

            goodPoint.push_back(tempLand);
            landSite++;
            //cout<<"\nLandSite : "<<landSite;
            showAns(parent ,i);

        }
    }
    else;
        //cout<<"\nNULL";
   }
    return;
}


#endif // FUNCTIONS
