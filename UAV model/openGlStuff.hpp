#include <windows.h>
#include <iostream>
#include <stdlib.h>

#ifdef __APPLE__
#include <OpenGL/OpenGL.h>
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include "terrain.hpp"
#include "quad.hpp"

void cleanup() {
	delete _terrain;
}


void handleKeypress(unsigned char key, int x, int y) {
	switch (key) {
		case 27: //Escape key
			cleanup();
			exit(0);
	}
}

void initRendering() {
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_COLOR_MATERIAL);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_NORMALIZE);
	glShadeModel(GL_SMOOTH);
}

void handleResize(int w, int h) {
	glViewport(0, 0, w, h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45.0, (double)w / (double)h, 1.0, 200.0);
}

void drawScene() {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glTranslatef(0.0f, -2.0f, -20.0f);
	glRotatef(30.0f, 1.0f, 0.0f, 0.0f);
	glRotatef(-_angle, 0.0f, 1.0f, 0.0f);

	GLfloat ambientColor[] = {0.4f, 0.4f, 0.4f, 1.0f};
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, ambientColor);

	GLfloat lightColor0[] = {0.6f, 0.6f, 0.6f, 1.0f};
	GLfloat lightPos0[] = {-0.5f, 0.8f, 0.1f, 0.0f};
	glLightfv(GL_LIGHT0, GL_DIFFUSE, lightColor0);
	glLightfv(GL_LIGHT0, GL_POSITION, lightPos0);

	float scale = 10.0f / max(_terrain->width() - 1, _terrain->length() - 1);
	glScalef(scale, 2*scale, scale);
	glTranslatef(-(float)(_terrain->width() - 1) / 2,
				 0.0f,
				 -(float)(_terrain->length() - 1) / 2);

	glColor3f(0.3f, 0.9f, 0.0f);
	for(int z = 0; z < _terrain->length() - 1; z++) {
		//Makes OpenGL draw a triangle at every three consecutive vertices

		glBegin(GL_TRIANGLE_STRIP);
		for(int x = 0; x < _terrain->width(); x++) {

			Vec3f normal = _terrain->getNormal(x, z);
			glNormal3f(normal[0], normal[1], normal[2]);
			glVertex3f(x, _terrain->getHeight(x, z), z);
			normal = _terrain->getNormal(x, z + 1);
			glNormal3f(normal[0], normal[1], normal[2]);
			glVertex3f(x, _terrain->getHeight(x, z + 1), z + 1);
		}
		glEnd();

        /**  Normals are used just for Visual Representation.
        If You want to avoid normals, then the terrain can be simply shown in form of lines.
        Comment out above written code and uncomment it. You'll understand what I want to say.
        **/

/**
        glBegin(GL_LINES);
        for(int x = 0; x < _terrain->width(); x++) {
          glVertex3d(x, _terrain->getHeight(x, z), z);
            glVertex3d(x, _terrain->getHeight(x, z + 1), z + 1);
		}
		glEnd();

**/

	}

	glColor3f(1.0f, 0.0f, 0.0f);
	for(int i = 0;i<landSite;i++)
    {
        glPointSize(5.0f);
        glColor3f(1.0f, 0.0f, 0.0f);
        //cout<<"\n\nLanding Site : "<<i;
        //cout<<"\nMean Height : "<<goodPoint[i].height;
        //cout<<"\nMean Variance : "<<goodPoint[i].variance;
        //cout<<"\nHeight of each pixel : ";
        for(int x = goodPoint[i].startPointX; x < goodPoint[i].endPointX; x++)
        {
            for(int y = goodPoint[i].startPointY; y < goodPoint[i].endPointY; y++)
            {
                //cout<<_terrain->getHeight(x,y)<<"\t";
                //glTranslatef(x,_terrain->getHeight(x,y), y);
                glBegin(GL_POINTS); //starts drawing of points
                glVertex3f(x,_terrain->getHeight(x,y), y);//upper-right corner
                glVertex3f(x+1,_terrain->getHeight(x+1,y+1), y+1);//lower-left corner
                glEnd();

            }

        }
    }

int temp_val=newPathCount-1;

    glPushMatrix();
    glTranslatef(newPath[temp_val].self_x, newPath[temp_val].self_z+5, newPath[temp_val].self_y);
    glBegin(GL_POLYGON);
        glColor3f(0.0f, 0.0f, 1.0f);
        glutWireSphere(5,40,40);
		glEnd();
    glPopMatrix();
    // To draw path
    /*
    for(temp_val=0;temp_val<newPathCount-1;temp_val++)
    {
        glLineWidth(1.0f);
        glBegin(GL_LINES);
        glColor3f(0.0f, 0.0f, 0.0f);
        glVertex3d(newPath[temp_val].self_x, newPath[temp_val].self_z+5, newPath[temp_val].self_y);
        glVertex3d(newPath[temp_val].node_parent_x, newPath[temp_val].node_parent_z+5 ,  newPath[temp_val].node_parent_y);
        glEnd();



        glPopMatrix();

    }
    */
    for(temp_val=0;temp_val<newPathCount-2;temp_val++)
    {
        glLineWidth(1.0f);
        glBegin(GL_LINES);
        glColor3f(0.0f, 0.0f, 0.0f);
        glVertex3d(newPath[temp_val].self_x, newPath[temp_val].self_z+5, newPath[temp_val].self_y);
        glVertex3d(newPath[temp_val+1].self_x, newPath[temp_val+1].self_z+5, newPath[temp_val+1].self_y);
        //glVertex3d(newPath[temp_val].node_parent_x, newPath[temp_val].node_parent_z+5 ,  newPath[temp_val].node_parent_y);
        glEnd();



        glPopMatrix();

    }
    for(temp_val=0;temp_val<record-2;temp_val++)
    {
        glLineWidth(2.0f);
        glBegin(GL_LINES);
        glColor3f(0.0f, 0.0f, 1.0f);
        glVertex3d(finalPath[temp_val].self_x, finalPath[temp_val].self_z+5, finalPath[temp_val].self_y);
        glVertex3d(finalPath[temp_val+1].self_x, finalPath[temp_val+1].self_z+5, finalPath[temp_val+1].self_y);
        //glVertex3d(finalPath[temp_val].node_parent_x, finalPath[temp_val].node_parent_z+5 ,  finalPath[temp_val].node_parent_y);
        glEnd();



        glPopMatrix();

    }

	glutSwapBuffers();
}

void update(int value) {
	_angle += 2.0f;
	if (_angle > 360) {
		_angle -= 360;
	}

	glutPostRedisplay();
	glutTimerFunc(250, update, 0);
}

