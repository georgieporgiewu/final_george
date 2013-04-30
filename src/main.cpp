// CS184 Simple OpenGL Example
#include <vector>
#include <iostream>
#include <fstream>
#include <cmath>
#include <ctime>

#ifdef _WIN32
#include <windows.h>
#else
#include <sys/time.h>
#endif

#ifdef OSX
#include <GLUT/glut.h>
#include <OpenGL/glu.h>
#else
#include <GL/glut.h>
#include <GL/glu.h>
#endif

#include <time.h>
#include <math.h>

//#include <chrono>
#include <Eigen/Dense>

#ifdef _WIN32
static DWORD lastTime;
#else
static struct timeval lastTime;
#endif

#include "main.h"
#include "Constraints.h"
#include "Particles.h"
#include "Spring.h"

#define PI 3.14159265

using namespace std;
using namespace Eigen;
//using namespace std::chrono;

// trying out timer


// code from paul's

Vector3f gravity(0.0f, -0.98f, 0.0f);

//number of particles grid
const int gridSize = 13;

//Spring properties
int numS;
float springConst = 10.0f;
float restLength = 1.0f;
float damper = 0.8f;
Spring * springs = NULL;

//Particle properties
int numP;
float mass = 0.01f;
Particles * p1=NULL;
Particles * p2=NULL;
Particles * currentP=NULL;
Particles * nextP=NULL;

// What do we want to draw?
bool drawBalls = false, drawSprings = false;
bool drawTriangles = true, drawPatches = false;

// Sphere
float sphereRadius = 4.0f;

// floor texture
GLuint floorTexture;

// how tesselated is each patch?
int patchTesselation = 5;



//****************************************************
// Some Classes
//****************************************************
class Viewport {
  public:
    int w, h; // width and height
};


//****************************************************
// Global Variables
//****************************************************
Viewport    viewport;

//****************************************************
// reshape viewport if the window is resized
//****************************************************
void myReshape(int w, int h) {
  viewport.w = w;
  viewport.h = h;

  glViewport(0,0,viewport.w,viewport.h);// sets the rectangle that will be the window
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();                // loading the identity matrix for the screen

  //----------- setting the projection -------------------------
  // glOrtho sets left, right, bottom, top, zNear, zFar of the chord system


  // glOrtho(-1, 1 + (w-400)/200.0 , -1 -(h-400)/200.0, 1, 1, -1); // resize type = add
  // glOrtho(-w/400.0, w/400.0, -h/400.0, h/400.0, 1, -1); // resize type = center

  glOrtho(-1, 1, -1, 1, 1, -1);    // resize type = stretch

  //------------------------------------------------------------
}


//****************************************************
// sets the window up
//****************************************************
void initScene(){



  //this section from example_00
  
  glClearColor(0.0f, 0.0f, 0.0f, 0.0f); // Clear to black, fully transparent
  
  
  /***
  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);
  glEnable(GL_NORMALIZE);
  glColorMaterial( GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE ) ;
  glEnable(GL_COLOR_MATERIAL);
  GLfloat light_ambient[] = {0.6, 0.2, 0.2, 1.0};
  GLfloat light_diffuse[] = {1.0, 1.0, 1.0, 1.0};
  GLfloat light_specular[] = {1.0, 1.0, 1.0, 1.0};
  GLfloat light_position[] = {-1.0, 1.0, -1.0, 0.0};
  glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
  glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
  glLightfv(GL_LIGHT0, GL_POSITION, light_position);
  ***/

  myReshape(viewport.w,viewport.h);
  
  //end this section


  /***

  //this section from Paul's, edited though
  
  glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
  
  glViewport(0, 0, 640, 480);
  
  //set up projection matrix
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(45.0f, 640/480, 1.0f, 100.0f);
  
  // load identity modelview
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  
  //shading states
  glShadeModel(GL_SMOOTH);
  glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
  glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
  glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
  
  //depth states
  glClearDepth(1.0f);
  glDepthFunc(GL_LEQUAL);
  glEnable(GL_DEPTH_TEST);
  
  //set up light
  GLfloat l1 [] = {1.0f, 1.0f, 1.0f, 0.0f};
  GLfloat l2 [] = {1.0f, 1.0f, 1.0f, 1.0f};
  GLfloat l3 [] = {0.2f, 0.2f, 0.2f, 0.2f};
  GLfloat l4 [] = {1.0f, 1.0f, 1.0f, 0.0f};
  glLightfv(GL_LIGHT1, GL_POSITION, l1);
  glLightfv(GL_LIGHT1, GL_DIFFUSE, l2);
  glLightfv(GL_LIGHT1, GL_AMBIENT, l3);
  glLightfv(GL_LIGHT1, GL_SPECULAR, l4);
  glEnable(GL_LIGHT1);
  
  //Use 2-sided lighting
  glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, true);
  
  //end Paul's section

  //glOrtho(-4, 4, -4, 4, 1, -1); 
  
  ***/
  
}

//****************************************************
// from Paul's, set up variables
//****************************************************
bool DemoInit()
{
	//Calculate number of balls
	numP=gridSize*gridSize;
		
	//Calculate number of springs
	//There is a spring pointing right for each ball which is not on the right edge, 
	//and one pointing down for each ball not on the bottom edge
	numS=(gridSize-1)*gridSize*2;

	//There is a spring pointing down & right for each ball not on bottom or right,
	//and one pointing down & left for each ball not on bottom or left
	numS+=(gridSize-1)*(gridSize-1)*2;

	//There is a spring pointing right (to the next but one ball)
	//for each ball which is not on or next to the right edge, 
	//and one pointing down for each ball not on or next to the bottom edge
	numS+=(gridSize-2)*gridSize*2;

	//Create space for p & springs
	p1=new Particles[numP];
	p2=new Particles[numP];
	springs=new Spring[numS];

	//Reset cloth
	ResetCloth();

	/***
	//Load floor texture
	IMAGE floorImage;
	if(!floorImage.Load("floor.tga"))
		return false;
	if(floorImage.paletted)
		floorImage.ExpandPalette();

	glGenTextures(1, &floorTexture);
	glBindTexture(GL_TEXTURE_2D, floorTexture);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	
	gluBuild2DMipmaps(	GL_TEXTURE_2D, GL_RGBA8, floorImage.width, floorImage.height,
						floorImage.format, GL_UNSIGNED_BYTE, floorImage.data);
	
	***/

	return true;
}

//***************************************************
// function from Paul's that resets cloth
//***************************************************

void ResetCloth()
{
	//Initialise the balls in an evenly spaced grid in the x-z plane
	for(int i=0; i<gridSize; ++i)
	{
		for(int j=0; j<gridSize; ++j)
		{
			p1[i*gridSize+j].position << float(j)-float(gridSize-1)/2,
												 8.5f,
												 float(i)-float(gridSize-1)/2;
			p1[i*gridSize+j].velocity << 0,0,0;
			p1[i*gridSize+j].mass=mass;
			p1[i*gridSize+j].fixed=false;
		}
	}

	//Fix the top left & top right balls in place
	p1[0].fixed=true;
	p1[gridSize-1].fixed=true;
	
	//Fix the bottom left & bottom right balls
	p1[gridSize*(gridSize-1)].fixed=true;
	p1[gridSize*gridSize-1].fixed=true;

	//Copy the balls into the other array
	for(int i=0; i<numP; ++i)
		p2[i]=p1[i];

	//Set the currentP and nextP pointers
	currentP=p1;
	nextP=p2;


	//Initialise the springs
	Spring * currentSpring=&springs[0];

	//The first (gridSize-1)*gridSize springs go from one ball to the next,
	//excluding those on the right hand edge
	for(int i=0; i<gridSize; ++i)
	{
		for(int j=0; j<gridSize-1; ++j)
		{
			currentSpring->ball1=i*gridSize+j;
			currentSpring->ball2=i*gridSize+j+1;

			currentSpring->springConst=springConst;
			currentSpring->restLength=restLength;
			
			++currentSpring;
		}
	}

	//The next (gridSize-1)*gridSize springs go from one ball to the one below,
	//excluding those on the bottom edge
	for(int i=0; i<gridSize-1; ++i)
	{
		for(int j=0; j<gridSize; ++j)
		{
			currentSpring->ball1=i*gridSize+j;
			currentSpring->ball2=(i+1)*gridSize+j;

			currentSpring->springConst=springConst;
			currentSpring->restLength=restLength;
			
			++currentSpring;
		}
	}

	//The next (gridSize-1)*(gridSize-1) go from a ball to the one below and right
	//excluding those on the bottom or right
	for(int i=0; i<gridSize-1; ++i)
	{
		for(int j=0; j<gridSize-1; ++j)
		{
			currentSpring->ball1=i*gridSize+j;
			currentSpring->ball2=(i+1)*gridSize+j+1;

			currentSpring->springConst=springConst;
			currentSpring->restLength=restLength*sqrt(2.0f);
			
			++currentSpring;
		}
	}

	//The next (gridSize-1)*(gridSize-1) go from a ball to the one below and left
	//excluding those on the bottom or right
	for(int i=0; i<gridSize-1; ++i)
	{
		for(int j=1; j<gridSize; ++j)
		{
			currentSpring->ball1=i*gridSize+j;
			currentSpring->ball2=(i+1)*gridSize+j-1;

			currentSpring->springConst=springConst;
			currentSpring->restLength=restLength*sqrt(2.0f);
			
			++currentSpring;
		}
	}

	//The first (gridSize-2)*gridSize springs go from one ball to the next but one,
	//excluding those on or next to the right hand edge
	for(int i=0; i<gridSize; ++i)
	{
		for(int j=0; j<gridSize-2; ++j)
		{
			currentSpring->ball1=i*gridSize+j;
			currentSpring->ball2=i*gridSize+j+2;

			currentSpring->springConst=springConst;
			currentSpring->restLength=restLength*2;
			
			++currentSpring;
		}
	}

	//The next (gridSize-2)*gridSize springs go from one ball to the next but one below,
	//excluding those on or next to the bottom edge
	for(int i=0; i<gridSize-2; ++i)
	{
		for(int j=0; j<gridSize; ++j)
		{
			currentSpring->ball1=i*gridSize+j;
			currentSpring->ball2=(i+2)*gridSize+j;

			currentSpring->springConst=springConst;
			currentSpring->restLength=restLength*2;
			
			++currentSpring;
		}
	}
}

//perform per frame updates
void UpdateFrame() {
	currentP[0].fixed = false;
	currentP[gridSize-1].fixed = false;
	currentP[gridSize*(gridSize-1)].fixed = false;
	currentP[gridSize*gridSize-1].fixed = false;
	
	usleep(1000);
	
	float timePassedInSeconds = 0.01f;

	//Calculate the tensions in the springs
	for(int i=0; i<numS; ++i)
	{
		float springLength=(	currentP[springs[i].ball1].position-
								currentP[springs[i].ball2].position).norm();

		float extension=springLength-springs[i].restLength;

		springs[i].tension=springs[i].springConst*extension/springs[i].restLength;
	}

	//Calculate the nextP from the currentP
	for(int i=0; i<numP; ++i)
	{
		//Transfer properties which do not change
		nextP[i].fixed=currentP[i].fixed;
		nextP[i].mass=currentP[i].mass;

		//If the ball is fixed, transfer the position and zero the velocity, otherwise calculate
		//the new values
		if(currentP[i].fixed)
		{
			nextP[i].position=currentP[i].position;
			nextP[i].velocity<<(0,0,0);
		}
		else
		{
			//Calculate the force on this ball
			Vector3f force=gravity;

			//Loop through springs
			for(int j=0; j<numS; ++j)
			{
				//If this ball is "ball1" for this spring, add the tension to the force
				if(springs[j].ball1==i)
				{
					Vector3f tensionDirection=	currentP[springs[j].ball2].position-
												currentP[i].position;
					tensionDirection.normalize();

					force+=springs[j].tension*tensionDirection;
				}

				//Similarly if the ball is "ball2"
				if(springs[j].ball2==i)
				{
					Vector3f tensionDirection=	currentP[springs[j].ball1].position-
												currentP[i].position;
					tensionDirection.normalize();

					force+=springs[j].tension*tensionDirection;
				}
			}

			//Calculate the acceleration
			Vector3f acceleration=force/currentP[i].mass;
		
			//Update velocity
			nextP[i].velocity=currentP[i].velocity+acceleration*
																(float)timePassedInSeconds;

			//Damp the velocity
			nextP[i].velocity*=damper;
		
			//Calculate new position
			nextP[i].position=currentP[i].position+
				(nextP[i].velocity+currentP[i].velocity)*(float)timePassedInSeconds/2;

			//Check against sphere (at origin)
			if((nextP[i].position.norm()*nextP[i].position.norm())<sphereRadius*1.08f*sphereRadius*1.08f)
				nextP[i].position=nextP[i].position.normalized()*
																		sphereRadius*1.08f;

			//Check against floor
			if(nextP[i].position(1) < (-8.5f)) {
				nextP[i].position(1)=-8.5f;
			}
		}
	}

	//Swap the currentP and newBalls pointers
	Particles * temp=currentP;
	currentP=nextP;
	nextP=currentP;
	
	//calculate normals, because positions are always updated :]
	
	
	//Zero the normals on each ball
	for(int i=0; i<numP; ++i) {
		Vector3f zeroVect(0,0,0);
		currentP[i].normal = zeroVect;
	}

	//Calculate the normals on the current balls
	for(int i=0; i<gridSize-1; ++i)
	{
		for(int j=0; j<gridSize-1; ++j)
		{
			Vector3f & q0=currentP[i*gridSize+j].position;
			Vector3f & q1=currentP[i*gridSize+j+1].position;
			Vector3f & q2=currentP[(i+1)*gridSize+j].position;
			Vector3f & q3=currentP[(i+1)*gridSize+j+1].position;

			Vector3f & n0=currentP[i*gridSize+j].normal;
			Vector3f & n1=currentP[i*gridSize+j+1].normal;
			Vector3f & n2=currentP[(i+1)*gridSize+j].normal;
			Vector3f & n3=currentP[(i+1)*gridSize+j+1].normal;

			//Calculate the normals for the 2 triangles and add on
			Vector3f normal=(q1-q0).cross(q2-q0);

			n0+=normal;
			n1+=normal;
			n2+=normal;

			normal=(q1-q2).cross(q3-q2);

			n1+=normal;
			n2+=normal;
			n3+=normal;
		}
	}

	//Normalize normals
	for(int i=0; i<numP; ++i)
		currentP[i].normal.normalize();
	
	//Render frame
	RenderFrame();

}

//render a frame
void RenderFrame()
{

	//Clear buffers
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	//glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();										//reset modelview matrix
	
	//glutWireTeapot(1.0);
	
	glColor3f (1.0, 1.0, 1.0);
	//glTranslatef(0.0f, 0.0f, -28.0f);
	glutSolidSphere(0.2, 48, 24);
	
	/*** 
	//triangle works....
	glBegin(GL_POLYGON);
	glVertex3f(-0.5f, -0.5f, 0.0f);
	glVertex3f(0.5f, -0.5f, 0.0f);
	glVertex3f(0.0f, 0.5f, 0.0f);
	glEnd();
	***/
	
	/***
	glColor4f(1.0f, 1.0f, 1.0f, 1.0f);

	glTranslatef(0.0f, 0.0f, -28.0f);

	//Draw sphere (at origin)
	static GLUquadricObj * sphere=gluNewQuadric();
	glEnable(GL_LIGHTING);
	GLfloat mat1 [] = {1.0f, 0.0f, 0.0f, 0.0f};
	GLfloat mat2 [] = {1.0f, 0.0f, 0.0f, 0.0f};
	GLfloat mat3 [] = {1.0f, 1.0f, 1.0f, 1.0f};
	glMaterialfv(GL_FRONT, GL_AMBIENT, mat1);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, mat2);
	glMaterialfv(GL_FRONT, GL_SPECULAR, mat3);
	glMaterialf(GL_FRONT, GL_SHININESS, 32.0f);
	glEnable(GL_CULL_FACE);

	gluSphere(sphere, sphereRadius, 48, 24);
			
	glDisable(GL_LIGHTING);
	glDisable(GL_CULL_FACE);
	GLfloat mat4 [] = {0.0f, 0.0f, 0.0f, 0.0f};
	glMaterialfv(GL_FRONT, GL_SPECULAR, mat4);
	
	***/
	
	
	
	//Draw cloth as triangles
	if(drawTriangles)
	{
		/***
		glEnable(GL_LIGHTING);
		GLfloat tri_mat1 [] = {0.8f, 0.0f, 1.0f};
		GLfloat tri_mat2 [] = {0.8f, 0.0f, 1.0f};
		GLfloat tri_mat3 [] = {1.0f, 1.0f, 0.0f};
		GLfloat tri_mat4 [] = {1.0f, 1.0f, 0.0f};
		glMaterialfv(GL_FRONT, GL_AMBIENT, tri_mat1);	//set material
		glMaterialfv(GL_FRONT, GL_DIFFUSE, tri_mat2);
		glMaterialfv(GL_BACK, GL_AMBIENT, tri_mat3);
		glMaterialfv(GL_BACK, GL_DIFFUSE, tri_mat4);
		***/
		
		//glBegin(GL_TRIANGLES);
		{
			for(int i=0; i<gridSize-1; ++i)
			{
				for(int j=0; j<gridSize-1; ++j)
				{
					//cout << "drawing triangles!" << endl;
					//cout << currentP[i*gridSize+j].position << endl << endl;
					glBegin(GL_POLYGON);
					/***
					glNormal3f(currentP[i*gridSize+j].normal(0),
						currentP[i*gridSize+j].normal(1),
						currentP[i*gridSize+j].normal(2));
					***/
					glVertex3f(currentP[i*gridSize+j].position(0),
						currentP[i*gridSize+j].position(1),
						currentP[i*gridSize+j].position(2));
					/***
					glNormal3f(currentP[i*gridSize+j+1].normal(0),
						currentP[i*gridSize+j+1].normal(1),
						currentP[i*gridSize+j+1].normal(2));
					***/
					glVertex3f(currentP[i*gridSize+j+1].position(0),
						currentP[i*gridSize+j+1].position(1),
						currentP[i*gridSize+j+1].position(2));
					/***	
					glNormal3f(currentP[(i+1)*gridSize+j].normal(0),
						currentP[(i+1)*gridSize+j].normal(1),
						currentP[(i+1)*gridSize+j].normal(2));
					***/	
					glVertex3f(currentP[(i+1)*gridSize+j].position(0),
						currentP[(i+1)*gridSize+j].position(1),
						currentP[(i+1)*gridSize+j].position(2));
						
					glEnd();
					
					glBegin(GL_POLYGON);
					/***
					glNormal3f(currentP[(i+1)*gridSize+j].normal(0),
						currentP[(i+1)*gridSize+j].normal(1),
						currentP[(i+1)*gridSize+j].normal(2));
					***/	
					glVertex3f(currentP[(i+1)*gridSize+j].position(0),
						currentP[(i+1)*gridSize+j].position(1),
						currentP[(i+1)*gridSize+j].position(2));
					/***	
					glNormal3f(currentP[i*gridSize+j+1].normal(0),
						currentP[i*gridSize+j+1].normal(1),
						currentP[i*gridSize+j+1].normal(2));
					***/	
					glVertex3f(currentP[i*gridSize+j+1].position(0),
						currentP[i*gridSize+j+1].position(1),
						currentP[i*gridSize+j+1].position(2));
					/***	
					glNormal3f(currentP[(i+1)*gridSize+j+1].normal(0),
						currentP[(i+1)*gridSize+j+1].normal(1),
						currentP[(i+1)*gridSize+j+1].normal(2));
					***/	
					glVertex3f(currentP[(i+1)*gridSize+j+1].position(0),
						currentP[(i+1)*gridSize+j+1].position(1),
						currentP[(i+1)*gridSize+j+1].position(2));
						
					glEnd();
				}
			}
		}
		//glEnd();
		//glDisable(GL_LIGHTING);
	}
	
	
	glFlush();
	glutSwapBuffers();
	
}



//shut down demo
void DemoShutdown() {
	if (p1)
		delete [] p1;
	p1 = NULL;
	
	if (p2)
		delete [] p2;
	p2 = NULL;
	
	if (springs)
		delete [] springs;
	springs = NULL;


}

//***************************************************
// function that does the actual drawing
//***************************************************
void myDisplay() {


  //----------------------- ----------------------- -----------------------
  // This is a quick hack to add a little bit of animation.
  static float tip = 0.5f;
  const  float stp = 0.0001f;
  const  float beg = 0.1f;
  const  float end = 0.9f;

  tip += stp;
  if (tip>end) tip = beg;
  //----------------------- ----------------------- -----------------------


  glClear(GL_COLOR_BUFFER_BIT);                // clear the color buffer (sets everything to black)

  glMatrixMode(GL_MODELVIEW);                  // indicate we are specifying camera transformations
  glLoadIdentity();                            // make sure transformation is "zero'd"

  //----------------------- code to draw objects --------------------------
  // Rectangle Code
  //glColor3f(red component, green component, blue component);
  glColor3f(1.0f,0.0f,0.0f);                   // setting the color to pure red 90% for the rect

  glBegin(GL_POLYGON);                         // draw rectangle 
  //glVertex3f(x val, y val, z val (won't change the point because of the projection type));
  glVertex3f(-0.8f, 0.0f, 0.0f);               // bottom left corner of rectangle
  glVertex3f(-0.8f, 0.5f, 0.0f);               // top left corner of rectangle
  glVertex3f( 0.0f, 0.5f, 0.0f);               // top right corner of rectangle
  glVertex3f( 0.0f, 0.0f, 0.0f);               // bottom right corner of rectangle
  glEnd();
  
  glColor3f(0.5f,0.0f,1.0f); 
  
  glBegin(GL_POLYGON);                         // draw left eye
  glVertex3f(-0.60f, -0.3f, 0.0f);               // bottom left corner of rectangle
  glVertex3f(-0.60f, -0.1f, 0.0f);               // top left corner of rectangle
  glVertex3f(-0.55f, -0.1f, 0.0f);               // top right corner of rectangle
  glVertex3f(-0.55f, -0.3f, 0.0f);               // bottom right corner of rectangle
  glEnd();
  
  glBegin(GL_POLYGON);                         // draw left eye
  glVertex3f(-0.45f, -0.3f, 0.0f);               // bottom left corner of rectangle
  glVertex3f(-0.45f, -0.1f, 0.0f);               // top left corner of rectangle
  glVertex3f(-0.40f, -0.1f, 0.0f);               // top right corner of rectangle
  glVertex3f(-0.40f, -0.3f, 0.0f);               // bottom right corner of rectangle
  glEnd();
  
  glBegin(GL_POLYGON); 						
  glVertex3f(-0.8f, -0.38f, 0.0f);
  glVertex3f(-0.2f, -0.38f, 0.0f);
  glVertex3f(-0.38f, -.53f, 0.0f);
  glVertex3f(-0.5f, -0.60f, 0.0f);
  glVertex3f(-0.62f, -.53f, 0.0f);
  glEnd();
  
  glColor3f(.36f,.25f,.20f); 
  
  glBegin(GL_POLYGON);						//draw cross
  glVertex3f(.25f, .78f, 0.0f);
  glVertex3f(.75f, .78f, 0.0f);
  glVertex3f(.75f, .72f, 0.0f);
  glVertex3f(.25f, .72f, 0.0f);
  glEnd();
  
  glBegin(GL_POLYGON);
  glVertex3f(.47f, .95f, 0.0f);
  glVertex3f(.53f, .95f, 0.0f);
  glVertex3f(.53f, .10f, 0.0f);
  glVertex3f(.47f, .10f, 0.0f);
  glEnd();
  
  
  // Triangle Code
  glColor3f(1.0f,0.5f,0.0f);                   // setting the color to orange for the triangle

  float basey = -sqrt(0.48f);                  // height of triangle = sqrt(.8^2-.4^2)
  glBegin(GL_POLYGON);
  glVertex3f(tip,  0.0f, 0.0f);                // top tip of triangle
  glVertex3f(0.1f, basey, 0.0f);               // lower left corner of triangle
  glVertex3f(0.9f, basey, 0.0f);               // lower right corner of triangle
  glEnd();
  //-----------------------------------------------------------------------

  glFlush();
  glutSwapBuffers();                           // swap buffers (we earlier set double buffer)
  
  
}


//****************************************************
// called by glut when there are no messages to handle
//****************************************************
void myFrameMove() {
  //nothing here for now
#ifdef _WIN32
  Sleep(10);                                   //give ~10ms back to OS (so as not to waste the CPU)
#endif
  glutPostRedisplay(); // forces glut to call the display function (myDisplay())
}


//****************************************************
// the usual stuff, nothing exciting here
//****************************************************
int main(int argc, char *argv[]) {

  /***
  //testing timer things
  //double lastTime=time(&timer);
  time_t lastTime = time(0);
  usleep(1000);
  //double currentTime = time(&timer);
  time_t currentTime = time(0);
  float timePassed = difftime(lastTime, currentTime);
  cout << "timePassed: " << timePassed << endl;
  ***/
  
  //testing using chrono
  /***
  clock_t uptime = clock() / (CLOCKS_PER_SEC / 1000000);
  usleep(1000);
  clock_t uptime2 = clock() / (CLOCKS_PER_SEC / 1000000);
  cout << "diff: " << uptime2 - uptime << endl;
    ***/

  //This initializes glut
  glutInit(&argc, argv);

  //This tells glut to use a double-buffered window with red, green, and blue channels 
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);

  // Initalize theviewport size
  viewport.w = 640;
  viewport.h = 480;

  //The size and position of the window
  glutInitWindowSize(viewport.w, viewport.h);
  glutInitWindowPosition(0, 0);
  glutCreateWindow("CS184!");

  initScene();                                 // quick function to set up scene
  

  DemoInit();

  glutDisplayFunc(UpdateFrame);                  // function to run when its time to draw something
  glutReshapeFunc(myReshape);                  // function to run when the window gets resized
  glutIdleFunc(myFrameMove);                   // function to run when not handling any other task
  glutMainLoop();                              // infinite loop that will keep drawing and resizing and whatever else
  
  DemoShutdown();

  return 0;
}







