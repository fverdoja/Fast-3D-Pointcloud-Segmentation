#include "stdafx.h"
#include "utilities.h"



GLuint window;

SegmFrame* frame;
//InputFrame* frame;

float Calib[11] = {580.8857, 583.317, 319.5, 239.5, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 8000.0}; // Kinect data

GLfloat intrinsics[16];

float Znear = /*0.5*/0.1;
float Zfar = 10.0;

//Switch
bool DISPLAY_FRAME = 1;
bool DISPLAY_ELMTS = 0;
bool DISPLAY_SURFS = 0;
bool DISPLAY_CONTROLS = 0;
bool SPEED_M = 1;
bool DISPLAY_BB = 0;
bool DISPLAY_BLOBS = 0;
bool DISPLAY_REC = 0;
bool DISPLAY_REC_PLAN = 0;
int switchh;
unsigned int NUM_VERTEX = 8;


// Surface :
//Surface *Surf;


// angle of rotation for the camera direction
float anglex = 0.0f;
float angley = 0.0f;
 
// actual vector representing the camera's direction
float lx=0.0f, ly=0.0f, lz=-1.0f;
float lxStrap=-1.0f,lyStrap=0.0f,lzStrap=0.0f;
 
// XZ position of the camera
float x=0.0f, y=1.0f, z=5.0f;
 
// the key states. These variables will be zero
//when no key is being presses
float deltaAnglex = 0.0f;
float deltaAngley = 0.0f;
float deltaMove = 0;
float deltaStrap = 0;
float deltaStrap2 = 0;
float speedVal = 0.02;
int xOrigin = -1;
int yOrigin = -1;


//double angleX = -2;//0;
//double angleY = -101;//-92;
//double angleZ = -4;//-18;
//double dist = -1.2;//4;
//double center = 1.6;// 3.6;




void Init(void){

	GLenum code;

	cudaError_t cudaStatus;

	/*Set up the device for OpenGL to work with */
	cudaStatus = cudaGLSetGLDevice(gpuGetMaxGflopsDeviceId());
	if (cudaStatus != cudaSuccess){
				cout << "cudaSetDevice failed ! Do you have a CUDA-capable GPU installed ?" << endl;
				//return 1;
				}
				

	/* Initialisation de GLEW */
	code = glewInit();

	if(code != GLEW_OK){
				cout << "impossible d'initialiser GLEW :" << glewGetErrorString(code) << endl;
				}



	glViewport(0, 0, WIDTH, HEIGHT) ;
	glMatrixMode(GL_PROJECTION);						// Select The Projection Matrix
	glLoadIdentity();	

	glMatrixMode (GL_MODELVIEW);
	glLoadIdentity ();

	glShadeModel(GL_SMOOTH);							// Enable Smooth Shading
	glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
	//glClearColor(0.0f, 0.0f, 0.0f, 0.5f);				// Black Background
	glClearColor(1.0f, 1.0f, 1.0f, 0.5f);				// White Background
	glClearDepth(1.0f);									// Depth Buffer Setup

	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glEnable(GL_DEPTH_TEST);							// Enables Depth Testing
	glDepthFunc(GL_LEQUAL);								// The Type Of Depth Testing To Do
	glEnable(GL_CULL_FACE);
	glCullFace(GL_BACK);// suprime l'arriere des faces

	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);	// Really Nice Perspective Calculations
	// enable color tracking
	glEnable(GL_COLOR_MATERIAL);
	// set material properties which will be assigned by glColor
	glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);

	glDisable ( GL_LIGHTING ) ;

	Calib[0] = 525.0; Calib[1] = 525.0; Calib[2] = 319.5; Calib[3] = 239.5; Calib[4] = 0.2624; Calib[5] = -0.9531; Calib[6] = -0.0054; Calib[7] = 0.0026; Calib[8] = 1.1633; Calib[9] = 1.035; Calib[10] = 5000.0;

	intrinsics[0] = 2.0*525.0/640.0; intrinsics[5] = 2.0*525.0/480.0; 
	intrinsics[8] = 0.0;
	intrinsics[9] = 0.0; 
	intrinsics[10] = -(Zfar+Znear)/(Zfar-Znear); 
	intrinsics[11] = -1.0; intrinsics[14] = -2.0*(Zfar*Znear)/(Zfar-Znear);


	//////////////////////////////////
	//Init Frame :

	frame = new SegmFrame(HEIGHT, WIDTH);

	SetCalibrationMatrix(Calib);

	frame->LoadFrame("Depth1.tiff", "RGB1.tiff");// definition (en utilisant le GPU) des VBO à partir des 2 images .tiff (inputs)
	
	//get segmented 3D point clouds (blobs) :
	frame->Segment(); //get indexes of segmented part
	frame->Get3DBlobs(); //get segmented 3D point clouds (blobs)

	//Get control points :
	frame->ComputePCA();// give eig vector for each blobs
	frame->GetBboxOriented();//get bboxs and initialise 4 first ctrl pts
	frame->Find2DCtrlPts();// get indexes of control points on the 2D depth image
	frame->Get3DCtrlPts();// get 3Dcontrol points

	//Create Surfaces :
	frame->InitSurfs();
	frame->ComputeSurfs();

}


void keyboard(unsigned char key, int x, int y) {
switch (key) {
	case 100 : if(DISPLAY_FRAME == 0) DISPLAY_FRAME = 1; else if(DISPLAY_FRAME == 1) DISPLAY_FRAME = 0; break;//d
	case 101 : if(DISPLAY_ELMTS == 0) DISPLAY_ELMTS = 1; else if(DISPLAY_ELMTS == 1) DISPLAY_ELMTS = 0; break;//e
	case 110 : if(DISPLAY_BLOBS == 0) DISPLAY_BLOBS = 1; else if(DISPLAY_BLOBS == 1) DISPLAY_BLOBS = 0; break;//n
	case 115 : if(DISPLAY_SURFS == 0) DISPLAY_SURFS = 1; else if(DISPLAY_SURFS == 1) DISPLAY_SURFS = 0; break;//s
	case 114 : if(DISPLAY_REC == 0) DISPLAY_REC = 1; else if(DISPLAY_REC == 1) DISPLAY_REC = 0; break;	//r
	case 116 : if(DISPLAY_REC_PLAN == 0) DISPLAY_REC_PLAN = 1; else if(DISPLAY_REC_PLAN == 1) DISPLAY_REC_PLAN = 0; break;	//t
		
	case 99 :  if(DISPLAY_CONTROLS == 0) DISPLAY_CONTROLS = 1; else if(DISPLAY_CONTROLS == 1) DISPLAY_CONTROLS = 0; break;//s
	case 113 : if(switchh == 0) switchh = 1; 
				else if(switchh == 1) switchh = 2; 
				else if(switchh == 2) switchh = 0; 		   
				break;//q
	case 109 : if(SPEED_M== 0) {SPEED_M = 1; speedVal=0.02;} else if(SPEED_M == 1) {SPEED_M = 0; speedVal=0.004;} break;//m
	case 98 : if(DISPLAY_BB == 0) DISPLAY_BB = 1; else if(DISPLAY_BB == 1) DISPLAY_BB = 0; break;//b
	//case 118 : cin >> NUM_VERTEX ; cout << "Num Vertex " << NUM_VERTEX <<  endl; break ;//v
	case 97 : if(NUM_VERTEX < frame->_blobsF.size()-1) NUM_VERTEX++; break;//a
	case 122 : if(NUM_VERTEX > 0) NUM_VERTEX--;break;//z
//	case 110 : if(DISPLAY_SPLINES == 0) DISPLAY_SPLINES = 1; else if(DISPLAY_SPLINES == 1) DISPLAY_SPLINES = 0; break;//n
	case 27 /* esc */: exit(1) ;
			}

//	printf("angleX = %f,angleY = %f,angleZ = %f, dist = %f, center = %f\n",angleX,angleY,angleZ,dist,center);
	
//printf("vous avez appuyé sur %c ",key);	
//printf("position de la souris : ");	
//printf("%d,%d\n ",x,y);	 // x de 0 à 640 (largeur),  y de 0 à 480 (hauteur)
}

void mouseMove(int x, int y) { 	
 
    // this will only be true when the left button is down
    if (xOrigin >= 0 || yOrigin >= 0) {
 
		// update deltaAngle
		deltaAnglex = (x - xOrigin) * 0.001f;
		deltaAngley = (y - yOrigin) * 0.001f;
 
		// update camera's direction
		//lx = sin(anglex + deltaAnglex);
		//lz = -cos(anglex + deltaAnglex);
		lx = sin(anglex + deltaAnglex);
		ly = cos(anglex + deltaAnglex) * sin(-(angley + deltaAngley));
		lz = -cos(anglex + deltaAnglex) * cos(-(angley + deltaAngley));

		// update camera's direction
		//lxStrap = cos(anglex + deltaAnglex);
		//lzStrap = sin(anglex + deltaAnglex);
		lxStrap = -cos(anglex + deltaAnglex);
		lyStrap = sin(anglex + deltaAnglex) * sin(-(angley + deltaAngley));
		lzStrap = -sin(anglex + deltaAnglex) * cos(-(angley + deltaAngley));
	}
}

void mouseButton(int button, int state, int x, int y) {
 
	// only start motion if the left button is pressed
	if (button == GLUT_LEFT_BUTTON) {
 
		// when the button is released
		if (state == GLUT_UP) {
			anglex += deltaAnglex;
			angley += deltaAngley;
			xOrigin = -1;
			yOrigin = -1;
		}
		else  {// state = GLUT_DOWN
			xOrigin = x;
			yOrigin = y;
		}
	}


	 if ((button == 3) || (button == 4)) // It's a wheel event
	   {
		   // Each wheel event reports like a button click, GLUT_DOWN then GLUT_UP
		   if (state == GLUT_UP) return; // Disregard redundant GLUT_UP events
		   if(button == 4 && SPEED_M){deltaMove = -1.0f;}
		   if(button == 3 && SPEED_M){deltaMove = 1.0f;}
		   if(button == 4 && !SPEED_M){deltaMove = -0.05f;}
		   if(button == 3 && !SPEED_M){deltaMove = 0.05f;}


	   }else{  // normal button event

	   }



}


void pressKey(int key, int xx, int yy) {
       switch (key) {
             case GLUT_KEY_UP : deltaStrap2 = speedVal; break;
             case GLUT_KEY_DOWN : deltaStrap2 = -speedVal; break;
             case GLUT_KEY_LEFT : deltaStrap = speedVal; break;
             case GLUT_KEY_RIGHT : deltaStrap = -speedVal; break;


       }
}

void releaseKey(int key, int x, int y) { 	
 
        switch (key) {
             case GLUT_KEY_LEFT :
             case GLUT_KEY_RIGHT :
             case GLUT_KEY_UP :
             case GLUT_KEY_DOWN : /*deltaMove = 0;*/ deltaStrap = 0; deltaStrap2 = 0; break;
        }
} 

void reshape(int width1, int height1){

	glViewport(0, 0, width1, height1) ;
	glMatrixMode(GL_PROJECTION);						// Select The Projection Matrix
	glLoadIdentity();									// Reset The Projection Matrix
	glLoadMatrixf(intrinsics);

	glMatrixMode (GL_MODELVIEW);
	glLoadIdentity ();


}

void computePos() {
 
	//deltaMove : Molette, deltaStrap : gauche/droite, deltaStrap2 : haut/bas
	x += deltaMove * lx * 0.1f + deltaStrap * lxStrap * 0.8f ;
	y += deltaMove * ly * 0.1f + deltaStrap * lyStrap * 0.8f  + deltaStrap2 * 0.8f;
	z += deltaMove * lz * 0.1f + deltaStrap * lzStrap * 0.8f;

	deltaMove=0;
}

void display(void) {

	if (deltaMove || deltaStrap || deltaStrap2)
	computePos();
	
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT) ;


	glMatrixMode (GL_MODELVIEW);
	glLoadIdentity ();

	// Set up camera intrinsics
	glLoadMatrixf(intrinsics);

	glMatrixMode (GL_MODELVIEW);
	glLoadIdentity ();
	// Set the camera
	gluLookAt(	x, y, z,
			x+lx, y+ly,  z+lz,
			0.0f, 1.0f,  0.0f);


	//Debut du Dessin :
	// axe x horizontal, axe y vertical, axe z profondeur

	if(DISPLAY_FRAME){frame->Draw(1);}
	
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	if(DISPLAY_ELMTS){	frame->DisplayEigVect(NUM_VERTEX);}

	if(DISPLAY_BLOBS){frame->Display3DBlobs(NUM_VERTEX);}	

	DrawAxis0();// Red : x, Green : y, Blue : z

	if(DISPLAY_CONTROLS){frame->Display3DCtrlPts(NUM_VERTEX);}


	for(int num=0; num<frame->_Surfs.size(); num++){
		//int num = 8; //2 chair // 8 globe //7

	//	cout<<"display surf n "<<num<<endl;

		if(DISPLAY_SURFS){frame->_Surfs[num]->DisplaySurfsPlus(0);}

		if(DISPLAY_REC){frame->_Surfs[num]->DisplayRecImg(0);}

		//if(DISPLAY_REC_PLAN){frame->_Surfs[num+1]->DisplayRecImg(1);}
	}

	if(DISPLAY_BB){frame->DisplayBbox();}

	// Fin du dessin
	
	glDisable(GL_BLEND);


	glutSwapBuffers() ;
	glutPostRedisplay() ;

}



/*void oneImageVisu(void){

///////////  Test afficher une image    //////////////

	  //Création et chargement de l'image
    ImageRGBD imageRGBD1;
	imageRGBD1.load("RGB1.tiff", "Depth1.tiff");

	


	// Show our image inside it.

	imageRGBD1.visu();


////////////////////////////////////////////////////

}*/


int _tmain(int argc, char *argv[])
		{
			
///////// init GLUT and create Window
			glutInit(&argc, argv) ;
	
			glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);

			glutInitWindowSize(WIDTH, HEIGHT);

			window = glutCreateWindow("test") ;




			Init();


	/*		///////////verifier si GPU fonctionne////////
			int a;
			a=gpu::getCudaEnabledDeviceCount();
			cout<<"nombre de GPU"<<endl;
			cout<<a<<endl;
			cin>>a;
			*/
////////////////////////////////////////////////////////
	
			/* callback for mouse drags */
			//glutMotionFunc(mousedrag);
			/* callback for idle mouse movement */
			//glutPassiveMotionFunc(mouseidle);
			/* callback for mouse button changes */
			//glutMouseFunc(mousebutton);

     		glutReshapeFunc(reshape);
			glutDisplayFunc(display);

			glutKeyboardFunc(keyboard) ;
			glutSpecialFunc(pressKey);
			glutSpecialUpFunc(releaseKey);

			// here are the two new functions
			glutMouseFunc(mouseButton);
			glutMotionFunc(mouseMove);

			glutMainLoop() ;



			return 0;
		}