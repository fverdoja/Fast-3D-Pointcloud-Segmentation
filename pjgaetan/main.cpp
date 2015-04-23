//#include "stdafx.h" //Visual Studio specific
#include "utilities.h"
#include <boost/timer/timer.hpp>


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
    puts("cudaSetDevice failed ! Do you have a CUDA-capable GPU installed ?");
    exit(EXIT_FAILURE);
  }

  /* GLEW Init */
  code = glewInit();

  if(code != GLEW_OK){
    printf("Coud not initialize GLEW : %s\n",glewGetErrorString(code));
    exit(EXIT_FAILURE);
  }

  glViewport(0, 0, WIDTH, HEIGHT) ;
  glMatrixMode(GL_PROJECTION);			// Select The Projection Matrix
  glLoadIdentity();	

  glMatrixMode (GL_MODELVIEW);
  glLoadIdentity ();

  glShadeModel(GL_SMOOTH);			// Enable Smooth Shading
  glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
  glClearColor(0.0f, 0.0f, 0.0f, 0.5f);	// Black Background
  //glClearColor(1.0f, 1.0f, 1.0f, 0.5f);		// White Background
  glClearDepth(1.0f);				// Depth Buffer Setup

  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  glEnable(GL_DEPTH_TEST);			// Enables Depth Testing
  glDepthFunc(GL_LEQUAL);			// The Type Of Depth Testing To Do
  glEnable(GL_CULL_FACE);
  glCullFace(GL_BACK);				// Backplane culling (do not display surfaces that are not facing the camera)

  glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);	// Really Nice Perspective Calculations
  // enable color tracking
  glEnable(GL_COLOR_MATERIAL);
  // set material properties which will be assigned by glColor
  glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);

  glDisable ( GL_LIGHTING ) ;

  Calib[0]  = 525.0; 
  Calib[1]  = 525.0; 
  Calib[2]  = 319.5; 
  Calib[3]  = 239.5; 
  Calib[4]  = 0.2624; 
  Calib[5]  = -0.9531; 
  Calib[6]  = -0.0054; 
  Calib[7]  = 0.0026; 
  Calib[8]  = 1.1633; 
  Calib[9]  = 1.035; 
  Calib[10] = 5000.0;

  intrinsics[0]  = 2.0*525.0/640.0; 
  intrinsics[5]  = 2.0*525.0/480.0; 
  intrinsics[8]  = 0.0;
  intrinsics[9]  = 0.0; 
  intrinsics[10] = -(Zfar+Znear)/(Zfar-Znear); 
  intrinsics[11] = -1.0;
  intrinsics[14] = -2.0*(Zfar*Znear)/(Zfar-Znear);


  //////////////////////////////////
  //Init Frame :
  //////////////////////////////////

  frame = new SegmFrame(HEIGHT, WIDTH);

  SetCalibrationMatrix(Calib);

  boost::timer::auto_cpu_timer timer; //start timer (will stop when this function ends)
  
  puts("Loading input frames... ");
    frame->LoadFrame("Depth1.tiff", "RGB1.tiff"); // Define the VBO (using the GPU) from the 2 input TIFF images
  
  //get segmented 3D point clouds (blobs) :
  puts("Segmenting the point cloud... ");
    frame->Segment(); //get indexes of segmented part

  puts("Getting segmented blobs... ");
    frame->Get3DBlobs(); //get segmented 3D point clouds (blobs)

  //Get control points :
  puts("Computing PCA... ");
    frame->ComputePCA();// give eig vector for each blobs

  puts("Determining oriented Bboxes... ");
    frame->GetBboxOriented();//get bboxs and initialise 4 first ctrl pts

  puts("Finding control points from 2D depth image... ");
    frame->Find2DCtrlPts();// get indexes of control points on the 2D depth image

  puts("Finding 3D control points... ");
    frame->Get3DCtrlPts();// get 3Dcontrol points

  //Create Surfaces :
  puts("Computing surfaces... ");
    frame->InitSurfs();
    frame->ComputeSurfs();
    puts("Done!");
    
  return;
}


void keyboard(unsigned char key, int x, int y) {
  
// - r : display control points (red) computed from depth image
// - n : write original points in blue (a & z to move between steps)
// - e : display eigen vectors (a & z to move between steps)
// - s : display Surfaces 
// - b : display bounding boxes
// - c : display control points (a et z to move between steps)
// - d : display original pts in RGB
  
  switch (key) {
    case 'd' : DISPLAY_FRAME    = !DISPLAY_FRAME; 	break;	//d
    case 'e' : DISPLAY_ELMTS    = !DISPLAY_ELMTS; 	break;	//e
    case 'n' : DISPLAY_BLOBS    = !DISPLAY_BLOBS; 	break;	//n
    case 's' : DISPLAY_SURFS    = !DISPLAY_SURFS; 	break;	//s
    case 'r' : DISPLAY_REC      = !DISPLAY_REC  ; 	break;	//r
    case 't' : DISPLAY_REC_PLAN = !DISPLAY_REC_PLAN;	break;	//t
    case 'b' : DISPLAY_BB       = !DISPLAY_BB   ;	break;	//b
    case 'c' : DISPLAY_CONTROLS = !DISPLAY_CONTROLS; 	break;	//c
    case 'a' : if(NUM_VERTEX < frame->_blobsF.size() - 1) NUM_VERTEX++; break;//a
    case 'z' : if(NUM_VERTEX > 0) NUM_VERTEX--; break;//z
    
    case 'q' : 
      switch(switchh) {
	default:
	case 0: switchh=1; break;
	case 1: switchh=2; break;
	case 2: switchh=0; break;
      }
    break;
    case 'm' : 
      if (!SPEED_M) {
	SPEED_M = 1; speedVal=0.02;
      }
      else {
	SPEED_M = 0; speedVal=0.004;
      }
    break;//m
    
    
  //case 'v' : cin >> NUM_VERTEX ; cout << "Num Vertex " << NUM_VERTEX <<  endl; break ;//v
  //	case 'n' : if(DISPLAY_SPLINES == 0) DISPLAY_SPLINES = 1; else if(DISPLAY_SPLINES == 1) DISPLAY_SPLINES = 0; break;//n
    
    default:
      //printf("angleX = %f,angleY = %f,angleZ = %f, dist = %f, center = %f\n",angleX,angleY,angleZ,dist,center);
      //printf("you pressed %c ",key);	
      //printf("mouse position : ");	
      //printf("%d,%d\n ",x,y);	 // x de 0 � 640 (width),  y de 0 � 480 (height)
    break;
    
    case 027 /* esc */: exit(EXIT_SUCCESS) ; break;
  }


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
             case GLUT_KEY_UP : deltaStrap2 = speedVal; 	break;
             case GLUT_KEY_DOWN : deltaStrap2 = -speedVal; 	break;
             case GLUT_KEY_LEFT : deltaStrap = speedVal; 	break;
             case GLUT_KEY_RIGHT : deltaStrap = -speedVal; 	break;


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

	  //Cr�ation et chargement de l'image
    ImageRGBD imageRGBD1;
	imageRGBD1.load("RGB1.tiff", "Depth1.tiff");

	


	// Show our image inside it.

	imageRGBD1.visu();


////////////////////////////////////////////////////

}*/


int main(int argc, char *argv[]) {
  puts("Debug: Hello World!");
  
  //check for available GPUs
  {
    int n_cuda=gpu::getCudaEnabledDeviceCount();
    printf("Notice: %d CUDA GPUs detected.\n",n_cuda);
    if(!n_cuda) {
      puts("Error: No CUDA GPU detected! Exiting");
      return EXIT_FAILURE;
    }
    else { //print GPU 0's name
      gpu::DeviceInfo info = gpu::DeviceInfo(0);
      puts("Selected GPU #0:");
      puts(info.name().c_str());
    }
  }
  
///////// init GLUT and create Window
  glutInit(&argc, argv) ;

  glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);

  glutInitWindowSize(WIDTH, HEIGHT);

  window = glutCreateWindow("test") ;

  puts("Debug: GLUT initialized");
  
  Init();


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

  puts("Debug: Entering GLUT main loop");
  glutMainLoop();

  return EXIT_SUCCESS;
}
