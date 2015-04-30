#ifndef __UTILITIES_H
#define __UTILITIES_H

/* #include <windows.h> */
#include "typedefs.h" //define LONG, USHORT and BYTE (microsoft, why y do dis?)

#include <GL/glew.h>
#include <GL/glut.h>
#include <vector>


#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string.h>
#include <math.h>

#include <tuple>

//#include <cv.h>
#include <opencv/highgui.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include "opencv2/features2d/features2d.hpp"
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/nonfree/features2d.hpp"
//#include <opencv2/gpu/gpu.hpp>
#include <opencv2/nonfree/gpu.hpp>


#include <cuda_runtime.h>
#include <cuda_gl_interop.h>

// Utilities and timing functions
#include <helper_functions.h>    // includes cuda.h and cuda_runtime_api.h

// CUDA helper functions
#include <helper_cuda.h>         // helper functions for CUDA error check
#include <helper_cuda_gl.h>      // helper functions for CUDA/GL interop

//CGAL
#include <CGAL/compiler_config.h>

using namespace std;

using namespace cv;

#define WIDTH 640
#define HEIGHT 480

#define MIN_SIZE_PLAN 2000 //min nb of points in a blob


#define DISPLAY_FRAME_IN true



//#include "Surface.h"
#include "Frame.h"


#include "Kernel.cuh"
#include <limits>

#define STR_LEN 256
static char FILENAME_DEPTH[STR_LEN] = "Depth1.tiff"; //default image name
static char FILENAME_COLOR[STR_LEN] = "RGB1.tiff";
static char FILENAME_EXT[] = ".tiff";

void FindBlobs(const cv::Mat &binary, std::vector < std::vector<cv::Point2i> > &blobs);

#endif
