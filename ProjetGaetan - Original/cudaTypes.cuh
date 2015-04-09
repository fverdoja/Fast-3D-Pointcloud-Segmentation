#ifndef __CUDA_UTILITIES_H
#define __CUDA_UTILITIES_H

#include <stdio.h>
#include <conio.h>
#include <Windows.h>
#include <Ole2.h>
#include <cuComplex.h>
#include <cuda_runtime.h>
#include <cuda.h>
#include "device_launch_parameters.h"
#include "device_functions.h"

#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include <thrust/sort.h>
#include <thrust/copy.h>
#include <thrust/sequence.h>
#include <thrust/remove.h>
#include <thrust/generate.h>
#include <thrust/detail/type_traits.h>

#include <math.h>
#include <algorithm>
#include <time.h>
#include <limits.h>
#include <vector>

#include <opencv2/core/core.hpp>
#include "opencv2/core/devmem2d.hpp"

// Utilities and timing functions
#include <helper_functions.h>    // includes cuda.h and cuda_runtime_api.h

// CUDA helper functions
#include <helper_cuda.h>         // helper functions for CUDA error check
#include <helper_cuda_gl.h>      // helper functions for CUDA/GL interop


// Preprocessor definitions for width and height of color and depth streams
#define THREAD_SIZE_X 32
#define THREAD_SIZE_Y 32
#define THREAD_SIZE_L_X 8
#define THREAD_SIZE_L_Y 8
#define THREAD_SIZE THREAD_SIZE_L_X*THREAD_SIZE_L_Y
#define STRIDE 512
#define MAXTOLERANCE 0.2
#define EPSILON 0.1
#define ALPHA 0.8 
#define PI 3.1415926535897932384626433832795
#define MAX_DEPTH 100.0

#define divUp(x,y) (x%y) ? ((x+y-1)/y) : (x/y)

using namespace std;

#endif