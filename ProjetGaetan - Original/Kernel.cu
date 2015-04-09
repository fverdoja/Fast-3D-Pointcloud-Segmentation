#include "Kernel.cuh"

/******* Global variables ******/
texture<float4, cudaTextureType2D, cudaReadModeElementType> texRef;

float *calib_dev;
float *pose_dev;

void SetCalibrationMatrix(float *Calib_CPU) {
	checkCudaErrors( cudaMalloc((void**)&calib_dev, 11 * sizeof(float)) );
	checkCudaErrors( cudaMemcpy(calib_dev, Calib_CPU,  11 * sizeof(float), cudaMemcpyHostToDevice) );
}

void FreeCalibrationMatrix() {
	checkCudaErrors( cudaFree(calib_dev) );
}

void AllocPoseMatrix() {
	checkCudaErrors( cudaMalloc((void**)&pose_dev, 16 * sizeof(float)) );
}

void SetPoseMatrix(float *pose_CPU) {
	checkCudaErrors( cudaMemcpy(pose_dev, pose_CPU,  16 * sizeof(float), cudaMemcpyHostToDevice) );
}

void FreePoseMatrix() {
	checkCudaErrors( cudaFree(pose_dev) );
}

//////////////////////////////////////////
///**** Device Function definitions ****/
/////////////////////////////////////////
__device__ __forceinline__ void VertexMapKinectProcess(cv::gpu::DevMem2D_<unsigned short> d, float *out, float *calib, int n, int m)
{
	unsigned int i = threadIdx.x + blockIdx.x * THREAD_SIZE_X;
    unsigned int j = threadIdx.y + blockIdx.y * THREAD_SIZE_Y;
    unsigned int idx = i*m + j;
	
	if (i > n-1 || j > m-1)
		return;
		
	out[3*idx] = d(n-i-1,j) == 0 ? 0.0 : (calib[9]/calib[10]) * float(d(n-i-1,j)) * ((float(j)-calib[2])/calib[0]);// axe y
	out[3*idx+1] = d(n-i-1,j) == 0 ? 0.0 : (calib[9]/calib[10]) * float(d(n-i-1,j)) * ((float(i)-calib[3])/calib[1]);// axe x
	out[3*idx+2] = d(n-i-1,j) == 0 ? 0.0 : -(calib[9]/calib[10]) * float(d(n-i-1,j));// axe z
	// rappel:  A == 0 ? 0.0 :B si la condition est vérifiée A = 0.0 sinon A = B.
	//n-i-1 pour convention OpenCV vs OpenGL
}
__global__ void VertexMapKinectKernel(cv::gpu::DevMem2D_<unsigned short> d, float *out, float *calib, int n, int m)
{
    VertexMapKinectProcess(d, out, calib, n, m);

}

__device__ __forceinline__ void VertexMapGLProcess(float *rgb, float *out, float *calib, int n, int m)
{
    // identifiant de thread ? deux dimensions, comme la matrice
	unsigned int i = threadIdx.x + blockIdx.x * THREAD_SIZE_X;
    unsigned int j = threadIdx.y + blockIdx.y * THREAD_SIZE_Y;
    unsigned int idx = i*m + j;

	if (i > n-1 || j > m-1)
		return;
		
	out[3*idx] = rgb[4*idx+3] == 0.0 ? 0.0 : (rgb[4*idx+3]*MAX_DEPTH) * ((float(j)-calib[2])/calib[0]);
	out[3*idx+1] = rgb[4*idx+3] == 0.0 ? 0.0 : (rgb[4*idx+3]*MAX_DEPTH) * ((float(i)-calib[3])/calib[1]);
	out[3*idx+2] = -rgb[4*idx+3]*MAX_DEPTH;
	
}
__global__ void VertexMapGLKernel(float *rgb, float *out, float *calib, int n, int m)
{
	VertexMapGLProcess(rgb, out, calib, n, m);
}

__device__ __forceinline__ void MapData2VBOKProcess(BYTE *d_colorFrame, USHORT *d_depthFrame, LONG *d_colorCoord, float *VMap, float *RGB, float *calib, int n, int m) {
	float fx = calib[0];  //# focal length x
    float fy = calib[1];  //# focal length y
    float cx = calib[2];  //# optical center x
    float cy = calib[3];  //# optical center y

    // identifiant de thread ? deux dimensions, comme la matrice
	unsigned int i = threadIdx.x + blockIdx.x * THREAD_SIZE_X;
    unsigned int j = threadIdx.y + blockIdx.y * THREAD_SIZE_Y;
    unsigned int idx = i*m + j;
    unsigned int idx_new = (n-1-i)*m + (m-1-j);

	if (i > n-1 || j > m-1)
		return;

	//USHORT depth_v = d_depthFrame[idx];
	//int playerIndex = depth & ((1 << 3)-1);
	// Get the point coordinate in World Space
	float pointCoord[3];
	
	// convert from mm to m and pixels to m using Kinect Calibration values
	pointCoord[2] = (float)(-calib[9])*(d_depthFrame[idx])/calib[10]; 
	pointCoord[0] = (float(j) - cx)*pointCoord[2] / fx;
	pointCoord[1] = (float(i) - cy)*pointCoord[2] / fy;
	
	// extracting color coordinates mapped from the depth position (depthIndex)
	LONG colorForDepthX = d_colorCoord[idx*2];
	LONG colorForDepthY = d_colorCoord[idx*2 + 1];

	// check if the color coordinates lie within the range of the color map
	if(colorForDepthX >= 0 && colorForDepthX < m && colorForDepthY >= 0 && colorForDepthY < n && pointCoord[2] > -MAX_DEPTH)
	{
		// calculate index in the color image
		int colorIndex = colorForDepthY * (m*4) + (colorForDepthX*4);
		int m_ColorIndex = (n-1-i)*(m*3)+(m-1-j)*3;
		
		VMap[3*idx_new] = pointCoord[0];
		VMap[3*idx_new+1] = pointCoord[1];
		VMap[3*idx_new+2] = pointCoord[2];

		RGB[m_ColorIndex] = float(d_colorFrame[colorIndex+2])/255.0;
		RGB[m_ColorIndex+1] = float(d_colorFrame[colorIndex+1])/255.0;
		RGB[m_ColorIndex+2] = float(d_colorFrame[colorIndex])/255.0;
	}
}
__global__ void MapData2VBOKernel(BYTE *d_colorFrame, USHORT *d_depthFrame, LONG *d_colorCoord, float *VMap, float *RGB, float *calib, int n, int m) {
	MapData2VBOKProcess(d_colorFrame, d_depthFrame, d_colorCoord, VMap, RGB, calib, n, m);
}

__device__ __forceinline__ void NormalProcess(float *NMap, float *VMap, int n, int m, bool inverse) {
	float p1 [3];
    float p2 [3];
    float p3 [3];
    float n_p [3];
    float n_p1 [3];
    float n_p2 [3];
    float n_p3 [3];
    float n_p4 [3];
    float norm_n;


    // identifiant de thread a deux dimensions, comme la matrice
	int i = threadIdx.x + blockIdx.x * THREAD_SIZE_L_X;
    int j = threadIdx.y + blockIdx.y * THREAD_SIZE_L_Y;
    int idx = i*m + j;
    int idx_out = i*m + j;

    unsigned short n_tot = 0;

    if (i > n-1 || j > m-1)
        return;
        
    if ( i < 1 || i > n-2 || j < 1 || j > m-2 || VMap[3*idx+2] == 0.0) {
		NMap[3*idx_out] = 0.0;
        NMap[3*idx_out+1] = 0.0;
        NMap[3*idx_out+2] = 0.0;
        return;
    }

    p1[0] = VMap[3*idx];
    p1[1] = VMap[3*idx+1];
    p1[2] = VMap[3*idx+2];

    n_p1[0] = 0.0; n_p1[1] = 0.0; n_p1[2] = 0.0;
    n_p2[0] = 0.0; n_p2[1] = 0.0; n_p2[2] = 0.0;
    n_p3[0] = 0.0; n_p3[1] = 0.0; n_p3[2] = 0.0;
    n_p4[0] = 0.0; n_p4[1] = 0.0; n_p4[2] = 0.0;

    ////////////////////////// Triangle 1 /////////////////////////////////

    idx = (i+1)*m + j;
    p2[0] = VMap[3*idx];
    p2[1] = VMap[3*idx+1];
    p2[2] = VMap[3*idx+2];

    idx = i*m + (j+1);
    p3[0] = VMap[3*idx];
    p3[1] = VMap[3*idx+1];
    p3[2] = VMap[3*idx+2];

    if (p2[2] != 0.0 && p3[2] != 0.0) {
        n_p1[0] = (p2[1]-p1[1])*(p3[2]-p1[2]) - (p2[2]-p1[2])*(p3[1]-p1[1]);
        n_p1[1] = (p2[2]-p1[2])*(p3[0]-p1[0]) - (p2[0]-p1[0])*(p3[2]-p1[2]);
        n_p1[2] = (p2[0]-p1[0])*(p3[1]-p1[1]) - (p2[1]-p1[1])*(p3[0]-p1[0]);

        norm_n = (n_p1[0]*n_p1[0] + n_p1[1]*n_p1[1] + n_p1[2]*n_p1[2]);

        if (norm_n != 0.0) {
            n_p1[0] = n_p1[0] / sqrt(norm_n);
            n_p1[1] = n_p1[1] / sqrt(norm_n);
            n_p1[2] = n_p1[2] / sqrt(norm_n);

            n_tot++;
        }
    }

    ////////////////////////// Triangle 2 /////////////////////////////////

    idx = i*m + (j+1);
    p2[0] = VMap[3*idx];
    p2[1] = VMap[3*idx+1];
    p2[2] = VMap[3*idx+2];

    idx = (i-1)*m + j;
    p3[0] = VMap[3*idx];
    p3[1] = VMap[3*idx+1];
    p3[2] = VMap[3*idx+2];

    if (p2[2] != 0.0 && p3[2] != 0.0) {
        n_p2[0] = (p2[1]-p1[1])*(p3[2]-p1[2]) - (p2[2]-p1[2])*(p3[1]-p1[1]);
        n_p2[1] = (p2[2]-p1[2])*(p3[0]-p1[0]) - (p2[0]-p1[0])*(p3[2]-p1[2]);
        n_p2[2] = (p2[0]-p1[0])*(p3[1]-p1[1]) - (p2[1]-p1[1])*(p3[0]-p1[0]);

        norm_n = (n_p2[0]*n_p2[0] + n_p2[1]*n_p2[1] + n_p2[2]*n_p2[2]);

        if (norm_n != 0.0) {
            n_p2[0] = n_p2[0] / sqrt(norm_n);
            n_p2[1] = n_p2[1] / sqrt(norm_n);
            n_p2[2] = n_p2[2] / sqrt(norm_n);

            n_tot++;
        }
    }

    ////////////////////////// Triangle 3 /////////////////////////////////

    idx = (i-1)*m + j;
    p2[0] = VMap[3*idx];
    p2[1] = VMap[3*idx+1];
    p2[2] = VMap[3*idx+2];

    idx = i*m + (j-1);
    p3[0] = VMap[3*idx];
    p3[1] = VMap[3*idx+1];
    p3[2] = VMap[3*idx+2];

    if (p2[2] != 0.0 && p3[2] != 0.0) {
        n_p3[0] = (p2[1]-p1[1])*(p3[2]-p1[2]) - (p2[2]-p1[2])*(p3[1]-p1[1]);
        n_p3[1] = (p2[2]-p1[2])*(p3[0]-p1[0]) - (p2[0]-p1[0])*(p3[2]-p1[2]);
        n_p3[2] = (p2[0]-p1[0])*(p3[1]-p1[1]) - (p2[1]-p1[1])*(p3[0]-p1[0]);

        norm_n = (n_p3[0]*n_p3[0] + n_p3[1]*n_p3[1] + n_p3[2]*n_p3[2]);

        if (norm_n != 0) {
            n_p3[0] = n_p3[0] / sqrt(norm_n);
            n_p3[1] = n_p3[1] / sqrt(norm_n);
            n_p3[2] = n_p3[2] / sqrt(norm_n);

            n_tot++;
        }
    }

    ////////////////////////// Triangle 4 /////////////////////////////////

    idx = i*m+ (j-1);
    p2[0] = VMap[3*idx];
    p2[1] = VMap[3*idx+1];
    p2[2] = VMap[3*idx+2];

    idx = (i+1)*m + j;
    p3[0] = VMap[3*idx];
    p3[1] = VMap[3*idx+1];
    p3[2] = VMap[3*idx+2];

    if (p2[2] != 0.0 && p3[2] != 0.0) {
        n_p4[0] = (p2[1]-p1[1])*(p3[2]-p1[2]) - (p2[2]-p1[2])*(p3[1]-p1[1]);
        n_p4[1] = (p2[2]-p1[2])*(p3[0]-p1[0]) - (p2[0]-p1[0])*(p3[2]-p1[2]);
        n_p4[2] = (p2[0]-p1[0])*(p3[1]-p1[1]) - (p2[1]-p1[1])*(p3[0]-p1[0]);

        norm_n = (n_p4[0]*n_p4[0] + n_p4[1]*n_p4[1] + n_p4[2]*n_p4[2]);

        if (norm_n != 0) {
            n_p4[0] = n_p4[0] / sqrt(norm_n);
            n_p4[1] = n_p4[1] / sqrt(norm_n);
            n_p4[2] = n_p4[2] / sqrt(norm_n);

            n_tot++;
        }
    }

    if (n_tot == 0) {
		NMap[3*idx_out] = 0.0;
        NMap[3*idx_out+1] = 0.0;
        NMap[3*idx_out+2] = 0.0;
        return;
    }

    n_p[0] = (n_p1[0] + n_p2[0] + n_p3[0] + n_p4[0])/float(n_tot);
    n_p[1] = (n_p1[1] + n_p2[1] + n_p3[1] + n_p4[1])/float(n_tot);
    n_p[2] = (n_p1[2] + n_p2[2] + n_p3[2] + n_p4[2])/float(n_tot);

    norm_n = sqrt(n_p[0]*n_p[0] + n_p[1]*n_p[1] + n_p[2]*n_p[2]);

    if (norm_n != 0) {
		if (inverse) {
			NMap[3*idx_out] = -n_p[0]/norm_n;
			NMap[3*idx_out+1] =  -n_p[1]/norm_n;
			NMap[3*idx_out+2] = -n_p[2]/norm_n;
		} else {
			NMap[3*idx_out] = n_p[0]/norm_n;
			NMap[3*idx_out+1] =  n_p[1]/norm_n;
			NMap[3*idx_out+2] = n_p[2]/norm_n;
		}
    } else {
		NMap[3*idx_out] = 0.0;
        NMap[3*idx_out+1] = 0.0;
        NMap[3*idx_out+2] = 0.0;
	}

    return;
}
__global__ void NormalKernel(float *NMap, float *VMap, int n, int m, bool inverse) {
	NormalProcess(NMap, VMap, n, m, inverse);
}

__device__ __forceinline__ void ReadProcess(float *VMap, float *RGBD, float *calib,  int n, int m) {
	int i = threadIdx.x + blockIdx.x * THREAD_SIZE_X;
    int j = threadIdx.y + blockIdx.y * THREAD_SIZE_Y;
    int idx = i*m + j;

	if (i > n-1 || j > m-1)
        return;

	float4 RGBD_v = tex2D(texRef, (float)j+0.5f,  (float)i+0.5f);

	if (RGBD_v.x == 0.0 && RGBD_v.y == 0.0 && RGBD_v.z == 0.0) {
		VMap[3*idx] = 0.0;
		VMap[3*idx+1] = 0.0;
		VMap[3*idx+2] = 0.0;
		RGBD[4*idx+3] = 0.0;	
		return;
	}
	
	float z;
	z = RGBD_v.w*MAX_DEPTH;
		
	VMap[3*idx] = z * ((float(j)-calib[2])/calib[0]);
	VMap[3*idx+1] = z * ((float(i)-calib[3])/calib[1]);
	VMap[3*idx+2] = -z;
			
	RGBD[4*idx] = RGBD_v.x;
	RGBD[4*idx+1] = RGBD_v.y;
	RGBD[4*idx+2] = RGBD_v.z;
	RGBD[4*idx+3] = RGBD_v.w;	

	return;
}
__global__ void ReadKernel(float *VMap, float *RGBD, float *calib, int n, int m) {
	ReadProcess(VMap, RGBD, calib, n, m);
}

__device__ __forceinline__ void TransformVertexMapProcess(float *pose, float *Vmap, float *RGBD, int n, int m)
{
	float x, y, z;

	// identifiant de thread ? deux dimensions, comme la matrice
	unsigned int i = threadIdx.x + blockIdx.x * THREAD_SIZE_X;
    unsigned int j = threadIdx.y + blockIdx.y * THREAD_SIZE_Y;
    unsigned int idx = i*m + j;

	if (i > n-1 || j > m-1)
		return;

    if ((Vmap[3*idx] == 0.0 && Vmap[3*idx+1] == 0.0 && Vmap[3*idx+2] == 0.0)) {
		Vmap[3*idx] = 0.0;
		Vmap[3*idx+1] = 0.0;
		Vmap[3*idx+2] = 0.0;
		RGBD[4*idx+3] = 0.0;
		return;
	}

	x = pose[0]*Vmap[3*idx] + pose[4]*Vmap[3*idx+1] + pose[8]*Vmap[3*idx+2] + pose[12];
	y = pose[1]*Vmap[3*idx] + pose[5]*Vmap[3*idx+1] + pose[9]*Vmap[3*idx+2] + pose[13];
	z = pose[2]*Vmap[3*idx] + pose[6]*Vmap[3*idx+1] + pose[10]*Vmap[3*idx+2] + pose[14];

    Vmap[3*idx] = x;
    Vmap[3*idx+1] = y;
    Vmap[3*idx+2] = z;
	RGBD[4*idx+3] = (-z > MAX_DEPTH || -z < 0.0) ? 0.0 : -z/MAX_DEPTH;


}
__global__ void TransformVertexMapKernel(float *pose, float *Vmap, float *RGBD, int n, int m)
{
	TransformVertexMapProcess(pose, Vmap, RGBD, n, m);
}

__device__ __forceinline__ void QuadTrimFrameProcess(unsigned int *index_dev, float *VMap, int n, int m, float thresh)
{
	unsigned int VIdx [4];

	// identifiant de thread a deux dimensions, comme la matrice
	int i = threadIdx.x + blockIdx.x * THREAD_SIZE_L_X;
    int j = threadIdx.y + blockIdx.y * THREAD_SIZE_L_Y;
    int idx = i*(m-1) + j;
	
	if (i > n-2 || j > m-2)
		return;

	VIdx[0] = i*m+j; 
	VIdx[1] = i*m+j+1; 
	VIdx[2] = (i+1)*m+j+1; 
	VIdx[3] = (i+1)*m+j; 

	float diff1 = sqrt((VMap[3*VIdx[0]]-VMap[3*VIdx[1]])*(VMap[3*VIdx[0]]-VMap[3*VIdx[1]]) + (VMap[3*VIdx[0]+1]-VMap[3*VIdx[1]+1])*(VMap[3*VIdx[0]+1]-VMap[3*VIdx[1]+1])
						+ (VMap[3*VIdx[0]+2]-VMap[3*VIdx[1]+2])*(VMap[3*VIdx[0]+2]-VMap[3*VIdx[1]+2])); 

	float diff2 = sqrt((VMap[3*VIdx[0]]-VMap[3*VIdx[2]])*(VMap[3*VIdx[0]]-VMap[3*VIdx[2]]) + (VMap[3*VIdx[0]+1]-VMap[3*VIdx[2]+1])*(VMap[3*VIdx[0]+1]-VMap[3*VIdx[2]+1])
						+ (VMap[3*VIdx[0]+2]-VMap[3*VIdx[2]+2])*(VMap[3*VIdx[0]+2]-VMap[3*VIdx[2]+2])); 

	float diff3 = sqrt((VMap[3*VIdx[0]]-VMap[3*VIdx[3]])*(VMap[3*VIdx[0]]-VMap[3*VIdx[3]]) + (VMap[3*VIdx[0]+1]-VMap[3*VIdx[3]+1])*(VMap[3*VIdx[0]+1]-VMap[3*VIdx[3]+1])
						+ (VMap[3*VIdx[0]+2]-VMap[3*VIdx[3]+2])*(VMap[3*VIdx[0]+2]-VMap[3*VIdx[3]+2])); 

	float diff4 = sqrt((VMap[3*VIdx[1]]-VMap[3*VIdx[2]])*(VMap[3*VIdx[1]]-VMap[3*VIdx[2]]) + (VMap[3*VIdx[1]+1]-VMap[3*VIdx[2]+1])*(VMap[3*VIdx[1]+1]-VMap[3*VIdx[2]+1])
						+ (VMap[3*VIdx[1]+2]-VMap[3*VIdx[2]+2])*(VMap[3*VIdx[1]+2]-VMap[3*VIdx[2]+2])); 

	float diff5 = sqrt((VMap[3*VIdx[1]]-VMap[3*VIdx[3]])*(VMap[3*VIdx[1]]-VMap[3*VIdx[3]]) + (VMap[3*VIdx[1]+1]-VMap[3*VIdx[3]+1])*(VMap[3*VIdx[1]+1]-VMap[3*VIdx[3]+1])
						+ (VMap[3*VIdx[1]+2]-VMap[3*VIdx[3]+2])*(VMap[3*VIdx[1]+2]-VMap[3*VIdx[3]+2])); 

	float diff6 = sqrt((VMap[3*VIdx[2]]-VMap[3*VIdx[3]])*(VMap[3*VIdx[2]]-VMap[3*VIdx[3]]) + (VMap[3*VIdx[2]+1]-VMap[3*VIdx[3]+1])*(VMap[3*VIdx[2]+1]-VMap[3*VIdx[3]+1])
						+ (VMap[3*VIdx[2]+2]-VMap[3*VIdx[3]+2])*(VMap[3*VIdx[2]+2]-VMap[3*VIdx[3]+2])); 

	float max_diff = max(diff1, diff2);
	max_diff = max(diff3, max_diff);
	max_diff = max(diff4, max_diff);
	max_diff = max(diff5, max_diff);
	max_diff = max(diff6, max_diff);
	if (max_diff > thresh) {
		index_dev[4*idx] = 0;
		index_dev[4*idx+1] = 0;
		index_dev[4*idx+2] = 0;
		index_dev[4*idx+3] = 0;
	} else {
		index_dev[4*idx] = VIdx[0];
		index_dev[4*idx+1] = VIdx[1];
		index_dev[4*idx+2] = VIdx[2];
		index_dev[4*idx+3] = VIdx[3];
	}

}
__global__ void QuadTrimFrameKernel(unsigned int *index_dev, float *VMap, int n, int m, float thresh)
{
	QuadTrimFrameProcess(index_dev, VMap, n, m, thresh);

}

__device__ __forceinline__ void MergeDepthOffProcess(float *RGB_predicted, cv::gpu::DevMem2D_<unsigned short> depth, float *RGB, unsigned char *mask, float *calib, int n, int m) {
	unsigned int i = threadIdx.x + blockIdx.x * THREAD_SIZE_L_X;
    unsigned int j = threadIdx.y + blockIdx.y * THREAD_SIZE_L_Y;
    unsigned int idx = i*m + j;

	if (i > n-1 || j > m-1)
        return;
	
    float ds = calib[9];  //# distortion
    float factor = calib[10];  //# factor

	//float w = 1.0;//(1.0+sqrt((i-n/2.0)*(i-n/2.0) + (j-m/2.0)*(j-m/2.0)))/((n+m)/2.0);

	float depth_val = (ds/factor) * float(depth(n-i-1, j)); //[i*m+j];
	float depth_predicted_val = RGB_predicted[4*idx+3]*MAX_DEPTH; //depth_predicted[idx];
	float mask_val = float(mask[idx])-10.0;
	float mask_tmp = 1.0;//min(1.0,(2.0/depth_val))/w;

	//if (!RGBDREP) {
		/*RGB_predicted[4*idx] = RGB[4*idx];
		RGB_predicted[4*idx+1] = RGB[4*idx+1];
		RGB_predicted[4*idx+2] = RGB[4*idx+2];*/
	//}
	RGB_predicted[4*idx] = RGB[3*idx];
	RGB_predicted[4*idx+1] = RGB[3*idx+1];
	RGB_predicted[4*idx+2] = RGB[3*idx+2];
	
	if (depth_val < 0.1) {
		//maskRGB[4*idx+1] = (20.0)/255.0;
		//depth_predicted[idx] = 0.0;
		/*RGB_predicted[4*idx] = 0.0;
		RGB_predicted[4*idx+1] = 0.0;
		RGB_predicted[4*idx+2] = 0.0;*/
		//maskRGB[4*idx] = (10.0)/255.0;
		return;
	}

	if (depth_predicted_val < 0.1 || fabs(mask_val+9.0) < 0.01) {
		RGB_predicted[4*idx+3] = depth_val/MAX_DEPTH;
		//depth_predicted[idx] = depth_val;
		/*RGB_predicted[4*idx] = RGB[4*idx];
		RGB_predicted[4*idx+1] = RGB[4*idx+1];
		RGB_predicted[4*idx+2] = RGB[4*idx+2];*/
		mask[idx] = unsigned char((mask_tmp+10.0));
		//maskRGB[4*idx] = (mask_tmp+10.0)/255.0;
		//maskRGB[4*idx+1] = (20.0)/255.0;
		return;
	}
	
	if (depth_val < (depth_predicted_val - MAXTOLERANCE)) {//occlusion	
		RGB_predicted[4*idx+3] = depth_val/MAX_DEPTH;
		//depth_predicted[idx] = depth_val;
		/*RGB_predicted[4*idx] = RGB[4*idx];
		RGB_predicted[4*idx+1] = RGB[4*idx+1];
		RGB_predicted[4*idx+2] = RGB[4*idx+2];	*/
		mask[idx] = unsigned char((mask_tmp+10.0));
		//maskRGB[4*idx] = (mask_tmp+10.0)/255.0;
		//maskRGB[4*idx+1] = (20.0)/255.0;
		return;
	}

	
	if (depth_val > (depth_predicted_val + max(1.0,1.0*depth_val)*MAXTOLERANCE)) { // visibility violation
		mask[idx] = 10;
		//maskRGB[4*idx] = 10.0/255.0;
		//maskRGB[4*idx+1] = 0.0;
		/*RGB_predicted[4*idx] = RGB[4*idx];
		RGB_predicted[4*idx+1] = RGB[4*idx+1];
		RGB_predicted[4*idx+2] = RGB[4*idx+2];*/
		return;
	}

	RGB_predicted[4*idx+3] = ((depth_val*mask_tmp + fabs(mask_val)*depth_predicted_val)/(mask_tmp+fabs(mask_val)))/MAX_DEPTH;
	//depth_predicted[idx] = (depth_val*mask_tmp + fabs(mask)*depth_predicted_val)/(mask_tmp+fabs(mask));
	/*RGB_predicted[4*idx] = (RGB[4*idx]*mask_tmp + fabs(mask)*RGB_predicted[4*idx])/(mask_tmp+fabs(mask));
	RGB_predicted[4*idx+1] = (RGB[4*idx+1]*mask_tmp + fabs(mask)*RGB_predicted[4*idx+1])/(mask_tmp+fabs(mask));
	RGB_predicted[4*idx+2] = (RGB[4*idx+2]*mask_tmp + fabs(mask)*RGB_predicted[4*idx+2])/(mask_tmp+fabs(mask));*/
	/*RGB_predicted[4*idx] = RGB[4*idx];
	RGB_predicted[4*idx+1] = RGB[4*idx+1];
	RGB_predicted[4*idx+2] = RGB[4*idx+2];*/
	mask[idx] = unsigned char(min(255.0, mask_tmp+fabs(mask_val)+10.0));
	//maskRGB[4*idx] = min(1.0, ((mask_tmp+fabs(mask))+10)/255.0);
	//maskRGB[4*idx+1] = (20.0)/255.0;
}
__global__ void MergeDepthOffKernel(float *RGB_predicted, cv::gpu::DevMem2D_<unsigned short> depth, float *RGB, unsigned char *mask, float *calib, int n, int m) {
	MergeDepthOffProcess(RGB_predicted, depth, RGB, mask, calib, n, m);
}

__device__ __forceinline__ void MergeDepthKinectProcess(float *RGB_predicted, unsigned short *depth, float *RGB, unsigned char *mask, float *calib, int n, int m) {
	unsigned int i = threadIdx.x + blockIdx.x * THREAD_SIZE_L_X;
    unsigned int j = threadIdx.y + blockIdx.y * THREAD_SIZE_L_Y;
    unsigned int idx = i*m + j;

	if (i > n-1 || j > m-1)
        return;
	
    float ds = calib[9];  //# distortion
    float factor = calib[10];  //# factor

	//float w = 1.0;//(1.0+sqrt((i-n/2.0)*(i-n/2.0) + (j-m/2.0)*(j-m/2.0)))/((n+m)/2.0);

	float depth_val = (ds/factor) * float(depth[(n-1-i)*m + (m-1-j)]); 
	float depth_predicted_val = RGB_predicted[4*idx+3]*MAX_DEPTH; //depth_predicted[idx];
	float mask_val = float(mask[idx])-10.0;
	float mask_tmp = 1.0;//min(1.0,(2.0/depth_val))/w;

	//if (!RGBDREP) {
		/*RGB_predicted[4*idx] = RGB[4*idx];
		RGB_predicted[4*idx+1] = RGB[4*idx+1];
		RGB_predicted[4*idx+2] = RGB[4*idx+2];*/
	//}
	RGB_predicted[4*idx] = RGB[3*idx];
	RGB_predicted[4*idx+1] = RGB[3*idx+1];
	RGB_predicted[4*idx+2] = RGB[3*idx+2];
	
	if (depth_val < 0.1) {
		//maskRGB[4*idx+1] = (20.0)/255.0;
		//depth_predicted[idx] = 0.0;
		/*RGB_predicted[4*idx] = 0.0;
		RGB_predicted[4*idx+1] = 0.0;
		RGB_predicted[4*idx+2] = 0.0;*/
		//maskRGB[4*idx] = (10.0)/255.0;
		return;
	}

	if (depth_predicted_val < 0.1 || fabs(mask_val+9.0) < 0.01) {
		RGB_predicted[4*idx+3] = depth_val/MAX_DEPTH;
		//depth_predicted[idx] = depth_val;
		/*RGB_predicted[4*idx] = RGB[4*idx];
		RGB_predicted[4*idx+1] = RGB[4*idx+1];
		RGB_predicted[4*idx+2] = RGB[4*idx+2];*/
		mask[idx] = unsigned char((mask_tmp+10.0));
		//maskRGB[4*idx] = (mask_tmp+10.0)/255.0;
		//maskRGB[4*idx+1] = (20.0)/255.0;
		return;
	}
	
	if (depth_val < (depth_predicted_val - MAXTOLERANCE)) {//occlusion	
		RGB_predicted[4*idx+3] = depth_val/MAX_DEPTH;
		//depth_predicted[idx] = depth_val;
		/*RGB_predicted[4*idx] = RGB[4*idx];
		RGB_predicted[4*idx+1] = RGB[4*idx+1];
		RGB_predicted[4*idx+2] = RGB[4*idx+2];	*/
		mask[idx] = unsigned char((mask_tmp+10.0));
		//maskRGB[4*idx] = (mask_tmp+10.0)/255.0;
		//maskRGB[4*idx+1] = (20.0)/255.0;
		return;
	}

	
	if (depth_val > (depth_predicted_val + max(1.0,1.0*depth_val)*MAXTOLERANCE)) { // visibility violation
		mask[idx] = 10;
		//maskRGB[4*idx] = 10.0/255.0;
		//maskRGB[4*idx+1] = 0.0;
		/*RGB_predicted[4*idx] = RGB[4*idx];
		RGB_predicted[4*idx+1] = RGB[4*idx+1];
		RGB_predicted[4*idx+2] = RGB[4*idx+2];*/
		return;
	}

	RGB_predicted[4*idx+3] = ((depth_val*mask_tmp + fabs(mask_val)*depth_predicted_val)/(mask_tmp+fabs(mask_val)))/MAX_DEPTH;
	//depth_predicted[idx] = (depth_val*mask_tmp + fabs(mask)*depth_predicted_val)/(mask_tmp+fabs(mask));
	/*RGB_predicted[4*idx] = (RGB[4*idx]*mask_tmp + fabs(mask)*RGB_predicted[4*idx])/(mask_tmp+fabs(mask));
	RGB_predicted[4*idx+1] = (RGB[4*idx+1]*mask_tmp + fabs(mask)*RGB_predicted[4*idx+1])/(mask_tmp+fabs(mask));
	RGB_predicted[4*idx+2] = (RGB[4*idx+2]*mask_tmp + fabs(mask)*RGB_predicted[4*idx+2])/(mask_tmp+fabs(mask));*/
	/*RGB_predicted[4*idx] = RGB[4*idx];
	RGB_predicted[4*idx+1] = RGB[4*idx+1];
	RGB_predicted[4*idx+2] = RGB[4*idx+2];*/
	mask[idx] = unsigned char(min(255.0, mask_tmp+fabs(mask_val)+10.0));
	//maskRGB[4*idx] = min(1.0, ((mask_tmp+fabs(mask))+10)/255.0);
	//maskRGB[4*idx+1] = (20.0)/255.0;
}
__global__ void MergeDepthKinectKernel(float *RGB_predicted, unsigned short *depth, float *RGB, unsigned char *mask, float *calib, int n, int m) {
	MergeDepthKinectProcess(RGB_predicted, depth, RGB, mask, calib, n, m);
}

__device__ __forceinline__ void InitMaskProcess (unsigned char *mask, float *RGB, int n, int m) {
	int i = threadIdx.x + blockIdx.x * THREAD_SIZE_X;
    int j = threadIdx.y + blockIdx.y * THREAD_SIZE_Y;
    int idx = i*m + j;

	if (i > n-1 || j > m-1)
        return;

	if (RGB[4*idx+3] == 0.0)
		mask[idx] = 10;
	else
		mask[idx] = 11;

}
__global__ void InitMaskKernel (unsigned char *mask, float *RGB, int n, int m) {
	InitMaskProcess(mask, RGB, n, m);
}

__device__ __forceinline__ void ProjectMaskProcess (float *pose, float *calib, unsigned char *mask, unsigned char *mask_swap, float *Vmap, int n, int m) {
	int i = threadIdx.x + blockIdx.x * THREAD_SIZE_X;
    int j = threadIdx.y + blockIdx.y * THREAD_SIZE_Y;
    int idx = i*m + j;

	if (i > n-1 || j > m-1)
        return;

	if (Vmap[3*idx] == 0.0 && Vmap[3*idx+1] == 0.0 && Vmap[3*idx+2] == 0.0) {
		mask_swap[idx] = 0;
		return;
	}

	float x, y, z;
	x = pose[0]*Vmap[3*idx] + pose[4]*Vmap[3*idx+1] + pose[8]*Vmap[3*idx+2] + pose[12];
	y = pose[1]*Vmap[3*idx] + pose[5]*Vmap[3*idx+1] + pose[9]*Vmap[3*idx+2] + pose[13];
	z = pose[2]*Vmap[3*idx] + pose[6]*Vmap[3*idx+1] + pose[10]*Vmap[3*idx+2] + pose[14];

	int p_indx[2];
	p_indx[0] = __float2int_rn((x/fabs(z))*calib[0] + calib[2]); 
	p_indx[1] = __float2int_rn((y/fabs(z))*calib[1] + calib[3]); 

	if (p_indx[1] > n-1 || p_indx[0] > m-1) {
		mask_swap[idx] = 0;
        return;
	}

	int indx_proj = p_indx[1]*m + p_indx[0];
	mask_swap[idx] = mask[indx_proj];

}
__global__ void ProjectMaskKernel (float *pose, float *calib, unsigned char *mask, unsigned char *mask_swap, float *VMap, int n, int m) {
	ProjectMaskProcess(pose, calib, mask, mask_swap, VMap, n, m);
}

/*** Device functions that use OpenCv ***/
__device__ __forceinline__ void CpyProcess (cv::gpu::DevMem2D_<float> mat, float *rgb, int n, int m) {
	int i = threadIdx.x + blockIdx.x * THREAD_SIZE_X;
    int j = threadIdx.y + blockIdx.y * THREAD_SIZE_Y;
    int idx = i*m + j;

	if (i > n-1 || j > m-1)
        return;

	mat(i,j) = (rgb[3*idx+2] + rgb[3*idx+1] + rgb[3*idx])/3.0;
}
__global__ void CpyKernel (cv::gpu::DevMem2D_<float> mat, float *rgb, int n, int m) {
	CpyProcess(mat, rgb, n, m);
}

__device__ __forceinline__ void CpyProcessN (cv::gpu::DevMem2D_<unsigned char> mat, float *Nmap, int n, int m, int chnl) {
	int i = threadIdx.x + blockIdx.x * THREAD_SIZE_X;
    int j = threadIdx.y + blockIdx.y * THREAD_SIZE_Y;

	if (i > n-1 || j > m-1)
        return;

	mat(i,j) = unsigned char(125.0*(1.0+Nmap[3*(i*m + j)+ chnl])); 
}
__global__ void CpyKernelN (cv::gpu::DevMem2D_<unsigned char> mat, float *Nmap, int n, int m, int chnl) {
	CpyProcessN(mat, Nmap, n, m, chnl);
}

__device__ __forceinline__ void CpyProcessCChar (cv::gpu::DevMem2D_<unsigned char> mat, unsigned char *rgb, int n, int m) {
	int i = threadIdx.x + blockIdx.x * THREAD_SIZE_X;
    int j = threadIdx.y + blockIdx.y * THREAD_SIZE_Y;

	if (i > n-1 || j > m-1)
        return;

	mat(i,j) = rgb[i*m + j] > 10 ? 255 : 0;
}
__global__ void CpyKernelCChar (cv::gpu::DevMem2D_<unsigned char> mat, unsigned char *rgb, int n, int m) {
	CpyProcessCChar(mat, rgb, n, m);
}

__device__ __forceinline__ void CpyProcessChar3 (cv::gpu::DevMem2D_<uchar[3]> mat, float *rgb, int n, int m) {
	int i = threadIdx.x + blockIdx.x * THREAD_SIZE_X;
	int j = threadIdx.y + blockIdx.y * THREAD_SIZE_Y;
	int idx = i*m + j;

	if (i > n-1 || j > m-1)
		return;

	rgb[3*idx] =  float(mat(n-i-1, j)[2])/255.0;// Normalisation : pour OpenGL couleurs comprise entre 0 et 1; puis passage de rgb à bgr; puis changement de repère
	rgb[3*idx+1] = float(mat(n-i-1, j)[1])/255.0;
	rgb[3*idx+2] = float(mat(n-i-1, j)[0])/255.0;
}
__global__ void CpyKernelChar3 (cv::gpu::DevMem2D_<uchar[3]>mat, float *rgb, int n, int m) {
	CpyProcessChar3(mat, rgb, n, m);
}

__device__ __forceinline__ void SumProcess (cv::gpu::DevMem2D_<unsigned char> Out, cv::gpu::DevMem2D_<unsigned char> InR, cv::gpu::DevMem2D_<unsigned char> InG, cv::gpu::DevMem2D_<unsigned char> InB, int n, int m) {
	int i = threadIdx.x + blockIdx.x * THREAD_SIZE_X;
    int j = threadIdx.y + blockIdx.y * THREAD_SIZE_Y;

	if (i > n-1 || j > m-1)
        return;

	Out(i,j) = unsigned char((float(InR(i,j)) + float(InG(i,j)) + float(InB(i,j)))/3.0);
}
__global__ void  SumKernel(cv::gpu::DevMem2D_<unsigned char> Out, cv::gpu::DevMem2D_<unsigned char> InR, cv::gpu::DevMem2D_<unsigned char> InG, cv::gpu::DevMem2D_<unsigned char> InB, int n, int m) {
	SumProcess(Out, InR, InG, InB, n, m);
}

__device__ __forceinline__ void ThreshProcess (cv::gpu::DevMem2D_<unsigned char> Out, cv::gpu::DevMem2D_<unsigned char> In, unsigned char thresh, float *NMap, int n, int m) {
	int i = threadIdx.x + blockIdx.x * THREAD_SIZE_X;
    int j = threadIdx.y + blockIdx.y * THREAD_SIZE_Y;
    int idx = i*m + j;

	if (i > n-1 || j > m-1)
        return;

	float norm_n = NMap[3*idx]*NMap[3*idx] + NMap[3*idx+1]*NMap[3*idx+1] + NMap[3*idx+2]*NMap[3*idx+2];
	Out(i,j) = (In(i,j) > thresh || norm_n == 0.0) ? 0 : 255;
}
__global__ void  ThreshKernel(cv::gpu::DevMem2D_<unsigned char> Out, cv::gpu::DevMem2D_<unsigned char> In, unsigned char thresh, float *NMap, int n, int m) {
	ThreshProcess(Out, In, thresh, NMap, n, m);
}

__device__ __forceinline__ void AddZBUFFOffProcess (cv::gpu::DevMem2D_<unsigned short> depth, float *rgb_in, float *rgb, float *calib, int n, int m) {
	int i = threadIdx.x + blockIdx.x * THREAD_SIZE_X;
    int j = threadIdx.y + blockIdx.y * THREAD_SIZE_Y;
    int idx = i*m + j;

	if (i > n-1 || j > m-1)
        return;

    float ds = calib[9];  //# distortion
    float factor = calib[10];  //# factor

	float z = (ds/factor) * float(depth(n-i-1, j));
	rgb[4*idx] = rgb_in[3*idx];
	rgb[4*idx+1] = rgb_in[3*idx+1];
	rgb[4*idx+2] = rgb_in[3*idx+2];
	rgb[4*idx+3] = (z > MAX_DEPTH || z < 0.0) ? 0.0 : z/MAX_DEPTH;
}
__global__ void  AddZBUFFOffKernel(cv::gpu::DevMem2D_<unsigned short> depth, float *rgb_in, float *rgb, float *calib, int n, int m) {
	AddZBUFFOffProcess(depth, rgb_in, rgb, calib, n, m);
}

__device__ __forceinline__ void AddZBUFFKinectProcess (unsigned short *depth, float *rgb_in, float *rgb, float *calib, int n, int m) {
	int i = threadIdx.x + blockIdx.x * THREAD_SIZE_X;
    int j = threadIdx.y + blockIdx.y * THREAD_SIZE_Y;
    int idx = i*m + j;

	if (i > n-1 || j > m-1)
        return;

    float ds = calib[9];  //# distortion
    float factor = calib[10];  //# factor

	float z = (ds/factor) * float(depth[idx]);
	rgb[4*idx] = rgb_in[3*idx];
	rgb[4*idx+1] = rgb_in[3*idx+1];
	rgb[4*idx+2] = rgb_in[3*idx+2];
	rgb[4*idx+3] = (z > MAX_DEPTH || z < 0.0) ? 0.0 : z/MAX_DEPTH;
}
__global__ void  AddZBUFFKinectKernel(unsigned short *depth, float *rgb_in, float *rgb, float *calib, int n, int m) {
	AddZBUFFKinectProcess(depth, rgb_in, rgb, calib, n, m);
}


//////////////////////////////////////////
///******* Function definitions *********/
//////////////////////////////////////////

void VertexMapKinect(float *VMap, cv::gpu::DevMem2D_<unsigned short> depth, int n, int m) {
	
	dim3 dimBlock(THREAD_SIZE_X, THREAD_SIZE_Y);
	dim3 dimGrid (1, 1, 1);
	dimGrid.x = divUp (n, dimBlock.x);
	dimGrid.y = divUp (m, dimBlock.y);

	VertexMapKinectKernel<<<dimGrid, dimBlock>>>(depth, VMap, calib_dev, n, m);

	checkCudaErrors( cudaDeviceSynchronize() );// erreur ici
    
	return;
}

void VertexMapGL(float *VMap, float *RGB, int n, int m) {
	
	dim3 dimBlock(THREAD_SIZE_X, THREAD_SIZE_Y);
	dim3 dimGrid (1, 1, 1);
	dimGrid.x = divUp (n, dimBlock.x);
	dimGrid.y = divUp (m, dimBlock.y);

	VertexMapGLKernel<<<dimGrid, dimBlock>>>(RGB, VMap, calib_dev, n, m);

	checkCudaErrors( cudaDeviceSynchronize() );
    
	return;
}

void MapData2VBO_cu(BYTE *d_colorFrame, USHORT *d_depthFrame, LONG *d_colorCoord, float *VMap, float *RGB, int n, int m) {
	dim3 dimBlock(THREAD_SIZE_X, THREAD_SIZE_Y);
	dim3 dimGrid (1, 1, 1);
	dimGrid.x = divUp (n, dimBlock.x);
	dimGrid.y = divUp (m, dimBlock.y);

	MapData2VBOKernel<<<dimGrid, dimBlock>>>(d_colorFrame, d_depthFrame, d_colorCoord, VMap, RGB, calib_dev, n, m);

	checkCudaErrors( cudaDeviceSynchronize() );
    
	return;
}

void ComputeNormal(float *NMap, float *VMap, int n, int m, bool inverse) {
	
	dim3 dimBlock(THREAD_SIZE_L_X, THREAD_SIZE_L_Y);
	dim3 dimGrid (1, 1, 1);
	dimGrid.x = divUp (n, dimBlock.x);
	dimGrid.y = divUp (m, dimBlock.y);

	NormalKernel<<<dimGrid, dimBlock>>>(NMap, VMap, n, m, inverse);

	checkCudaErrors( cudaDeviceSynchronize() );// erreur ici
    
	return;
}

void ReadFrame_cu(float *Vmap, float *RGBD, float *NMap, cudaArray* Array, int n, int m) {

	cudaChannelFormatDesc desc = cudaCreateChannelDesc<float4>();

	checkCudaErrors( cudaBindTextureToArray( &texRef, Array, & desc) );

	dim3 dimBlock(THREAD_SIZE_X, THREAD_SIZE_Y);
	dim3 dimGrid (1, 1, 1);
	dimGrid.x = divUp (n, dimBlock.x); // !! ne pas inverser n et m !!
	dimGrid.y = divUp (m, dimBlock.y);

	texRef.filterMode = cudaFilterModeLinear;

	texRef.normalized = false;
	
	checkCudaErrors( cudaMemset(RGBD, 0, 4*n*m*sizeof(float)) );
	ReadKernel<<<dimGrid, dimBlock>>>(Vmap, RGBD, calib_dev, n, m);
	checkCudaErrors( cudaDeviceSynchronize() );
		
	checkCudaErrors( cudaUnbindTexture( &texRef) );
	return;

}

void TransformVertexMap(float *VMap, float *RGBD, int n, int m) {
	
	dim3 dimBlock(THREAD_SIZE_X, THREAD_SIZE_Y);
	dim3 dimGrid (1, 1, 1);
	dimGrid.x = divUp (n, dimBlock.x);
	dimGrid.y = divUp (m, dimBlock.y);

	TransformVertexMapKernel<<<dimGrid, dimBlock>>>(pose_dev, VMap, RGBD, n, m);

	checkCudaErrors( cudaDeviceSynchronize() );
    
	return;
}

void QuadTrimFrame(float *VMap, unsigned int *indices_dev, int n, int m, float thresh) {

	dim3 dimBlock(THREAD_SIZE_L_X, THREAD_SIZE_L_Y);
	dim3 dimGrid (1, 1, 1);
	dimGrid.x = divUp (n-1, dimBlock.x);
	dimGrid.y = divUp (m-1, dimBlock.y);

	QuadTrimFrameKernel<<<dimGrid, dimBlock>>>(indices_dev, VMap, n, m, thresh);

	checkCudaErrors( cudaDeviceSynchronize() );
	
	return;
}

void MergeDepthOff(float *RGB_predicted, cv::gpu::DevMem2D_<unsigned short> depth, float *RGB, unsigned char *mask, int n, int m) {

	dim3 dimBlock(THREAD_SIZE_L_X, THREAD_SIZE_L_Y);
	dim3 dimGrid (1, 1, 1);
	dimGrid.x = divUp (n, dimBlock.x);
	dimGrid.y = divUp (m, dimBlock.y);
	
    MergeDepthOffKernel<<<dimGrid, dimBlock>>>(RGB_predicted, depth, RGB, mask, calib_dev, n, m);
	
	checkCudaErrors( cudaDeviceSynchronize() );
	
	return;
}

void MergeDepthKinect(float *RGB_predicted, unsigned short *depth, float *RGB, unsigned char *mask, int n, int m) {

	dim3 dimBlock(THREAD_SIZE_L_X, THREAD_SIZE_L_Y);
	dim3 dimGrid (1, 1, 1);
	dimGrid.x = divUp (n, dimBlock.x);
	dimGrid.y = divUp (m, dimBlock.y);
	
    MergeDepthKinectKernel<<<dimGrid, dimBlock>>>(RGB_predicted, depth, RGB, mask, calib_dev, n, m);
	
	checkCudaErrors( cudaDeviceSynchronize() );
	
	return;
}

void InitMask_cu(unsigned char *mask, float *RGB, int n, int m) {
	dim3 dimBlock(THREAD_SIZE_X, THREAD_SIZE_Y);
	dim3 dimGrid (1, 1, 1);
	dimGrid.x = divUp (n, dimBlock.x);
	dimGrid.y = divUp (m, dimBlock.y);
	
    InitMaskKernel<<<dimGrid, dimBlock>>>(mask, RGB, n, m);
	
	checkCudaErrors( cudaDeviceSynchronize() );
	
	return;
}

void ProjectMask_cu(unsigned char *mask, unsigned char *mask_swap, float *VMap, int n, int m)  {
	dim3 dimBlock(THREAD_SIZE_X, THREAD_SIZE_Y);
	dim3 dimGrid (1, 1, 1);
	dimGrid.x = divUp (n, dimBlock.x);
	dimGrid.y = divUp (m, dimBlock.y);
	
    ProjectMaskKernel<<<dimGrid, dimBlock>>>(pose_dev, calib_dev, mask, mask_swap, VMap, n, m);
	
	checkCudaErrors( cudaDeviceSynchronize() );
	
	return;
}

/*********************************/
/*** Function used with opencv ***/
/*********************************/

void gpu_cpy(cv::gpu::DevMem2D_<float> mat, float *rgb, int n, int m) {
	dim3 dimBlock(THREAD_SIZE_X, THREAD_SIZE_Y);
	dim3 dimGrid (1, 1, 1);
	dimGrid.x = divUp (n, dimBlock.x);
	dimGrid.y = divUp (m, dimBlock.y);
	
    CpyKernel<<<dimGrid, dimBlock>>>(mat, rgb, n, m);
	
	checkCudaErrors( cudaDeviceSynchronize() );
	
	return;
}

void gpu_cpyN(cv::gpu::DevMem2D_<unsigned char> mat, float *NMap, int n, int m, int chnl) {
	dim3 dimBlock(THREAD_SIZE_X, THREAD_SIZE_Y);
	dim3 dimGrid (1, 1, 1);
	dimGrid.x = divUp (n, dimBlock.x);
	dimGrid.y = divUp (m, dimBlock.y);
	
    CpyKernelN<<<dimGrid, dimBlock>>>(mat, NMap, n, m, chnl);
	
	checkCudaErrors( cudaDeviceSynchronize() );
	
	return;
}

void gpu_cpyC_char(cv::gpu::DevMem2D_<unsigned char> mat, unsigned char *rgb, int n, int m){
	dim3 dimBlock(THREAD_SIZE_X, THREAD_SIZE_Y);
	dim3 dimGrid (1, 1, 1);
	dimGrid.x = divUp (n, dimBlock.x);
	dimGrid.y = divUp (m, dimBlock.y);
	
    CpyKernelCChar<<<dimGrid, dimBlock>>>(mat, rgb, n, m);
	
	checkCudaErrors( cudaDeviceSynchronize() );
	
	return;
}

void gpu_cpy_char3(cv::gpu::DevMem2D_<uchar[3]>mat, float *rgb, int n, int m){
	dim3 dimBlock(THREAD_SIZE_X, THREAD_SIZE_Y);
	dim3 dimGrid (1, 1, 1);
	dimGrid.x = divUp (n, dimBlock.x);
	dimGrid.y = divUp (m, dimBlock.y);
	
    CpyKernelChar3<<<dimGrid, dimBlock>>>(mat, rgb, n, m);
	
	checkCudaErrors( cudaDeviceSynchronize() );
	
	return;
}

void gpu_Sum(cv::gpu::DevMem2D_<unsigned char> Out, cv::gpu::DevMem2D_<unsigned char> InR, cv::gpu::DevMem2D_<unsigned char> InG, cv::gpu::DevMem2D_<unsigned char> InB, int n, int m){
	dim3 dimBlock(THREAD_SIZE_X, THREAD_SIZE_Y);
	dim3 dimGrid (1, 1, 1);
	dimGrid.x = divUp (n, dimBlock.x);
	dimGrid.y = divUp (m, dimBlock.y);
	
    SumKernel<<<dimGrid, dimBlock>>>(Out, InR, InG, InB, n, m);
	
	checkCudaErrors( cudaDeviceSynchronize() );
	
	return;
}

void gpu_Thresh(cv::gpu::DevMem2D_<unsigned char> Out, cv::gpu::DevMem2D_<unsigned char> In, unsigned char thresh, float *NMap, int n, int m) {
	dim3 dimBlock(THREAD_SIZE_X, THREAD_SIZE_Y);
	dim3 dimGrid (1, 1, 1);
	dimGrid.x = divUp (n, dimBlock.x);
	dimGrid.y = divUp (m, dimBlock.y);
	
    ThreshKernel<<<dimGrid, dimBlock>>>(Out, In, thresh, NMap, n, m);
	
	checkCudaErrors( cudaDeviceSynchronize() );
	
	return;
}

void gpu_add_zbuffOff(cv::gpu::DevMem2D_<unsigned short> depth, float *rgb_in, float *rgb, int n, int m){
	dim3 dimBlock(THREAD_SIZE_X, THREAD_SIZE_Y);
	dim3 dimGrid (1, 1, 1);
	dimGrid.x = divUp (n, dimBlock.x);
	dimGrid.y = divUp (m, dimBlock.y);
	
    AddZBUFFOffKernel<<<dimGrid, dimBlock>>>(depth, rgb_in, rgb, calib_dev, n, m);
	
	checkCudaErrors( cudaDeviceSynchronize() );
	
	return;
}

void gpu_add_zbuffKinect(unsigned short *depth, float *rgb_in, float *rgb, int n, int m){
	dim3 dimBlock(THREAD_SIZE_X, THREAD_SIZE_Y);
	dim3 dimGrid (1, 1, 1);
	dimGrid.x = divUp (n, dimBlock.x);
	dimGrid.y = divUp (m, dimBlock.y);
	
    AddZBUFFKinectKernel<<<dimGrid, dimBlock>>>(depth, rgb_in, rgb, calib_dev, n, m);
	
	checkCudaErrors( cudaDeviceSynchronize() );
	
	return;
}
