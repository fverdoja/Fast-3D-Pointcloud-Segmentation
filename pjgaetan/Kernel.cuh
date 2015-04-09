#ifndef __KERNEL_H
#define __KERNEL_H

#include "cudaTypes.cuh"

/***** Global variable accessor ******/
void SetCalibrationMatrix(float *Calib_CPU);
void FreeCalibrationMatrix();
void AllocPoseMatrix();
void SetPoseMatrix(float *pose_CPU);
void FreePoseMatrix();

/**** Function definitions ****/
void VertexMapKinect(float *VMap, cv::gpu::DevMem2D_<unsigned short> depth, int n, int m);
void VertexMapGL(float *VMap, float *RGB, int n, int m);
void MapData2VBO_cu(BYTE *d_colorFrame, USHORT *d_depthFrame, double *d_colorCoord, float *VMap, float *RGB, int n, int m);
void ComputeNormal(float *NMap, float *VMap, int n, int m, bool inverse = false);
void ReadFrame_cu(float *Vmap, float *RGBD, float *NMap, cudaArray* Array, int n, int m);
void TransformVertexMap(float *VMap, float *RGBD, int n, int m);
void QuadTrimFrame(float *VMap, unsigned int *indices_dev, int n, int m, float thresh);
void MergeDepthOff(float *RGB_predicted, cv::gpu::DevMem2D_<unsigned short> depth, float *RGB, unsigned char *mask, int n, int m);
void MergeDepthKinect(float *RGB_predicted, unsigned short *depth, float *RGB, unsigned char *mask, int n, int m);
void InitMask_cu(unsigned char *mask, float *RGB, int n, int m);
void ProjectMask_cu(unsigned char *mask, unsigned char *mask_swap, float *VMap, int n, int m);


/*** Function used with opencv ***/

void gpu_cpy(cv::gpu::DevMem2D_<float> mat, float *rgb, int n, int m); 
void gpu_cpyN(cv::gpu::DevMem2D_<unsigned char> mat, float *NMap, int n, int m, int chnl); 
void gpu_cpyC_char(cv::gpu::DevMem2D_<unsigned char> mat, unsigned char *rgb, int n, int m); 
void gpu_cpy_char3(cv::gpu::DevMem2D_<uchar[3]> mat, float *rgb, int n, int m); 
void gpu_Sum(cv::gpu::DevMem2D_<unsigned char> Out, cv::gpu::DevMem2D_<unsigned char> InR, cv::gpu::DevMem2D_<unsigned char> InG, cv::gpu::DevMem2D_<unsigned char> InB, int n, int m); 
void gpu_Thresh(cv::gpu::DevMem2D_<unsigned char> Out, cv::gpu::DevMem2D_<unsigned char> In, unsigned char thresh, float *NMap, int n, int m); 
void gpu_add_zbuffOff(cv::gpu::DevMem2D_<unsigned short> depth, float *rgb_in, float *rgb, int n, int m);
void gpu_add_zbuffKinect(unsigned short *depth, float *rgb_in, float *rgb, int n, int m);


#endif
