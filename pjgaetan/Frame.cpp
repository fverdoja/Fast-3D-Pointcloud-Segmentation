//#include "stdafx.h" //visual studio specific
#include "Frame.h"

#define extern VERBOSE;

/***************************************************************************************/
/*************************** Annex Functions Used only in frame.cpp ********************/
/***************************************************************************************/

void writeMatToFile16S(cv::Mat& m, const char* filename)
{
  ofstream fout(filename);

  if(!fout) {
      cout<<"File Not Opened"<<endl;  return;
  }

  for(int i=0; i<m.rows; i++) {
    for(int j=0; j<m.cols; j++) {
      fout<<m.at<short>(i,j)<<"\t";
    }
    fout<<endl;
  }

  fout.close();
}

void writeMatToFile8U(cv::Mat& m, const char* filename)
{
  ofstream fout(filename);

  if(!fout) {
    cout<<"File Not Opened"<<endl;  return;
  }

  for(int i=0; i<m.rows; i++) {
    for(int j=0; j<m.cols; j++) {
	fout<<int(m.at<unsigned char>(i,j))<<"\t";
    }
    fout<<endl;
  }
  fout.close();
}


string type2str(int type) {
  string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U";   break;
    case CV_8S:  r = "8S";   break;
    case CV_16U: r = "16U";  break;
    case CV_16S: r = "16S";  break;
    case CV_32S: r = "32S";  break;
    case CV_32F: r = "32F";  break;
    case CV_64F: r = "64F";  break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}

void drawHist(cv::Mat& m, cv::Mat& Mask){
  
  int histSize = 256; //from 0 to 255


  /// Set the ranges ( for B,G,R) )
  float range[] = { 0, 256 } ; //the upper boundary is exclusive
  const float* histRange = { range };

  bool uniform = true; bool accumulate = false;

  cv::Mat hist;
  /// Compute the histograms:
  cv::calcHist(&m, 1, 0, Mask, hist, 1, &histSize, &histRange, uniform, accumulate );

  // Draw the histograms for R, G and B
  int hist_w = 512; int hist_h = 400;
  int bin_w = cvRound( (double) hist_w/histSize );

  Mat histImage( hist_h, hist_w, CV_8UC1, Scalar( 0,0,0) );

  /// Normalize the result to [ 0, histImage.rows ]
  normalize(hist, hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );

  /// Draw for each channel
  for( int i = 1; i < histSize; i++ )
  {
  line( histImage, Point( bin_w*(i-1), hist_h - cvRound(hist.at<float>(i-1)) ) ,
		Point( bin_w*(i), hist_h - cvRound(hist.at<float>(i)) ),
		Scalar( 255, 255, 255), 2, 8, 0  );
  }

  namedWindow("calcHist Demo", CV_WINDOW_AUTOSIZE );
  imshow("calcHist Demo", histImage );

}

void simplifyCloud(std::vector<Point3D>& points, float cell_size)
{
   // float start_tic = ofGetElapsedTimeMillis();
    
    // simplification by clustering using erase-remove idiom
    // cell_size = 0.001;
    points.erase(CGAL::grid_simplify_point_set(points.begin(), points.end(), cell_size),
                 points.end());
    // Optional: after erase(), use Scott Meyer's "swap trick" to trim excess capacity
    vector<Point3D>(points).swap(points);
    
  // ofLogVerbose("simplifyCloud") << ofGetElapsedTimeMillis()-start_tic << "[msec]";

}




/***************************************************************************************/
/*************************** Methods for the class InputFrame **************************/
/***************************************************************************************/

void InputFrame::Draw(bool color) {
	
  /* on passe en mode VBO */
  glBindBuffer(GL_ARRAY_BUFFER, _frame_buf);

  glVertexPointer(3, GL_FLOAT, 0, BUFFER_OFFSET(0));
  glNormalPointer(GL_FLOAT, 0, BUFFER_OFFSET(_n*_m*3*sizeof(float)));// on vient stoquer les normales à la suite du buffer
  if (color)
    glColorPointer(3, GL_FLOAT , 0, BUFFER_OFFSET(_n*_m*3*sizeof(float)+_n*_m*3*sizeof(float)));

  /* activation des tableaux de donnees */
  glEnableClientState(GL_VERTEX_ARRAY);
  glEnableClientState(GL_NORMAL_ARRAY);
  if (color)
    glEnableClientState(GL_COLOR_ARRAY);

  /* rendu points */
  glDrawArrays(GL_POINTS, 0, _n*_m );

  if (color)
    glDisableClientState(GL_COLOR_ARRAY);
  glDisableClientState(GL_NORMAL_ARRAY);
  glDisableClientState(GL_VERTEX_ARRAY);

  glBindBuffer(GL_ARRAY_BUFFER, 0);

}

/***************************************************************************************/
/*************************** Methods for the class OffLineFrame **************************/
/***************************************************************************************/

void OffLineFrame::LoadFrame(string filename_depth, string filename_color) {
  
  cv::Mat depth_test;//variables CPU temporaires 
  cv::Mat color_test;

  depth_test = cv::imread(filename_depth, CV_LOAD_IMAGE_UNCHANGED);
  color_test = cv::imread(filename_color, CV_LOAD_IMAGE_UNCHANGED);
  
  //debug: print name & size of input data
  {
    puts("Input frames:");
    printf("\"%s\" (size", filename_depth.c_str());
    for(int i=0; i<depth_test.dims;++i){
      printf(" %u",depth_test.size[i]);
    }
    puts(")");
    printf("\"%s\" (size", filename_color.c_str());
    for(int i=0; i<color_test.dims;++i){
      printf(" %u",color_test.size[i]);
    }
    puts(")");
  }
  
  /*cv::Mat tmp;
  _depth_test.convertTo(tmp, CV_32FC1);
  cv::Mat out;
  bilateralFilter(tmp, out, 5, 100.0, 100.0);
  out.convertTo(_depth_test, CV_16UC1);*/
  this->_depth_dev_test.upload(depth_test); //on charge l'image dans le GPU dans l'attribut _depth_dev_test de notre frame
  depth_test.release();// On libere l'image du CPU

  this->_color_dev_test.upload(color_test); //on charge l'image dans le GPU
  color_test.release();// On libere l'image du CPU
  
  checkCudaErrors( cudaGraphicsMapResources (1, _Resources_t) );// On reserve l'utilisation du GPU à cuda
  // Copy RGBD data 
  gpu_cpy_char3(_color_dev_test, _RGB_dev, _n,  _m);// On met la matrice color dans le VBO _RGB_dev
  // Compute Vertex position
//	checkCudaErrors( cudaMemset(_VMap_dev, 0, 3*_n*_m*sizeof(float)) );
  VertexMapKinect(_VMap_dev, _depth_dev_test, _n, _m);
  // Compute Normal orientation
//	checkCudaErrors( cudaMemset(_NMap_dev, 0, 3*_n*_m*sizeof(float)) );
  puts("Computing normals...");
  ComputeNormal(_NMap_dev, _VMap_dev, this->_n, this->_m, true);

  // Release VBO data 
  checkCudaErrors( cudaGraphicsUnmapResources (1, _Resources_t) );// On rend l'utilisation du GPU a la carte graphique //erreur ici

  return;
}


/***************************************************************************************/
/*************************** Methods for the class KinectFrame *************************/
/***************************************************************************************/

void KinectFrame::LoadKinectData(BYTE* h_colorFrame, USHORT* h_depthFrame, LONG* h_colorCoord) {

  /**** copy data from host memory to device memory location ****/
  // Copy color frame
  checkCudaErrors(cudaMemcpy(d_colorFrame, h_colorFrame, sizeof(BYTE)* 4 * _n * _m, cudaMemcpyHostToDevice));
  //memcpy(_rgb_char, h_colorFrame, sizeof(BYTE)* 4 * _n * _m);

  // Copy depth frame
  checkCudaErrors(cudaMemcpy(d_depthFrame, h_depthFrame, sizeof(USHORT) * _n * _m, cudaMemcpyHostToDevice));

  // copy color coordinates
  checkCudaErrors(cudaMemcpy(d_colorCoord, h_colorCoord, sizeof(LONG)* 2 * _n * _m, cudaMemcpyHostToDevice));

  checkCudaErrors( cudaGraphicsMapResources (1, _Resources_t) );
  
  checkCudaErrors( cudaMemset(_VMap_dev,0, 3*_n * _m * sizeof(float)) );
  checkCudaErrors( cudaMemset(_NMap_dev,0, 3*_n * _m * sizeof(float)) );
  checkCudaErrors( cudaMemset(_RGB_dev, 0, 3*_n * _m * sizeof(float)) );

  MapData2VBO_cu(d_colorFrame, d_depthFrame, d_colorCoord, _VMap_dev, _RGB_dev, _n, _m);

  checkCudaErrors( cudaMemset(_NMap_dev, 0, 3*_n*_m*sizeof(float)) );
  ComputeNormal(_NMap_dev, _VMap_dev, _n, _m, true);
  
  checkCudaErrors( cudaGraphicsUnmapResources (1, _Resources_t) );

}


/***************************************************************************************/
/*************************** Methods for the class PredictedFrame **********************/
/***************************************************************************************/

void PredictedFrame::print(char *filename) {

	float *VMap_dev = getVMap();
	float *NMap_dev = getNMap();
	float *RGB_dev = getRGB();

	float *VMap = (float *) malloc(3*_n*_m*sizeof(float));
	cudaMemcpy(VMap, (float *) VMap_dev,  3*_n * _m * sizeof(float), cudaMemcpyDeviceToHost);
	float *NMap = (float *) malloc(3*_n*_m*sizeof(float));
	cudaMemcpy(NMap, (float *) NMap_dev,  3*_n * _m * sizeof(float), cudaMemcpyDeviceToHost);
	cudaMemcpy(_rgb, (float *) RGB_dev,  4*_n * _m * sizeof(float), cudaMemcpyDeviceToHost);

	int nbVertex = 0;
	for (int i = 0; i < _n; i++) {
		for (int j = 0; j < _m; j++) {
			if (VMap[3*(i*_m+j)+2] != 0.0)
				nbVertex++;
		}
	}

	//
    //  Open the input file in "read text" mode.
    //
	ofstream  filestr;

	filestr.open (filename, fstream::out);

	if (!filestr.is_open()) {
		cout << "Could not open " << filename << endl;
		return;
	}

	filestr << "ply\n";
	filestr << "format ascii 1.0\n";
    
    //
    //  Write the header.
    //
	filestr << "comment File created by Diego Thomas\n";
	filestr << "element vertex " << nbVertex << "\n";
	filestr << "property float x\n";
	filestr << "property float y\n";
	filestr << "property float z\n";
	filestr << "property float nx\n";
	filestr << "property float ny\n";
	filestr << "property float nz\n";
	filestr << "property uchar red\n";
	filestr << "property uchar green\n";
	filestr << "property uchar blue\n";
	filestr << "property uchar alpha\n";
    int nbFace = 0;
	filestr << "element face " << nbFace << "\n";
	filestr << "property list uchar int vertex_indices\n";
	filestr << "end_header\n";
    
    //
    //  Write data.
    //
  for (int i = 0; i < _n; i++) {
    for (int j = 0; j < _m; j++) {
      if (VMap[3*(i*_m+j)+2] != 0.0)
      filestr << VMap[3*(i*_m+j)] << " " << VMap[3*(i*_m+j)+1] << " " << VMap[3*(i*_m+j)+2] << " "
      << NMap[3*(i*_m+j)] << " " << NMap[3*(i*_m+j)+1] << " " << NMap[3*(i*_m+j)+2] << " "
      //<< 255 << " " << 255 << " " << 255 << " " << "255\n";
      << static_cast<int>(_rgb[4*(i*_m+j)]*255.0) << " " << static_cast<int>(_rgb[4*(i*_m+j)+1]*255.0) << " " << static_cast<int>(_rgb[4*(i*_m+j)+2]*255.0) << " " << "255\n";
    }
  }

  filestr.close();
  free(VMap);
  free(NMap);
}

void PredictedFrame::save(char *filename, int indx) {
  char destfilename[100];

  cv::Mat img(_n, _m, CV_16UC3);
  int i,j,k;
  for (i=0,k=_n-1;i<_n;i++,k--) {
    for (j=0;j<_m;j++) {
      //cout << _depth[(i*_m + j)] << endl;
      img.at<cv::Vec3w>(k,j)[2] = (unsigned short)(_depth[(i*_m + j)]*5000.0);
      img.at<cv::Vec3w>(k,j)[1] = (unsigned short)(_depth[(i*_m + j)]*5000.0);
      img.at<cv::Vec3w>(k,j)[0] = (unsigned short)(_depth[(i*_m + j)]*5000.0);
      //cout << img.at<cv::Vec3w>(k,j)[0] << endl;
    }
  }

  
  sprintf(destfilename, "%s\\Depth%d.tiff", filename, indx);
  if (! cv::imwrite(destfilename, img) )
    cout << "error print Depth" << endl;

  for (i=0,k=_n-1;i<_n;i++,k--) {
    for (j=0;j<_m;j++) {
      img.at<cv::Vec3w>(k,j)[2] = 200 * (unsigned short)(_rgb[4*(i*_m + j)]   * 255.0);
      img.at<cv::Vec3w>(k,j)[1] = 200 * (unsigned short)(_rgb[4*(i*_m + j)+1] * 255.0);
      img.at<cv::Vec3w>(k,j)[0] = 200 * (unsigned short)(_rgb[4*(i*_m + j)+2] * 255.0);
    }
  }

  sprintf(destfilename, "%s\\RGB%d.tiff", filename, indx);
  if (! cv::imwrite(destfilename, img) )
    cout << "error print RGB" << endl;

  img.~Mat();
}

void PredictedFrame::ReadFrame(cudaGraphicsResource_t *Resources) {
	
	cudaGraphicsResource_t Resources_tot [2];
	Resources_tot [0] = Resources[0];
	Resources_tot [1] = _Resources_t[0];

	checkCudaErrors( cudaGraphicsMapResources ( 2, Resources_tot) );

	cudaArray* My_Array;
	checkCudaErrors( cudaGraphicsSubResourceGetMappedArray( &My_Array, Resources[0], 0, 0) );		
	ReadFrame_cu(_VMap_dev, _RGB_dev, _NMap_dev, My_Array, _n, _m);

	checkCudaErrors( cudaMemset(_NMap_dev, 0, 3*_n*_m*sizeof(float)) );
	ComputeNormal(_NMap_dev, _VMap_dev, _n, _m, true);
		
	checkCudaErrors( cudaGraphicsUnmapResources (2,  Resources_tot) );		
}

void PredictedFrame::Draw(bool color, bool quad) {
	
	/* on passe en mode VBO */
    glBindBuffer(GL_ARRAY_BUFFER, _frame_buf);

	glVertexPointer(3, GL_FLOAT, 0, BUFFER_OFFSET(0));
	glNormalPointer(GL_FLOAT, 0, BUFFER_OFFSET(_n*_m*3*sizeof(float)));
	if (color)
		glColorPointer(4, GL_FLOAT , 0, BUFFER_OFFSET(_n*_m*3*sizeof(float)+_n*_m*3*sizeof(float)));

	/* activation des tableaux de donnees */
    glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_NORMAL_ARRAY);
	if (color)
		glEnableClientState(GL_COLOR_ARRAY);

	if (!quad) {
		/* rendu points */
		glDrawArrays(GL_POINTS, 0, _n*_m );
	} else {
		/* rendu indices */
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _index_buf);
		glDrawElements(GL_QUADS, 4*(_n-1)*(_m-1), GL_UNSIGNED_INT, BUFFER_OFFSET(0));
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	}

	if (color)
		glDisableClientState(GL_COLOR_ARRAY);
	glDisableClientState(GL_NORMAL_ARRAY);
	glDisableClientState(GL_VERTEX_ARRAY);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void PredictedFrame::Transform(float *pose, bool quad) {

	SetPoseMatrix(pose);
	
	cudaGraphicsResource_t Resources [2];
	Resources [0] = _Resources_t[0];
	Resources [1] = _Resources_t[1];
	checkCudaErrors( cudaGraphicsMapResources (2, Resources) );

	TransformVertexMap(_VMap_dev, _RGB_dev, _n, _m);
	
	if (quad) {
		QuadTrimFrame(_VMap_dev, _Index_dev, _n, _m, 0.1);
	}

	checkCudaErrors( cudaGraphicsUnmapResources (2, Resources) );
	checkCudaErrors( cudaDeviceSynchronize() );
}

void PredictedFrame::Merge(InputFrame *frame) {

	cudaGraphicsResource_t Resources [2];
	Resources [0] = _Resources_t[0];
	if (DISPLAY_FRAME_IN)
		Resources [1] = frame->_Resources_t[0];

	if (DISPLAY_FRAME_IN)
		checkCudaErrors( cudaGraphicsMapResources (2, Resources) );
	else
		checkCudaErrors( cudaGraphicsMapResources (1, Resources) );

	if (frame->_offline)
		MergeDepthOff(_RGB_dev, ( (OffLineFrame *) frame)->_depth_dev_test, frame->getRGB(), _Mask_dev, _n, _m);
	else
		MergeDepthKinect(_RGB_dev, ( (KinectFrame *) frame)->d_depthFrame, frame->getRGB(), _Mask_dev, _n, _m);

	checkCudaErrors( cudaMemset(_VMap_dev, 0, 3*_n*_m*sizeof(float)) );
	checkCudaErrors( cudaMemset(_NMap_dev, 0, 3*_n*_m*sizeof(float)) );
	VertexMapGL(_VMap_dev, _RGB_dev, _n, _m);
	ComputeNormal(_NMap_dev, _VMap_dev, _n, _m, true);
	
	if (DISPLAY_FRAME_IN)
		checkCudaErrors( cudaGraphicsUnmapResources (2, Resources) );
	else
		checkCudaErrors( cudaGraphicsMapResources (1, Resources) );

	/*cv::gpu::GpuMat tmp; 
	tmp.create(_n, _m, CV_8UC1);
	gpu_cpyC_char(tmp, _Mask_dev, _n, _m);
	cv::Mat tmp_host;
	tmp.download(tmp_host);
	cv::imshow("Mask", tmp_host);*/
}

void PredictedFrame::Cpy(InputFrame *frame) {
	cudaGraphicsResource_t Resources [2];
	Resources [0] = _Resources_t[0];
	if (DISPLAY_FRAME_IN)
		Resources [1] = frame->_Resources_t[0];

	if (DISPLAY_FRAME_IN)
		checkCudaErrors( cudaGraphicsMapResources (2, Resources) );
	else 
		checkCudaErrors( cudaGraphicsMapResources (1, Resources) );

	checkCudaErrors( cudaMemcpy(_VMap_dev, frame->getVMap(), 3*_n*_m*sizeof(float), cudaMemcpyDeviceToDevice) );
	checkCudaErrors( cudaMemcpy(_NMap_dev, frame->getNMap(), 3*_n*_m*sizeof(float), cudaMemcpyDeviceToDevice) );

	// Add z buffer in RGBD data 
	if (frame->_offline)
		gpu_add_zbuffOff(( (OffLineFrame *) frame)->_depth_dev_test, frame->getRGB(), _RGB_dev, _n,  _m);
	else
		gpu_add_zbuffKinect(( (KinectFrame *) frame)->d_depthFrame, frame->getRGB(), _RGB_dev, _n,  _m);
	
	if (DISPLAY_FRAME_IN)
		checkCudaErrors( cudaGraphicsUnmapResources (2, Resources) );
	else
		checkCudaErrors( cudaGraphicsUnmapResources (1, Resources) );
}

void PredictedFrame::InitMask() {
	cudaGraphicsResource_t Resources [2];
	Resources [0] = _Resources_t[0];

	checkCudaErrors( cudaGraphicsMapResources (1, Resources) );

	InitMask_cu(_Mask_dev, _RGB_dev, _n, _m);
	
	checkCudaErrors( cudaGraphicsUnmapResources (1, Resources) );
}

void PredictedFrame::ProjectMask(float *pose) {
	SetPoseMatrix(pose);
	checkCudaErrors( cudaGraphicsMapResources (1, _Resources_t) );

	ProjectMask_cu(_Mask_dev, _Mask_dev_swap, _VMap_dev, _n, _m);

	unsigned char *tmp = _Mask_dev;
	_Mask_dev = _Mask_dev_swap;	
	_Mask_dev_swap = tmp;
	
	checkCudaErrors( cudaGraphicsUnmapResources (1, _Resources_t) );
}


/***************************************************************************************/
/*************************** Methods for the class SegmFrame ***********************/
/***************************************************************************************/



void SegmFrame::Segment() {

  cv::Mat edge_canny,edge_thresh,edgeT, edgeU, edgeV,srcT,srcU,srcV;
  
  const int kern_size = 3;

  cv::gpu::GpuMat d_img;
  d_img.create(getN(), getM(), CV_8UC1);  // allocation
  cv::gpu::GpuMat d_img2;
  d_img2.create(getN(), getM(), CV_8UC1); // allocation
  
  cv::Mat tmp;

  checkCudaErrors( cudaGraphicsMapResources ( 1, _Resources_t) );

  /****** Compute edges from the Normal image **********/
  //	 Get edges from the T channel
  puts("Computing edges (Canny filter)");
    gpu_cpyN(d_img, this->getNMap(), getN(), getM(), 0); // On recupere que les composante Norm de la chnl 0
    d_img.download(srcT); //copy d_img to device in srcT
    
    //cv::imwrite("normch0.png", tmp);

//    cv::blur( srcT, srcT, cv::Size(3,3) );
    cv::Canny(srcT, edgeT, 120.0, 30.0, kern_size);
    //edgesR.download(tmp);


    gpu_cpyN(d_img, this->getNMap(), getN(), getM(), 1);
    d_img.download(srcU);// 
    
    //cv::imwrite("normch1.png", tmp);

//    cv::blur( srcU, srcU, cv::Size(3,3) );
    cv::Canny(srcU, edgeU, 120.0, 30.0, kern_size);
    //edgesG.download(tmp);
  

    gpu_cpyN(d_img, this->getNMap(), getN(), getM(), 2);
    d_img.download(srcV);// 
    
    //cv::imwrite("normch2.png", tmp);

//    cv::blur( srcV, srcV, cv::Size(3,3) );
    cv::Canny(srcV, edgeV, 120.0, 30.0, kern_size);
    //edgesB.download(tmp);
    
    cv::imwrite("Cannych0.png", edgeT);
    cv::imwrite("Cannych1.png", edgeU);
    cv::imwrite("Cannych2.png", edgeV);

//  gpu_Sum(d_img, edgesR, edgesG, edgesB, getN(), getM());// (moyenne)
  edge_canny = edgeT+edgeU+edgeV;
  cv::imwrite("CannyT.png",edge_canny);
  
  //debug
  //printf("Matrix sum (should not be 0): %lf\n",cv::sum(edge_canny)[0]);

//  d_img.download(tmp);// ici d_img = edge of the image
 // //cv::imshow("edges", tmp);
  //cv::imwrite("grad_im.png", tmp);

  //cvWaitKey(10);

  //cv::gpu::Canny(d_img, d_img, 120.0, 30.0, 3);

  //d_img.download(tmp);// ici d_img = edge of the image
  //cv::imshow("sumnormcanny", tmp);

  //cvWaitKey(10);

  //upload canny result to gpu
  cv::gpu::GpuMat d_edge_canny, d_edge_thresh;
  d_edge_canny.create(getN(), getM(), CV_8UC1);  // allocation
  d_edge_thresh.create(getN(), getM(), CV_8UC1);  // allocation
  
  d_edge_canny.upload(edge_canny);
  
  gpu_Thresh(d_edge_thresh, d_edge_canny, 20, getNMap(), getN(), getM());// Segmentation/Binarisation
  d_edge_thresh.download(edge_thresh);// img2 = parties segmentees (inverse binarisé de d_img)
  //threshold(edge_canny,edge_thresh,20,1,THRESH_BINARY); //cpu version, does not work

  //cv::imshow("edges", tmp);
  //cv::imwrite("edges.png", tmp);

  const int erosion_size = 2;
  cv::Mat element = cv::getStructuringElement( cv::MORPH_ELLIPSE, cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ), cv::Point( erosion_size, erosion_size ) );

  cv::Mat dilation_dst;

  /// Apply the erosion operation
  if (true)
    cv::erode( edge_thresh, dilation_dst, element );
  else
    dilation_dst = edge_thresh; 

  cv::imwrite("dilation.png", dilation_dst);

  //********************** Find connected components ********************************/
  cv::Mat output = cv::Mat::zeros(dilation_dst.size(), CV_8UC1);

  cv::Mat binary;
  std::vector < std::vector<cv::Point2i > > blobs;// tableau de tableaux de point (x,y), blobs rassemble tous les points connectés par label...

  cv::threshold(dilation_dst, binary, 0.0, 1.0, cv::THRESH_BINARY);//sert juste à ramener l'image dilation entre 0 et 1 au lieux de 0 et 255

  //cv::imshow("binary", binary);

  FindBlobs(binary, blobs);
  
  int blob_n = blobs.size();
  
  printf("Found %lu blobs in the input\n",blob_n);

  //size_t min_size = 5000;
  //std::vector < std::vector<cv::Point2i > > _blobsFidx;// blobsF pour stocker que les nuage de points avec plus de 2000 points

  for(size_t i=0; i < blob_n; i++) {
    
    size_t curr_size = blobs[i].size();
    
    if (curr_size==1)
      printf("%d ",i);
    else
      printf("%d S%d ",i,curr_size);
    
    if (curr_size > MIN_SIZE_PLAN) // MIN_SIZE_PLAN define min size, 2000 pts here
      _blobsFidx.push_back(blobs[i]);
    
  }   putchar('\n'); //newline after printf


  /*Test*/
  cv::Mat imgTest(getN(), getM(), CV_8UC3);
  //cv::Mat Mask = cv::Mat::zeros(getN(), getM(), CV_8UC1);//init mask to zero
  for (int i = 0; i < getN(); i++) {
    for (int j = 0; j < getM(); j++) {
      if (binary.at<unsigned char>(i,j) > 0) {//on inialise une image comme binary mais sur 3 channel
	imgTest.at<cv::Vec3b>(i,j)[0] = 255;
	imgTest.at<cv::Vec3b>(i,j)[1] = 255;
	imgTest.at<cv::Vec3b>(i,j)[2] = 255;
      } 
      else {
	imgTest.at<cv::Vec3b>(i,j)[0] = 0;
	imgTest.at<cv::Vec3b>(i,j)[1] = 0;
	imgTest.at<cv::Vec3b>(i,j)[2] = 0;
      }
    }
  }

  for(size_t i=0; i < _blobsFidx.size(); i++) {
    unsigned char R = (unsigned char)((float(rand())/RAND_MAX)*255);
    unsigned char G = (unsigned char)((float(rand())/RAND_MAX)*255);
    unsigned char B = (unsigned char)((float(rand())/RAND_MAX)*255);
    size_t curr_size = _blobsFidx[i].size();
    for (int j = 0; j < curr_size; j++) {
      if (binary.at<unsigned char>(_blobsFidx[i][j].y,_blobsFidx[i][j].x) > 0) { //is this condition usefull ??
	imgTest.at<cv::Vec3b>(_blobsFidx[i][j].y,_blobsFidx[i][j].x)[2] = R;
	imgTest.at<cv::Vec3b>(_blobsFidx[i][j].y,_blobsFidx[i][j].x)[1] = G;
	imgTest.at<cv::Vec3b>(_blobsFidx[i][j].y,_blobsFidx[i][j].x)[0] = B;
	//Mask.at<unsigned char>(_blobsFidx[i][j].y,_blobsFidx[i][j].x) = 255;
      }
    }
  }

  //cv::imshow("Mask", Mask);

  //cv::imwrite("Segmented.png", imgTest);
  //cv::imshow("Segmented", imgTest);

  checkCudaErrors( cudaGraphicsUnmapResources( 1, _Resources_t) );
}


void SegmFrame::Get3DBlobs() { 

  checkCudaErrors( cudaGraphicsMapResources ( 1, _Resources_t) );
  float *VMap = (float *) malloc(3*getN()*getM()*sizeof(float));
  cudaMemcpy(VMap, (float *) getVMap(),  3*getN()*getM() * sizeof(float), cudaMemcpyDeviceToHost);

  int m = getM();

  for (int i = 0; i < _blobsFidx.size(); i++) {
    std::vector<cv::Point3d >  blob;
      for (int j = 0; j < _blobsFidx[i].size(); ++j) {
	blob.push_back( cv::Point3d(
		double(VMap[3*(_blobsFidx[i][j].x+_blobsFidx[i][j].y*m)]),
		double(VMap[3*(_blobsFidx[i][j].x+_blobsFidx[i][j].y*m)+1]), 
		double(VMap[3*(_blobsFidx[i][j].x+_blobsFidx[i][j].y*m)+2])  ) );
      }

    _blobsF.push_back(blob);
    blob.clear();

    }

  checkCudaErrors( cudaGraphicsUnmapResources( 1, _Resources_t) );

  free(VMap);
}


void SegmFrame::ComputePCA() { 

  int m = getM();
  int blob_n = this->_blobsF.size();
  printf("Computing PCA on %d blobs...\n", blob_n);

  for (int i = 0; i < blob_n; i++) {
    
    int blob_size = (int)this->_blobsF[i].size();
    printf("Blob %d: size %d...\n",i,blob_size);

    cv::Mat Segm_vertex(blob_size, 3, CV_32FC1);

    for (int j = 0; j < blob_size; ++j){
      Segm_vertex.at<float>(j, 0) = float (_blobsF[i][j].x);
      Segm_vertex.at<float>(j, 1) = float (_blobsF[i][j].y);
      Segm_vertex.at<float>(j, 2) = float (_blobsF[i][j].z);
    }

    cv::PCA pca_analysis(Segm_vertex, cv::Mat(), CV_PCA_DATA_AS_ROW);
    cv::Point3d pos(double(pca_analysis.mean.at<float>(0, 0)), double(pca_analysis.mean.at<float>(0, 1)), double(pca_analysis.mean.at<float>(0, 2)));
    vector<cv::Point3d> eigenvectors(4); // 4 : adding the position to the last position

    for (int k = 0; k < 3; ++k){
      eigenvectors[k] = cv::Point3d(double(pca_analysis.eigenvectors.at<float>(k, 0)),
      double(pca_analysis.eigenvectors.at<float>(k, 1)), 
      double(pca_analysis.eigenvectors.at<float>(k, 2)));
    }
    // on met la position de valeur moyenne comme 4 eme coordonnee des vecteurs propres
    eigenvectors[3] = pos;

    cout << "eigen_vecs : " << eigenvectors[0] << " " << eigenvectors[1] << " " << eigenvectors[2] <<  endl;

    _eigenVecs.push_back(eigenvectors);
    Segm_vertex.release();
    eigenvectors.clear();
    
    printf("Blob %d: PCA done\n",i);
  }
}

void SegmFrame::GetBboxOriented(){ // Gives 4 first control points

		//////////////////////////////// Getting min and max of the bounding box ////////

	std::vector<cv::Point3d> bbox;
	std::vector<Point3D>  ctrls;
	double temp[3];

	double maxi = std::numeric_limits<double>::max();
	double mini = -maxi+ 1;


	for (int i = 0; i < _blobsF.size(); i++) {

			double xori = 0, yori = 0, zori = 0, xomin = maxi, yomin = maxi, zomin = maxi, xomax = mini , yomax = mini, zomax=mini, 
					xp = 0, yp = 0, zp = 0, xpmin = maxi, ypmin = maxi, zpmin = maxi, xpmax = mini, ypmax = mini, zpmax = mini,
					xp2=0, yp2=0, zp2=0;

			int Cxpmin = 0, Cypmin = 0, Czpmin = 0, Cxpmax = 0, Cypmax = 0, Czpmax = 0;
			
			for (int j = 0; j < _blobsF[i].size(); j++) {
					xori = _blobsF[i][j].x;
					yori = _blobsF[i][j].y;
					zori = _blobsF[i][j].z;
								

					xp = _eigenVecs[i][0].x*xori + _eigenVecs[i][0].y*yori + _eigenVecs[i][0].z*zori; 
					yp = _eigenVecs[i][1].x*xori + _eigenVecs[i][1].y*yori + _eigenVecs[i][1].z*zori; 
					zp = _eigenVecs[i][2].x*xori + _eigenVecs[i][2].y*yori + _eigenVecs[i][2].z*zori;


					if (xpmin > xp) {xpmin = xp;}
					if (xpmax < xp) {xpmax = xp;}

					if (ypmin > yp) {ypmin = yp;}
					if (ypmax < yp) {ypmax = yp;}

					if (zpmin > zp) {zpmin = zp;}
					if (zpmax < zp) {zpmax = zp;}
			}

			
		
			temp[0] = _eigenVecs[i][0].x*xpmin + _eigenVecs[i][1].x*ypmax + _eigenVecs[i][2].x*zpmin; 
			temp[1] = _eigenVecs[i][0].y*xpmin + _eigenVecs[i][1].y*ypmax + _eigenVecs[i][2].y*zpmin; 
			temp[2] = _eigenVecs[i][0].z*xpmin + _eigenVecs[i][1].z*ypmax + _eigenVecs[i][2].z*zpmin;
			bbox.push_back(cv::Point3d(temp[0],temp[1],temp[2]));

			temp[0] = _eigenVecs[i][0].x*xpmin + _eigenVecs[i][1].x*ypmax + _eigenVecs[i][2].x*zpmax; 
			temp[1] = _eigenVecs[i][0].y*xpmin + _eigenVecs[i][1].y*ypmax + _eigenVecs[i][2].y*zpmax;
			temp[2] = _eigenVecs[i][0].z*xpmin + _eigenVecs[i][1].z*ypmax + _eigenVecs[i][2].z*zpmax;
			bbox.push_back(cv::Point3d(temp[0],temp[1],temp[2]));

			temp[0] = _eigenVecs[i][0].x*xpmin + _eigenVecs[i][1].x*ypmin + _eigenVecs[i][2].x*zpmax; 
			temp[1] = _eigenVecs[i][0].y*xpmin + _eigenVecs[i][1].y*ypmin + _eigenVecs[i][2].y*zpmax; 
			temp[2] = _eigenVecs[i][0].z*xpmin + _eigenVecs[i][1].z*ypmin + _eigenVecs[i][2].z*zpmax;
			bbox.push_back(cv::Point3d(temp[0],temp[1],temp[2]));

			temp[0] = _eigenVecs[i][0].x*xpmin + _eigenVecs[i][1].x*ypmin + _eigenVecs[i][2].x*zpmin; 
			temp[1] = _eigenVecs[i][0].y*xpmin + _eigenVecs[i][1].y*ypmin + _eigenVecs[i][2].y*zpmin; 
			temp[2] = _eigenVecs[i][0].z*xpmin + _eigenVecs[i][1].z*ypmin + _eigenVecs[i][2].z*zpmin;
			bbox.push_back(cv::Point3d(temp[0],temp[1],temp[2]));

			temp[0] = _eigenVecs[i][0].x*xpmax + _eigenVecs[i][1].x*ypmin + _eigenVecs[i][2].x*zpmin; 
			temp[1] = _eigenVecs[i][0].y*xpmax + _eigenVecs[i][1].y*ypmin + _eigenVecs[i][2].y*zpmin; 
			temp[2] = _eigenVecs[i][0].z*xpmax + _eigenVecs[i][1].z*ypmin + _eigenVecs[i][2].z*zpmin;
			bbox.push_back(cv::Point3d(temp[0],temp[1],temp[2]));

			temp[0] = _eigenVecs[i][0].x*xpmax + _eigenVecs[i][1].x*ypmin + _eigenVecs[i][2].x*zpmax; 
			temp[1] = _eigenVecs[i][0].y*xpmax + _eigenVecs[i][1].y*ypmin + _eigenVecs[i][2].y*zpmax; 
			temp[2] = _eigenVecs[i][0].z*xpmax + _eigenVecs[i][1].z*ypmin + _eigenVecs[i][2].z*zpmax;
			bbox.push_back(cv::Point3d(temp[0],temp[1],temp[2]));

			temp[0] = _eigenVecs[i][0].x*xpmax + _eigenVecs[i][1].x*ypmax + _eigenVecs[i][2].x*zpmax; 
			temp[1] = _eigenVecs[i][0].y*xpmax + _eigenVecs[i][1].y*ypmax + _eigenVecs[i][2].y*zpmax; 
			temp[2] = _eigenVecs[i][0].z*xpmax + _eigenVecs[i][1].z*ypmax + _eigenVecs[i][2].z*zpmax;
			bbox.push_back(cv::Point3d(temp[0],temp[1],temp[2]));

			temp[0] = _eigenVecs[i][0].x*xpmax + _eigenVecs[i][1].x*ypmax + _eigenVecs[i][2].x*zpmin; 
			temp[1] = _eigenVecs[i][0].y*xpmax + _eigenVecs[i][1].y*ypmax + _eigenVecs[i][2].y*zpmin; 
			temp[2] = _eigenVecs[i][0].z*xpmax + _eigenVecs[i][1].z*ypmax + _eigenVecs[i][2].z*zpmin;
			bbox.push_back(cv::Point3d(temp[0],temp[1],temp[2]));


			/// Adding 4 First control points ! :

			temp[0] = (bbox[0].x + bbox[1].x)/2.0;
			temp[1] = (bbox[0].y + bbox[1].y)/2.0; 
			temp[2] = (bbox[0].z + bbox[1].z)/2.0;
			ctrls.push_back(Point3D(temp[0],temp[1],temp[2]));

			temp[0] = (bbox[2].x + bbox[3].x)/2.0;
			temp[1] = (bbox[2].y + bbox[3].y)/2.0; 
			temp[2] = (bbox[2].z + bbox[3].z)/2.0;
			ctrls.push_back(Point3D(temp[0],temp[1],temp[2]));

			temp[0] = (bbox[4].x + bbox[5].x)/2.0;
			temp[1] = (bbox[4].y + bbox[5].y)/2.0; 
			temp[2] = (bbox[4].z + bbox[5].z)/2.0;
			ctrls.push_back(Point3D(temp[0],temp[1],temp[2]));

			temp[0] = (bbox[6].x + bbox[7].x)/2.0;
			temp[1] = (bbox[6].y + bbox[7].y)/2.0; 
			temp[2] = (bbox[6].z + bbox[7].z)/2.0;
			ctrls.push_back(Point3D(temp[0],temp[1],temp[2]));

			_bboxs.push_back(bbox);
			bbox.clear();
			_ctrls.push_back(ctrls);
			ctrls.clear();
	}

}


void SegmFrame::Find2DCtrlPts(){
  //We got the segmented image and point clouds : _blobsFidx
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Let's find the control points on depth image :
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////


  // load depth image in CPU from GPU :
  cv::Mat depthImg;
  _depth_dev_test.download(depthImg); 

  //Apply gaussians filter to remove noises :
  GaussianBlur( depthImg, depthImg, Size(9, 9), 0, 0, BORDER_DEFAULT );
  GaussianBlur( depthImg, depthImg, Size(9, 9), 0, 0, BORDER_DEFAULT );
  GaussianBlur( depthImg, depthImg, Size(7, 7), 0, 0, BORDER_DEFAULT );
  //writeMatToFile8U(depthImg, "depthImg.txt");
  //cout<<type2str(depthImg.type())<<endl;//16UC1


  //// Compute symmetry of the depthImg :
  Point2f srcTri[3];
  Point2f dstTri[3];
  Mat warp_mat( 2, 3, CV_32FC1 );

  /// Set your 3 points to calculate the  Affine Transform
  srcTri[0] = Point2f( 0,0 );
  srcTri[1] = Point2f( getM() - 1, 0 );
  srcTri[2] = Point2f( 0, getN() - 1 );

  dstTri[0] = Point2f(  0, getN() - 1  );
  dstTri[1] = Point2f( getM() - 1, getN() - 1  );
  dstTri[2] = Point2f( 0,0 );

  /// Get the Affine Transform
  warp_mat = getAffineTransform( srcTri, dstTri );

  /// Apply the Affine Transform just found to the src image
  warpAffine( depthImg, depthImg, warp_mat, depthImg.size() );



  /// Compute grad from depth image (grad is the sum of second gradients):
  int scale = 1;
  int delta = 0;
  int ddepth = CV_32F;
  // Gradient i j
  cv::Mat grad_i, grad_j;
  cv::Mat abs_grad_i, abs_grad_j;

  cv::Mat grad_ii, grad_ij;
  cv::Mat abs_grad_ii, abs_grad_ij;

  cv::Mat grad_ji, grad_jj;
  cv::Mat abs_grad_ji, abs_grad_jj;

  cv::Mat grad1;
  cv::Mat grad2;
  cv::Mat grad;
  

  cv::Sobel(depthImg, grad_i, ddepth, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT);
  convertScaleAbs( grad_i, abs_grad_i );

  cv::Sobel(depthImg, grad_j, ddepth, 0, 1, 3, scale, delta, cv::BORDER_DEFAULT);
  convertScaleAbs( grad_j, abs_grad_j );
  
  cv::Sobel(grad_i, grad_ii, ddepth, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT);
  convertScaleAbs( grad_ii, abs_grad_ii );

  cv::Sobel(grad_i, grad_ij, ddepth, 0, 1, 3, scale, delta, cv::BORDER_DEFAULT);
  convertScaleAbs( grad_ij, abs_grad_ij );

  cv::Sobel(grad_j, grad_ji, ddepth, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT);
  convertScaleAbs( grad_ji, abs_grad_ji );

  cv::Sobel(grad_j, grad_jj, ddepth, 0, 1, 3, scale, delta, cv::BORDER_DEFAULT);
  convertScaleAbs( grad_jj, abs_grad_jj );

  /// Total Gradient (approximate)
  addWeighted( abs_grad_ii, 0.5, abs_grad_jj, 0.5, 0, grad1 );
  addWeighted( abs_grad_ji, 0.5, abs_grad_ij, 0.5, 0, grad2 );
  addWeighted( grad1, 0.5, grad2, 0.5, 0, grad );


  //cv::imshow("grad", grad);
  //cv::imshow("abs_grad_j", abs_grad_j);



  //drawHist(grad, depthimage);

  cv::threshold(grad, grad, /*120*/30, 255, cv::THRESH_BINARY);



  for(int m=0; m < _blobsFidx.size(); m++) {
  
    // define tyhe mask corresponding to the blobs
    cv::Mat Mask = cv::Mat::zeros(getN(), getM(), CV_8UC1);//init mask to zero

    for (int j = 0; j <  _blobsFidx[m].size(); j++) {
      Mask.at<unsigned char>(_blobsFidx[m][j].y,_blobsFidx[m][j].x) = 255;
    }

    //cv::imshow("maski", Mask);

    //Erode the mask to remove boundaries :
    int erosion_size2 =7;
    cv::Mat element2 = cv::getStructuringElement( cv::MORPH_ELLIPSE, cv::Size( 2*erosion_size2 + 1, 2*erosion_size2+1 ), cv::Point( erosion_size2, erosion_size2 ) );
    cv::erode( Mask, Mask, element2 );


    cv::Mat gradwithmask;
    grad.copyTo(gradwithmask, Mask);

    //drawHist(gradwithmask, Mask);


    //cv::imshow("gradwithmask2", gradwithmask);


    vector <Point3D> ctrlsidx;

    for (int i = 0; i < getN(); i++) {
      for (int j = 0; j < getM(); j++) {
	if(gradwithmask.at<unsigned char>(i,j)>0)
		ctrlsidx.push_back( Point3D(double(j),double(i), 0.0) ); //permut i and j for being coherent with Vmap
      }
    }

    _ctrlsidx.push_back(ctrlsidx);
    ctrlsidx.clear();
    //cvWaitKey(10);  

  
  }//end for
}


void SegmFrame::Get3DCtrlPts(){

  checkCudaErrors( cudaGraphicsMapResources ( 1, _Resources_t) );
  float *VMap = (float *) malloc(3*getN()*getM()*sizeof(float));
  cudaMemcpy(VMap, (float *) getVMap(),  3*getN()*getM() * sizeof(float), cudaMemcpyDeviceToHost);
  
  int m = getM();
  
  for (int i = 0; i < _ctrlsidx.size(); i++) {
    std::vector<Point3D>  ctrls;
      for (int j = 0; j < _ctrlsidx[i].size(); ++j){
	// cout<<int(_ctrlsidx[i][j].x())<<" "<<int(_ctrlsidx[i][j].y())<<" "<<&_ctrlsidx[i]<<" "<<VMap<<endl;
	ctrls.push_back( Point3D( 
	  double( VMap[3*(int(_ctrlsidx[i][j].x())+int(_ctrlsidx[i][j].y())*m)]),
	  double( VMap[3*(int(_ctrlsidx[i][j].x())+int(_ctrlsidx[i][j].y())*m)+1]), 
	  double( VMap[3*(int(_ctrlsidx[i][j].x())+int(_ctrlsidx[i][j].y())*m)+2])  ) );
	}

    cout<<"nb control pts before CGAL simplifyCloud:"<<ctrls.size()<<endl;

    simplifyCloud(ctrls, 0.06); // Use CGAL functions

    cout<<"nb control pts after CGAL simplifyCloud:"<<ctrls.size()<<endl;

    _ctrls[i].insert(_ctrls[i].end(), ctrls.begin(), ctrls.end());// on ajoute la liste de control pts à celle existante, initialisé dans GetBbox (car on avait deja les 4 premiers control points)
    ctrls.clear();
  }

  checkCudaErrors( cudaGraphicsUnmapResources( 1, _Resources_t) );

  free(VMap);

}


void SegmFrame::InitSurfs(){
int blob_n = _blobsF.size();

printf("Getting surfaces for %d blobs\n", blob_n);

  for(int i=0; i< blob_n ; i++){
    Surface *Surf;
    Surf = new Surface(300);
    _Surfs.push_back(Surf);

  }
}

void SegmFrame::ComputeSurfs(){
  
  int surf_n = _Surfs.size();
  printf("Computing %d surfaces\n", surf_n);

  string labelimage = "labelImage.png";
  string bumpimage = "bumpImage.png";
  string extension = ".png";

  for(int i = 0; i< surf_n; i++){
    
    printf("Surface #%d\n", i); fflush(stdout);

//  	 i = 1;//8 sphere // 2 chair //7 wall
    
    puts("ImportCtrlPts");
    _Surfs[i]->ImportCtrlPts(& _ctrls[i], 0);

    puts("Proj3Dpts");
    _Surfs[i]->Proj3Dpts();

    puts("GetEquations");
    _Surfs[i]->GetEquations();

    puts("FillEdges");
    _Surfs[i]->FillEdges();

    puts("g2omain");
    _Surfs[i]->g2omain();

    while(_Surfs[i]->findPointToDupli())
    {	
      _Surfs[i]->g2omain();

      if(_Surfs[i]->OverlapTest() == 0){
	_Surfs[i]->OptCopytoBuff();
      }
      else{
	_Surfs[i]->RecoverFromBuff();
	break;
      }
    }

    _Surfs[i]->ClearOptBuff();

  //	_Surfs[i]->g2omain(); // on peu le faire ou pas, si on le fait on met a jour les reultats de l'optimisation sauve dans le .g2o (apres un recover)

  //	_Surfs[i]->OptiResults();

    _Surfs[i]->ShiftOptCtrlsIdx();

    _Surfs[i]->ComputeImgIdx();
    _Surfs[i]->ComputeBumpImg(_blobsF[i]);

//     if(i==8){
// 	    labelimage = "labelImg" + to_string((long double)(i)) + extension ;
// 	    _Surfs[i]->DisplayLabelImg(labelimage);
// 	    bumpimage = "bumpImage" + to_string((long double)(i)) + extension ;
// 	    _Surfs[i]->DisplayBumpImg(bumpimage);
//     }

    _Surfs[i]->DrawRecImg();

//    cvWaitKey(20);// Just to let the time to images to display
  }

//	}


//
//		i = i+1;
//		
//		_Surfs[i]->ImportCtrlPts(& _ctrls[i-1], 1);
//
//
//		
//		_Surfs[i]->Proj3Dpts();
//

//
//		_Surfs[i]->GetEquations();
//
//
//		_Surfs[i]->FillEdges();
//
//		_Surfs[i]->g2omain();
//
//		while(_Surfs[i]->findPointToDupli())
//		{	
//				_Surfs[i]->g2omain();
//
//				if(_Surfs[i]->OverlapTest() == 0){
//					_Surfs[i]->OptCopytoBuff();
//				}
//				
//				else{
//					_Surfs[i]->RecoverFromBuff();
//					break;
//				}
//		}
//
//
//		_Surfs[i]->ClearOptBuff();
//
//		_Surfs[i]->g2omain(); // on peu le faire ou pas, si on le fait on met ajour les reultats de l'optimisation sauve dans le .g2o (apres un recover)
//		
//	
//
//		cout<<i<<endl;
//
//		_Surfs[i]->ShiftOptCtrlsIdx();
//
//		// allocate the good size of memory now !
//
//		_Surfs[i]->ComputeImgIdx();
//
//		_Surfs[i]->ComputeBumpImg(_blobsF[i-1], "bumprecplan.png");
//
//		_Surfs[i]->DrawRecImg();

}

void SegmFrame::Display3DBlobs(unsigned int NUM_VERTEX) {

  glBegin(GL_POINTS);
	  
  glColor4ub(0,0,255, 255);

  //glColor4ub(500*((NUM_VERTEX+2)*(NUM_VERTEX+1)),200*(NUM_VERTEX+1)*(NUM_VERTEX+5),300*(NUM_VERTEX+1), 180);

  for(int j = 0; j < _blobsF[NUM_VERTEX].size(); j++){
  glVertex3d(_blobsF[NUM_VERTEX][j].x,_blobsF[NUM_VERTEX][j].y, _blobsF[NUM_VERTEX][j].z);
  }
  
  glEnd();
}

void SegmFrame::Display3DCtrlPts(int num) {

  glPointSize(3);	

  glBegin(GL_POINTS);

  glColor4ub(255,0,255, 255);
  int i = num;
  //for(int i = 0; i < _ctrls.size(); i++){
    for(int j = 0; j < _ctrls[i].size(); j++){
      if(j==0){glColor4ub(255,0,0, 255);}
      if(j==1){glColor4ub(0,255,0, 255);}
      if(j==2){glColor4ub(0,0,255, 255);}
      if(j==3){glColor4ub(255,100,100, 255);}
//	else{glColor4ub(255,0,255, 255);}
      glVertex3d(_ctrls[i][j][0],_ctrls[i][j][1], _ctrls[i][j][2]);
    }
  //}

  glEnd();

  glPointSize(1);
}




void SegmFrame::DisplayEigVect(unsigned int NUM_VERTEX){

    glBegin(GL_LINES);
    
    glLineWidth( 3 );
    glColor4ub(255,255,0, 180);//x' jaune
    glVertex3d(_eigenVecs[NUM_VERTEX][3].x ,_eigenVecs[NUM_VERTEX][3].y,_eigenVecs[NUM_VERTEX][3].z);
    glVertex3d(_eigenVecs[NUM_VERTEX][0].x + _eigenVecs[NUM_VERTEX][3].x,_eigenVecs[NUM_VERTEX][0].y + _eigenVecs[NUM_VERTEX][3].y,_eigenVecs[NUM_VERTEX][0].z + _eigenVecs[NUM_VERTEX][3].z); // min x y z

    glColor4ub(0,255,255, 180);//y' cyan
    glVertex3d(_eigenVecs[NUM_VERTEX][3].x ,_eigenVecs[NUM_VERTEX][3].y,_eigenVecs[NUM_VERTEX][3].z);
    glVertex3d(_eigenVecs[NUM_VERTEX][1].x + _eigenVecs[NUM_VERTEX][3].x ,_eigenVecs[NUM_VERTEX][1].y + _eigenVecs[NUM_VERTEX][3].y,_eigenVecs[NUM_VERTEX][1].z + _eigenVecs[NUM_VERTEX][3].z); // min x y z

    glColor4ub(255,0,255, 180);//z' violet
    glVertex3d(_eigenVecs[NUM_VERTEX][3].x ,_eigenVecs[NUM_VERTEX][3].y,_eigenVecs[NUM_VERTEX][3].z);
    glVertex3d(_eigenVecs[NUM_VERTEX][2].x + _eigenVecs[NUM_VERTEX][3].x ,_eigenVecs[NUM_VERTEX][2].y + _eigenVecs[NUM_VERTEX][3].y,_eigenVecs[NUM_VERTEX][2].z + _eigenVecs[NUM_VERTEX][3].z); // min x y z
      
  glEnd();
}






void SegmFrame::DisplayBbox(){

  glBegin(GL_QUADS);
  for(int i = 0; i < _bboxs.size(); i++) {
    
    glColor4ub(500*((i+2)*(i+1)),200*(i+1)*(i+5),300*(i+1), 180);

    glVertex3d(_bboxs[i][0].x,_bboxs[i][0].y,_bboxs[i][0].z); 
    glVertex3d(_bboxs[i][1].x,_bboxs[i][1].y,_bboxs[i][1].z);
    glVertex3d(_bboxs[i][2].x,_bboxs[i][2].y,_bboxs[i][2].z);
    glVertex3d(_bboxs[i][3].x,_bboxs[i][3].y,_bboxs[i][3].z);

    glVertex3d(_bboxs[i][4].x,_bboxs[i][4].y,_bboxs[i][4].z);
    glVertex3d(_bboxs[i][5].x,_bboxs[i][5].y,_bboxs[i][5].z);
    glVertex3d(_bboxs[i][6].x,_bboxs[i][6].y,_bboxs[i][6].z);
    glVertex3d(_bboxs[i][7].x,_bboxs[i][7].y,_bboxs[i][7].z);

    glVertex3d(_bboxs[i][7].x,_bboxs[i][7].y,_bboxs[i][7].z);
    glVertex3d(_bboxs[i][6].x,_bboxs[i][6].y,_bboxs[i][6].z);
    glVertex3d(_bboxs[i][1].x,_bboxs[i][1].y,_bboxs[i][1].z);
    glVertex3d(_bboxs[i][0].x,_bboxs[i][0].y,_bboxs[i][0].z);

    glVertex3d(_bboxs[i][5].x,_bboxs[i][5].y,_bboxs[i][5].z);
    glVertex3d(_bboxs[i][4].x,_bboxs[i][4].y,_bboxs[i][4].z);
    glVertex3d(_bboxs[i][3].x,_bboxs[i][3].y,_bboxs[i][3].z);
    glVertex3d(_bboxs[i][2].x,_bboxs[i][2].y,_bboxs[i][2].z);

    glVertex3d(_bboxs[i][0].x,_bboxs[i][0].y,_bboxs[i][0].z); 
    glVertex3d(_bboxs[i][3].x,_bboxs[i][3].y,_bboxs[i][3].z);
    glVertex3d(_bboxs[i][4].x,_bboxs[i][4].y,_bboxs[i][4].z);
    glVertex3d(_bboxs[i][7].x,_bboxs[i][7].y,_bboxs[i][7].z);

    glVertex3d(_bboxs[i][6].x,_bboxs[i][6].y,_bboxs[i][6].z);
    glVertex3d(_bboxs[i][5].x,_bboxs[i][5].y,_bboxs[i][5].z);
    glVertex3d(_bboxs[i][2].x,_bboxs[i][2].y,_bboxs[i][2].z);
    glVertex3d(_bboxs[i][1].x,_bboxs[i][1].y,_bboxs[i][1].z);

  }
  
  glEnd();

}


void DrawAxis0(void){

  glLineWidth( 2 );
  glBegin(GL_LINES);

  glColor4ub(255,0,0, 180);
  glVertex3f(0,0,0);
  glVertex3f(100,0,0);


  glColor4ub(0,255,0, 180);
  glVertex3f(0,0,0);
  glVertex3f(0,100,0);


  glColor4ub(0,0,255, 180);
  glVertex3f(0,0,0);
  glVertex3f(0,0,100);

  glEnd();

  glLineWidth( 1 );


  glPointSize(6);
  glEnable(GL_POINT_SMOOTH);

  glBegin(GL_POINTS);
	  
  glColor4ub(255,255,255, 255);
  glVertex3f(0, 0, 0);

  glEnd();

  glPointSize(1);
  glDisable(GL_POINT_SMOOTH);

}
