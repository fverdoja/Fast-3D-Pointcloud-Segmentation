#ifndef __SURFACE_H
#define __SURFACE_H

#pragma once


#include <fstream>
#include <iostream>

/*** Include files for OpenGL to work ***/
#include <GL/glew.h>
#include <GL/glut.h>

/*** Standard include files for manipulating vectors, files etc... ***/
#include <vector>
#include <map>
#include <Eigen/StdVector>

#include "g2outilities.h"

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

#define MeasType Eigen::Vector2d

#include "g2o_tutorial_slam2d_api.h"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

//#include <CGAL/Triangulation_euclidean_traits_xy_3.h> //DEPRECATED
#include <CGAL/Projection_traits_xy_3.h> //use instead of deprecated

#include <CGAL/Delaunay_triangulation_2.h>

#include <CGAL/grid_simplify_point_set.h>
#include <CGAL/IO/read_xyz_points.h>
#include <CGAL/property_map.h>
#include <CGAL/remove_outliers.h>
#include <vector>
#include <fstream>

// types
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;

//typedef CGAL::Triangulation_euclidean_traits_xy_3<K> Gt; //DEPRECATED
typedef CGAL::Projection_traits_xy_3<K>  Gt; //use instead of deprecated

typedef CGAL::Delaunay_triangulation_2<K> Delaunay;
typedef K::Point_3 Point3D;
typedef K::Point_2 Point2D;


#define EPSILON 1e-10 //TODO: conflict with a CUDA variable: cudaTypes.cuh:51:0: warning: "EPSILON" redefined

using namespace std;


void cross_3d(double * res, const double * A, const double * B);
double cross_2d(double res, const double * A, const double * B);
void base_2d(const double * V1,  const double * V2, const double * V3, double V2toB[2]) ;
void base_3d(const double * V1, const double * V2, const double * V3, double V2toB[3]);

struct edgeResult{

	int numTri;
	int idxPos[2]; // index from and to
	double res[5]; // dopt, dreal, hopt, hreal, f result

};

struct Equations {
	double _normal[3];
	double _dist;
	int _idxPos[3];
	bool _border[3];

	Equations(double normal[3], double dist, int idxPos[3], bool border[3]) {
		_normal[0] = normal[0]; _normal[1] = normal[1]; _normal[2] = normal[2];
		_dist = dist;
		_idxPos[0] = idxPos[0]; _idxPos[1] = idxPos[1]; _idxPos[2] = idxPos[2]; 
		_border[0] = border[0]; _border[1] = border[1] ; _border[2] = border[2];
	}
};

struct G2O_TUTORIAL_SLAM2D_API GridEdge
        {
          int from;
          int to;
		  int last;
		  double ratio;
		  MeasType trueMeas;
		  Eigen::Matrix2d information;
          EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        };

typedef std::vector<GridEdge, Eigen::aligned_allocator<GridEdge> >  GridEdgeVector;

class Surface {
public:

	int _n;
	int _m; //_n and _m size of the allocated memory for the 2D images (lab and bump)
	int _k; // _scale factor which will multiply every 3D distance to assure a minimum resolution of the optimised result (k is the resolution)

	vector <Point3D> _Controls;
	vector <Point2D> _ControlsIdx;
	vector <Point2D> _OptCtrsBuff;
	vector <Point2D> _OptControlsIdx;

	int _numLastInxDupli;


	vector <Equations> _Equations;

	GridEdgeVector _EdgesMeasures;


	//rendu :
	int **_ImgIndx;
	double **_BumpImg;
	char ***_RGBImg;
	double ***_VtxImg;
	double ***_RecImg;


public:

	Surface(int k): _k(k) {
		_m = 0;
		_n = 0;
		_numLastInxDupli = 0;
		_ImgIndx = NULL;
		_BumpImg = NULL;
		_VtxImg = NULL;
		_RecImg = NULL;
		_RGBImg = NULL;
	};
	~Surface() {};

	inline int getN() {return _n;}
	inline int getM() {return _m;}

	inline void AddCtrl(Point3D pt) {_Controls.push_back(pt);}
	inline void AddCtrlIdx(Point2D idx) {_ControlsIdx.push_back(idx);}
	inline void AddOptIdx(Point2D opt) {_OptControlsIdx.push_back(opt);}

	Eigen::Vector2d GetControlsIdxValue(int i){Eigen::Vector2d v; v[0]=_ControlsIdx[i][0]; v[1]=_ControlsIdx[i][1]; return v;}
	const double* GetCtrlsIdx(int i){return &(_ControlsIdx[i][0]);}
	const double* GetCtrls (int i){return &(_Controls[i][0]);}
	const double* GetOptCtrlsIdx(int i){return &(_OptControlsIdx[i][0]);}


	void ImportCtrlPts(std::vector<Point3D> * ctrls, int fourctrl);
	void ShowCtrlPts();
	void Proj3Dpts();
	void GetEquations();

	void FillEdges();
	void AddEdge(int i, int j, int k, Eigen::Matrix2d& inf);
	void g2omain();
	void OptiResults();

	bool findPointToDupli( );
	bool TriangleBorderTest(int numTri);
	void dupliIndex(int numDupli, int numTri);

	bool OverlapTest( );
	void OptCopytoBuff();
	void RecoverFromBuff();
	void ClearOptBuff();

	void ShiftOptCtrlsIdx();
	void ComputeImgIdx();
	void ComputeBumpImg(std::vector<cv::Point3d >& blob);
	bool GetDistAndIdx(cv::Point3d pt, double& distProj, int idx[2], int& numTri);

	void DrawSurf();
	void DrawRecImg();
	double* getPoint3D(int i, int j, int numPlan);

	void DisplayLabelImg(string filename_label);
	void DisplayBumpImg(string filename_bump);
	void DisplaySurf();
	void DisplayRecImg(int color);
	void DisplaySurfsPlus(int switch1);
};



#endif
