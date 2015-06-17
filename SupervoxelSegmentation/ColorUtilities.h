/*
 * ColorUtilities.h
 *
 *  Created on: 01/giu/2015
 *      Author: Francesco Verdoja <verdoja@di.unito.it>
 */

#ifndef COLORUTILITIES_H_
#define COLORUTILITIES_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/supervoxel_clustering.h>

using namespace pcl;

typedef PointXYZRGBA PointT;
typedef Supervoxel<PointT> SupervoxelT;

const double PI = 3.141592653589793238462643383279;
const float RGB_RANGE = 441.672943;
const float LAB_RANGE = 137.3607;

class ColorUtilities {
	static float * color_conversion(float in[3], int code);
	static float ciede00_test(float L1, float a1, float b1, float L2, float a2,
			float b2, float result);
	ColorUtilities() {
	}

public:
	static float * mean_color(SupervoxelT::Ptr s);
	static float * rgb2lab(float rgb[3]);
	static float * lab2rgb(float lab[3]);
	static float lab_ciede00(float lab1[3], float lab2[3], double kL = 1.0,
			double kC = 1.0, double kH = 1.0);
	static float rgb_eucl(float rgb1[3], float rgb2[3]);
	static void rgb_test();
	static void lab_test();
	static void convert_test();

};

#endif /* COLORUTILITIES_H_ */
