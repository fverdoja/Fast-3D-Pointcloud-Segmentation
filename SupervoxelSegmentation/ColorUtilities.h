/*
 * ColorUtilities.h
 *
 *  Created on: 01/06/2015
 *      Author: Francesco Verdoja <verdoja@di.unito.it>
 *
 *
 * Software License Agreement (BSD License)
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
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
