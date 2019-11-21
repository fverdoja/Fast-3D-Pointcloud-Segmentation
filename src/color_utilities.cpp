/*
 * color_utilities.cpp
 *
 *  Created on: 01/06/2015
 *      Author: Francesco Verdoja <francesco.verdoja@aalto.fi>
 *
 *
 * BSD 3-Clause License
 * 
 * Copyright (c) 2015, Francesco Verdoja
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 * 
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * 
 * * Neither the name of the copyright holder nor the names of its 
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "supervoxel_clustering/color_utilities.h"

/**
 * Convert from one color space to another
 * 
 * @param in    a vector containing either three L*a*b* or three RGB channels
 * @param code  color conversion code (see OpenCV cvtColor documentation for
 *              acceptable values
 * 
 * @return the converted color vector containing either three L*a*b* or three 
 *         RGB channels
 */
float * ColorUtilities::color_conversion(float in[3], int code) {
    float i1 = in[0];
    float i2 = in[1];
    float i3 = in[2];

    cv::Mat in_m(1, 1, CV_32FC3, cv::Scalar(i1, i2, i3));
    cv::Mat out_m(1, 1, CV_32FC3, cv::Scalar(0, 0, 0));

    cv::cvtColor(in_m, out_m, code);

    cv::Vec3f out_v = out_m.at<cv::Vec3f>(0, 0);
    float * out = new float[3];
    out[0] = out_v[0];
    out[1] = out_v[1];
    out[2] = out_v[2];

    return out;
}

/**
 * Test the internal implementation of CIEDE2000 distance comparing its result 
 * against an expected result
 * 
 * @param L1        L channel for the first color
 * @param a1        a channel for the first color
 * @param b1        b channel for the first color
 * @param L2        L channel for the second color
 * @param a2        a channel for the second color
 * @param b2        b channel for the second color
 * @param result    the expected CIEDE2000 distance
 * 
 * @return the difference between the computed CIED2000 distance and the 
 *         expected result
 */
float ColorUtilities::ciede00_test(float L1, float a1, float b1, float L2,
        float a2, float b2, float result) {
    float lab1[] = {L1, a1, b1};
    float lab2[] = {L2, a2, b2};
    float delta_e = lab_ciede00(lab1, lab2);
    return std::abs(delta_e - result);
}

/**
 * Get the color corresponding to the given label in the Glasbey lookup table
 * 
 * @param label the label of the desired color
 * 
 * @return the color corresponding to the given label
 */
uint8_t * ColorUtilities::get_glasbey(uint32_t label) {
    pcl::RGB color = pcl::GlasbeyLUT::at(label % pcl::GlasbeyLUT::size());
    uint8_t * rgb = new uint8_t[3];
    rgb[0] = color.r;
    rgb[1] = color.g;
    rgb[2] = color.b;
    return rgb;
}

/**
 * Compute the mean color of all points in a region
 * 
 * @param s a segmentation region
 * 
 * @return the mean color of all points as an array of RGB values
 */
float * ColorUtilities::mean_color(SupervoxelT::Ptr s) {
    pcl::PointCloud<PointT>::Ptr v = s->voxels_;
    pcl::PointCloud<PointT>::iterator v_itr, v_itr_end;
    v_itr = v->begin();
    v_itr_end = v->end();
    float count = 0;
    float mean_r = 0;
    float mean_g = 0;
    float mean_b = 0;
    for (; v_itr != v_itr_end; ++v_itr) {
        float r = v_itr->r;
        float g = v_itr->g;
        float b = v_itr->b;
        count++;

        mean_r = mean_r + (1 / count) * (r - mean_r);
        mean_g = mean_g + (1 / count) * (g - mean_g);
        mean_b = mean_b + (1 / count) * (b - mean_b);
    }
    float * mean = new float[3];
    mean[0] = mean_r;
    mean[1] = mean_g;
    mean[2] = mean_b;

    return mean;
}

/**
 * Convert RGB colors to L*a*b* color space
 * 
 * @param rgb   the RGB color to be converted
 * 
 * @return the corresponding L*a*b* color to the given RGB color
 */
float * ColorUtilities::rgb2lab(float rgb[3]) {
    float rgb2[3];
    rgb2[0] = rgb[0] / 255;
    rgb2[1] = rgb[1] / 255;
    rgb2[2] = rgb[2] / 255;

    float * lab = color_conversion(rgb2, cv::COLOR_RGB2Lab);

    return lab;
}

/**
 * Convert L*a*b* colors to RGB color space
 * 
 * @param lab   the L*a*b* color to be converted
 * 
 * @return the corresponding RGB color to the given L*a*b* color
 */
float * ColorUtilities::lab2rgb(float lab[3]) {
    float * rgb = color_conversion(lab, cv::COLOR_Lab2RGB);

    rgb[0] *= 255;
    rgb[1] *= 255;
    rgb[2] *= 255;

    return rgb;
}

/**
 * Compute the CIEDE2000 color distance in the L*a*b* color space
 * 
 * @param lab1  the first color
 * @param lab2  the second color
 * @param kL    compensation for luminance (optional, default = 1.0) 
 * @param kC    compensation for chroma (optional, default = 1.0)
 * @param kH    compensation for hue (optional, default = 1.0)
 * 
 * @return the color difference between the two given colors
 */
float ColorUtilities::lab_ciede00(float lab1[3], float lab2[3], double kL,
        double kC, double kH) {
    float L1 = lab1[0];
    float a1 = lab1[1];
    float b1 = lab1[2];

    float L2 = lab2[0];
    float a2 = lab2[1];
    float b2 = lab2[2];

    double Cab1 = std::sqrt(a1 * a1 + b1 * b1);
    double Cab2 = std::sqrt(a2 * a2 + b2 * b2);

    double Cab = (Cab1 + Cab2) / 2.0;

    double G = 0.5 * (1.0 - std::sqrt(std::pow(Cab, 7.0) / (std::pow(Cab, 7.0)
            + std::pow(25.0, 7.0))));

    double ap1 = (1.0 + G) * a1; // aprime in paper
    double ap2 = (1.0 + G) * a2; // aprime in paper

    double Cp1 = std::sqrt(ap1 * ap1 + b1 * b1);
    double Cp2 = std::sqrt(ap2 * ap2 + b2 * b2);

    // Compute product of chromas
    double Cp_prod = (Cp2 * Cp1);

    double hp1 = 0;
    if ((std::abs(ap1) + std::abs(b1)) != 0.0) {
        hp1 = std::atan2(b1, ap1);
        // Ensure hue is between 0 and 2pi
        if (hp1 < 0)
            hp1 += 2.0 * M_PI; // rollover ones that come -ve
    }

    double hp2 = 0;
    if ((std::abs(ap2) + std::abs(b2)) != 0.0) {
        hp2 = std::atan2(b2, ap2);
        // Ensure hue is between 0 and 2pi
        if (hp2 < 0)
            hp2 += 2.0 * M_PI; // rollover ones that come -ve
    }

    double dL = (L2 - L1);
    double dC = (Cp2 - Cp1);

    // Computation of hue difference
    double dhp = (hp2 - hp1);
    if (dhp > M_PI)
        dhp -= 2.0 * M_PI;
    else if (dhp < -M_PI)
        dhp += 2.0 * M_PI;
    // set chroma difference to zero if the product of chromas is zero
    if (Cp_prod == 0.0)
        dhp = 0.0;

    // Note that the defining equations actually need
    // signed Hue and chroma differences which is different
    // from prior color difference formulae

    double dH = 2.0 * std::sqrt(Cp_prod) * std::sin(dhp / 2.0);
    //%dH2 = 4*Cpprod.*(sin(dhp/2)).^2;

    // weighting functions
    double Lp = (L2 + L1) / 2.0;
    double Cp = (Cp1 + Cp2) / 2.0;

    // Average Hue Computation
    // This is equivalent to that in the paper but simpler programmatically.
    // Note average hue is computed in radians and converted to degrees only
    // where needed
    double hp = (hp1 + hp2) / 2.0;
    // Identify positions for which abs hue diff exceeds 180 degrees
    if (std::abs(hp1 - hp2) > M_PI)
        hp -= M_PI;
    // rollover ones that come -ve
    if (hp < 0)
        hp += 2.0 * M_PI;

    // Check if one of the chroma values is zero, in which case set
    // mean hue to the sum which is equivalent to other value
    if (Cp_prod == 0.0)
        hp = hp1 + hp2;

    double Lpm502 = (Lp - 50.0) * (Lp - 50.0);

    double T = 1.0 - 0.17 * std::cos(hp - M_PI / 6.0) + 0.24 
            * std::cos(2.0 * hp) + 0.32 * std::cos(3.0 * hp + M_PI / 30.0)
            - 0.20 * std::cos(4.0 * hp - 63.0 * M_PI / 180.0);
    double dheta_rad = (30.0 * M_PI / 180.0)
            * std::exp(-std::pow(((180.0 / M_PI * hp - 275.0) / 25.0), 2.0));
    double Rc = 2.0 * std::sqrt(std::pow(Cp, 7.0)
            / (std::pow(Cp, 7.0) + std::pow(25.0, 7.0)));
    double kLSL = kL * (1.0 + 0.015 * Lpm502 / std::sqrt(20.0 + Lpm502));
    double kLSC = kC * (1.0 + 0.045 * Cp);
    double kHSH = kH * (1.0 + 0.015 * Cp * T);
    double RT = -std::sin(2.0 * dheta_rad) * Rc;

    // The CIE 00 color difference
    float delta_e = std::sqrt(std::pow((dL / kLSL), 2.0) 
            + std::pow((dC / kLSC), 2.0) + std::pow((dH / kHSH), 2.0)
            + RT * (dC / kLSC) * (dH / kHSH));

    return delta_e;
}

/**
 * Compute the euclidean color distance in the RGB color space
 * 
 * @param rgb1  the first color
 * @param rgb2  the second color
 *  
 * @return the color difference between the two given colors
 */
float ColorUtilities::rgb_eucl(float rgb1[3], float rgb2[3]) {
    float r1 = rgb1[0];
    float g1 = rgb1[1];
    float b1 = rgb1[2];
    float r2 = rgb2[0];
    float g2 = rgb2[1];
    float b2 = rgb2[2];

    float rd = std::pow(r1 - r2, 2);
    float gd = std::pow(g1 - g2, 2);
    float bd = std::pow(b1 - b2, 2);

    float delta_c = std::sqrt(rd + gd + bd);

    return delta_c;
}

/**
 * Perform an euclidean distance test in RGB space
 */
void ColorUtilities::rgb_test() {
    pcl::console::print_info("Performing RGB euclidean distance test...\n");
    float rgb1[] = {0, 0, 0};
    pcl::console::print_debug("Distance: %f\t(exp: 0)\n", rgb_eucl(rgb1, rgb1));

    float rgb2[] = {255, 255, 255};
    pcl::console::print_debug("Distance: %f\t(exp: 441.672943)\n",
            rgb_eucl(rgb1, rgb2));

    pcl::console::print_debug("Distance: %f\t(exp: 0)\n", rgb_eucl(rgb2, rgb2));

    float rgb3[] = {255, 0, 0};
    pcl::console::print_debug("Distance: %f\t(exp: 255)\n", rgb_eucl(rgb1, rgb3));

    float rgb4[] = {0, 255, 0};
    pcl::console::print_debug("Distance: %f\t(exp: 255)\n", rgb_eucl(rgb4, rgb1));

    float rgb5[] = {0, 255, 0};
    float rgb6[] = {255, 0, 255};
    pcl::console::print_debug("Distance: %f\t(exp: 441.672943)\n",
            rgb_eucl(rgb5, rgb6));

    float rgb7[] = {100, 20, 35};
    float rgb8[] = {104, 20, 32};
    pcl::console::print_debug("Distance: %f\t(exp: 5)\n", rgb_eucl(rgb7, rgb8));
}

/**
 * Perform a CIEDE2000 distance test in L*a*b* space
 */
void ColorUtilities::lab_test() {
    float err = 0;
    pcl::console::print_info("Performing LAB CIEDE2000 test... \n");
    err = std::max(err,
            ciede00_test(50.0000, 2.6772, -79.7751, 50.0000, 0.0000, -82.7485,
            2.0425));
    err = std::max(err,
            ciede00_test(50.0000, 3.1571, -77.2803, 50.0000, 0.0000, -82.7485,
            2.8615));
    err = std::max(err,
            ciede00_test(50.0000, 2.8361, -74.0200, 50.0000, 0.0000, -82.7485,
            3.4412));
    err = std::max(err,
            ciede00_test(50.0000, -1.3802, -84.2814, 50.0000, 0.0000, -82.7485,
            1.0000));
    err = std::max(err,
            ciede00_test(50.0000, -1.1848, -84.8006, 50.0000, 0.0000, -82.7485,
            1.0000));
    err = std::max(err,
            ciede00_test(50.0000, -0.9009, -85.5211, 50.0000, 0.0000, -82.7485,
            1.0000));
    err = std::max(err,
            ciede00_test(50.0000, 0.0000, 0.0000, 50.0000, -1.0000, 2.0000,
            2.3669));
    err = std::max(err,
            ciede00_test(50.0000, -1.0000, 2.0000, 50.0000, 0.0000, 0.0000,
            2.3669));
    err = std::max(err,
            ciede00_test(50.0000, 2.4900, -0.0010, 50.0000, -2.4900, 0.0009,
            7.1792));
    err = std::max(err,
            ciede00_test(50.0000, 2.4900, -0.0010, 50.0000, -2.4900, 0.0010,
            7.1792));
    err = std::max(err,
            ciede00_test(50.0000, 2.4900, -0.0010, 50.0000, -2.4900, 0.0011,
            7.2195));
    err = std::max(err,
            ciede00_test(50.0000, 2.4900, -0.0010, 50.0000, -2.4900, 0.0012,
            7.2195));
    err = std::max(err,
            ciede00_test(50.0000, -0.0010, 2.4900, 50.0000, 0.0009, -2.4900,
            4.8045));
    err = std::max(err,
            ciede00_test(50.0000, -0.0010, 2.4900, 50.0000, 0.0010, -2.4900,
            4.8045));
    err = std::max(err,
            ciede00_test(50.0000, -0.0010, 2.4900, 50.0000, 0.0011, -2.4900,
            4.7461));
    err = std::max(err,
            ciede00_test(50.0000, 2.5000, 0.0000, 50.0000, 0.0000, -2.5000,
            4.3065));
    err = std::max(err,
            ciede00_test(50.0000, 2.5000, 0.0000, 73.0000, 25.0000, -18.0000,
            27.1492));
    err = std::max(err,
            ciede00_test(50.0000, 2.5000, 0.0000, 61.0000, -5.0000, 29.0000,
            22.8977));
    err = std::max(err,
            ciede00_test(50.0000, 2.5000, 0.0000, 56.0000, -27.0000, -3.0000,
            31.9030));
    err = std::max(err,
            ciede00_test(50.0000, 2.5000, 0.0000, 58.0000, 24.0000, 15.0000,
            19.4535));
    err = std::max(err,
            ciede00_test(50.0000, 2.5000, 0.0000, 50.0000, 3.1736, 0.5854,
            1.0000));
    err = std::max(err,
            ciede00_test(50.0000, 2.5000, 0.0000, 50.0000, 3.2972, 0.0000,
            1.0000));
    err = std::max(err,
            ciede00_test(50.0000, 2.5000, 0.0000, 50.0000, 1.8634, 0.5757,
            1.0000));
    err = std::max(err,
            ciede00_test(50.0000, 2.5000, 0.0000, 50.0000, 3.2592, 0.3350,
            1.0000));
    err = std::max(err,
            ciede00_test(60.2574, -34.0099, 36.2677, 60.4626, -34.1751, 39.4387,
            1.2644));
    err = std::max(err,
            ciede00_test(63.0109, -31.0961, -5.8663, 62.8187, -29.7946, -4.0864,
            1.2630));
    err = std::max(err,
            ciede00_test(61.2901, 3.7196, -5.3901, 61.4292, 2.2480, -4.9620,
            1.8731));
    err = std::max(err,
            ciede00_test(35.0831, -44.1164, 3.7933, 35.0232, -40.0716, 1.5901,
            1.8645));
    err = std::max(err,
            ciede00_test(22.7233, 20.0904, -46.6940, 23.0331, 14.9730, -42.5619,
            2.0373));
    err = std::max(err,
            ciede00_test(36.4612, 47.8580, 18.3852, 36.2715, 50.5065, 21.2231,
            1.4146));
    err = std::max(err,
            ciede00_test(90.8027, -2.0831, 1.4410, 91.1528, -1.6435, 0.0447,
            1.4441));
    err = std::max(err,
            ciede00_test(90.9257, -0.5406, -0.9208, 88.6381, -0.8985, -0.7239,
            1.5381));
    err = std::max(err,
            ciede00_test(6.7747, -0.2908, -2.4247, 5.8714, -0.0985, -2.2286,
            0.6377));
    err = std::max(err,
            ciede00_test(2.0776, 0.0795, -1.1350, 0.9033, -0.0636, -0.5514,
            0.9082));
    pcl::console::print_debug("Lab test max error: %f\n", err);
}

/**
 * Perform a color conversion test from RGB to L*a*b* space
 */
void ColorUtilities::convert_test() {
    pcl::console::print_info("Performing RGB-LAB Conversion test...\n");
    
    float rgb1[] = {123, 10, 200};
    float * lab1 = rgb2lab(rgb1);
    float * rgb1c = lab2rgb(lab1);
    pcl::console::print_debug(
            "rgb[%f, %f, %f]\t=>\tlab[%f, %f, %f]\t=>\trgb[%f, %f, %f]\n",
            rgb1[0], rgb1[1], rgb1[2], lab1[0], lab1[1], lab1[2], rgb1c[0],
            rgb1c[1], rgb1c[2]);
    
    float rgb2[] = {0, 0, 0};
    float * lab2 = rgb2lab(rgb2);
    float * rgb2c = lab2rgb(lab2);
    pcl::console::print_debug(
            "rgb[%f, %f, %f]\t=>\tlab[%f, %f, %f]\t=>\trgb[%f, %f, %f]\n",
            rgb2[0], rgb2[1], rgb2[2], lab2[0], lab2[1], lab2[2], rgb2c[0],
            rgb2c[1], rgb2c[2]);
    
    float rgb3[] = {255, 255, 255};
    float * lab3 = rgb2lab(rgb3);
    float * rgb3c = lab2rgb(lab3);
    pcl::console::print_debug(
            "rgb[%f, %f, %f]\t=>\tlab[%f, %f, %f]\t=>\trgb[%f, %f, %f]\n",
            rgb3[0], rgb3[1], rgb3[2], lab3[0], lab3[1], lab3[2], rgb3c[0],
            rgb3c[1], rgb3c[2]);
    
    float rgb4[] = {255, 255, 0};
    float * lab4 = rgb2lab(rgb4);
    float * rgb4c = lab2rgb(lab4);
    pcl::console::print_debug(
            "rgb[%f, %f, %f]\t=>\tlab[%f, %f, %f]\t=>\trgb[%f, %f, %f]\n",
            rgb4[0], rgb4[1], rgb4[2], lab4[0], lab4[1], lab4[2], rgb4c[0],
            rgb4c[1], rgb4c[2]);
}
