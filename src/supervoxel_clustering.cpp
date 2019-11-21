/*
 * supervoxel_clustering.cpp
 *
 *  Created on: 10/05/2015
 *      Author: Francesco Verdoja <francesco.verdoja@aalto.fi>
 *    Based on: example_supervoxels.cpp from PointCloudLibrary:
 *              https://github.com/PointCloudLibrary/pcl/commits/master/
 *                      examples/segmentation/example_supervoxels.cpp
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

#include <pcl/common/time.h>
#include <pcl/console/parse.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/png_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/supervoxel_clustering.h>

#include <vtkImageReader2Factory.h>
#include <vtkImageReader2.h>
#include <vtkImageData.h>
#include <vtkImageFlip.h>
#include <vtkPolyLine.h>

//#include <boost/filesystem.hpp>

#include "supervoxel_clustering/clustering.h"
#include "supervoxel_clustering/testing.h"

using namespace boost;
using namespace pcl;

typedef PointXYZRGBA PointT;
typedef PointCloud<PointT> PointCloudT;
typedef PointNormal PointNT;
typedef PointCloud<PointNT> PointNCloudT;
typedef PointXYZL PointLT;
typedef PointCloud<PointLT> PointLCloudT;
typedef PointXYZRGBL PointLCT;
typedef PointCloud<PointLCT> PointLCCloudT;

bool show_voxel_centroids = false;
bool show_segmentation = true;
bool show_supervoxels = false;
bool show_supervoxel_normals = false;
bool show_graph = true;
bool show_help = false;
float start_thresh = 0.8;
float end_thresh = 1;
float step_thresh = 0.005;

void keyboard_callback(const visualization::KeyboardEvent& event, void*) {
    int key = event.getKeyCode();

    if (event.keyUp())
        switch (key) {
            case (int) '1':
                show_voxel_centroids = !show_voxel_centroids;
                break;
            case (int) '2':
                show_segmentation = !show_segmentation;
                break;
            case (int) '3':
                show_graph = !show_graph;
                break;
            case (int) '4':
                show_supervoxel_normals = !show_supervoxel_normals;
                break;
            case (int) '0':
                show_supervoxels = !show_supervoxels;
                break;
            case (int) 'h':
            case (int) 'H':
                show_help = !show_help;
                break;
            default:
                break;
        }
}

inline bool isNan(float f) {
    return f != f;
}

void manageAllPerformances(
        std::vector<std::map<float, performanceSet> > all_performances,
        std::string filename);
void printBestPerformances(std::vector<performanceSet> best_performances);

void addSupervoxelConnectionsToViewer(PointT &supervoxel_center,
        PointCloudT &adjacent_supervoxel_centers, std::string supervoxel_name,
        shared_ptr<visualization::PCLVisualizer> & viewer);

void visualize(std::map<uint32_t, Supervoxel<PointT>::Ptr> supervoxel_clusters,
        PointCloudT::Ptr colored_cloud, PointCloudT::Ptr segm_cloud,
        PointCloudT::Ptr truth_cloud, PointNCloudT::Ptr normal_cloud,
        std::multimap<uint32_t, uint32_t> adjacency);

void printText(shared_ptr<visualization::PCLVisualizer> viewer);
void removeText(shared_ptr<visualization::PCLVisualizer> viewer);

int main(int argc, char ** argv) {
    if (argc < 3) {
        console::print_info(
                "Syntax is: "
                "%s {-d <direcory-of-pcd-files> OR -p <pcd-file>} [arguments] \n"
                "\n\t"
                "SUPERVOXEL optional arguments: \n\t"
                " -v <voxel-resolution>          (default: 0.008) \n\t"
                " -s <seed-resolution>           (default: 0.08) \n\t"
                " -c <color-weight>              (default: 0.2) \n\t"
                " -z <spatial-weight>            (default: 0.4) \n\t"
                " -n <normal-weight>             (default: 1.0) \n\t"
                "\n\t"
                "SEGMENTATION optional arguments: \n\t"
                " -t <threshold>                 (default: auto)\n\t"
                " --RGB                          (uses the RGB color space for "
                "measuring the color distance; if not given, L*A*B* color "
                "space is used) \n\t"
                " --CVX                          (uses the convexity criterion "
                "to weigh the geometric distance; if not given, convexity is "
                "not considered) \n\t"
                " --ML [manual-lambda] *         (uses Manual Lambda as "
                "merging criterion; if no parameter is given, lambda=0.5 is "
                "used) \n\t"
                " --AL                 *         (uses Adaptive lambda as "
                "merging criterion) \n\t"
                " --EQ [bins-number]   *         (uses Equalization as merging "
                "criterion; if no parameter is given, 200 bins are used) \n\t"
                "  * please note that only one of these arguments can be passed "
                "at the same time \n\t"
                "\n\t"
                "OTHER optional arguments: \n\t"
                " -r <label-to-be-removed>       (if ground-truth is provided, "
                "removes all points with the given label from the "
                "ground-truth)\n\t"
                " -f <test-results-filename>     (uses the given name as "
                "filename for all test results files; if not given, 'test' is "
                "going to be used)\n\t"
                " --NT                           (disables use of single "
                "camera transform) \n\t"
                " --V                            (verbose) \n",
                argv[0]);
        return (1);
    }

    ////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////
    ////// Input handling
    ////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////

    bool verbose = console::find_switch(argc, argv, "--V");
    if (verbose) {
        console::setVerbosityLevel(console::L_DEBUG);
    }

    bool disable_transform = console::find_switch(argc, argv, "--NT");

    std::string test_filename = "test";
    if (console::find_switch(argc, argv, "-f"))
        console::parse(argc, argv, "-f", test_filename);

    // Input parameters
    PointCloudT::Ptr cloud = make_shared<PointCloudT>();
    PointLCloudT::Ptr truth_cloud = make_shared<PointLCloudT>();
    PointLCCloudT::Ptr input_cloud = make_shared<PointLCCloudT>();

    std::string path;
    std::vector<std::string> file_list;

    bool directory_specified = console::find_switch(argc, argv, "-d");
    bool pcd_file_specified = console::find_switch(argc, argv, "-p");

    if (directory_specified) {
        console::parse(argc, argv, "-d", path);
        console::print_info("Counting files in directory...\n");
        if (!filesystem::exists(path) || !filesystem::is_directory(path)) {
            console::print_error(
                    "Specified directory doesn't exists or can't be opened\n");
            return (1);
        }

        filesystem::recursive_directory_iterator it(path);
        filesystem::recursive_directory_iterator end_it;
        for (; it != end_it; ++it) {
            if (filesystem::is_regular_file(*it)
                    && it->path().extension() == ".pcd") {
                file_list.push_back(it->path().string());
                console::print_debug("File found: %s\n", it->path().c_str());
            }
        }
        console::print_info("Found %d files\n", file_list.size());
    } else if (pcd_file_specified) {
        console::parse(argc, argv, "-p", path);
        file_list.push_back(path);
    } else {
        console::print_error("No input file or directory specified\n");
        return (1);
    }

    bool thresh_specified = console::find_switch(argc, argv, "-t");
    float thresh = 0;

    if (thresh_specified) {
        console::parse_argument(argc, argv, "-t", thresh);
        console::print_debug("Using threshold: %f\n", thresh);
    } else {
        console::print_debug("Using automatic threshold\n");
    }

    // Supervoxel segmentation parameters
    float voxel_resolution = 0.008f;
    bool voxel_res_specified = console::find_switch(argc, argv, "-v");
    if (voxel_res_specified)
        console::parse(argc, argv, "-v", voxel_resolution);

    float seed_resolution = 0.08f;
    bool seed_res_specified = console::find_switch(argc, argv, "-s");
    if (seed_res_specified)
        console::parse(argc, argv, "-s", seed_resolution);

    float color_importance = 0.2f;
    if (console::find_switch(argc, argv, "-c"))
        console::parse(argc, argv, "-c", color_importance);

    float spatial_importance = 0.4f;
    if (console::find_switch(argc, argv, "-z"))
        console::parse(argc, argv, "-z", spatial_importance);

    float normal_importance = 1.0f;
    if (console::find_switch(argc, argv, "-n"))
        console::parse(argc, argv, "-n", normal_importance);

    // Segmentation parameters
    bool rgb_color_space_specified = console::find_switch(argc, argv, "--RGB");
    bool convexity_specified = console::find_switch(argc, argv, "--CVX");
    bool manual_lambda_specified = console::find_switch(argc, argv, "--ML");
    bool adapt_lambda_specified = console::find_switch(argc, argv, "--AL");
    bool equalization_specified = console::find_switch(argc, argv, "--EQ");
    if (!(manual_lambda_specified || adapt_lambda_specified
            || equalization_specified)) {
        adapt_lambda_specified = true;
        console::print_debug("No merging criterion specified, Adaptive Lambda "
                "is going to be used\n");
    } else if (!(manual_lambda_specified ^ adapt_lambda_specified
            ^ equalization_specified)) {
        console::print_error("Only one parameter between --ML --AL and --EQ "
                "can be specified at a time\n");
        return (1);
    }

    float lambda = 0;
    if (manual_lambda_specified)
        console::parse_argument(argc, argv, "--ML", lambda);

    int bin_num = 0;
    if (equalization_specified)
        console::parse_argument(argc, argv, "--EQ", bin_num);

    bool remove_label = console::find_switch(argc, argv, "-r");
    uint32_t label_to_be_removed = 0;
    if (remove_label)
        console::parse_argument(argc, argv, "-r", label_to_be_removed);

    std::vector<performanceSet> best_performances;
    std::vector<std::map<float, performanceSet> > all_performances;
    std::vector<std::string>::iterator file_it = file_list.begin();
    for (; file_it != file_list.end(); ++file_it) {

        ////////////////////////////////////////////////////////////
        ////// File reading
        ////////////////////////////////////////////////////////////

        bool has_label = true; //TODO should be false

        console::print_info("Loading pointcloud from PCD file '%s'...\n",
                file_it->c_str());
        pcl::io::loadPCDFile(*file_it, *input_cloud);
        PointLCCloudT::Ptr cloud_temp = make_shared<PointLCCloudT>();
        PointLCCloudT::iterator cloud_itr = input_cloud->begin();
        for (; cloud_itr != input_cloud->end(); ++cloud_itr) {
            if (cloud_itr->z < 0) {
                console::print_debug(
                        "Found point with z<0, setting to absolute value\n");
                cloud_itr->z = std::abs(cloud_itr->z);
            }
            /*
             * TODO 
             * this doesn't work if label = 0 exists and is the one to be
             * removed
             */
            if (!has_label && cloud_itr->label != 0) {
                console::print_debug("Found label data, evaluation is going "
                        "to be performed\n");
                has_label = true;
            }
            if (!has_label || !remove_label
                    || (cloud_itr->label != label_to_be_removed
                    && !isNan(cloud_itr->z))) {
                cloud_temp->push_back(*cloud_itr);
            }
        }

        copyPointCloud(*cloud_temp, *cloud);
        copyPointCloud(*cloud_temp, *truth_cloud);

        console::print_info("Pointcloud loaded\n");

        ////////////////////////////////////////////////////////////
        ////// Supervoxel generation
        ////////////////////////////////////////////////////////////

        SupervoxelClustering<PointT> super(voxel_resolution, seed_resolution);
        super.setUseSingleCameraTransform(!disable_transform);
        super.setInputCloud(cloud);
        super.setColorImportance(color_importance);
        super.setSpatialImportance(spatial_importance);
        super.setNormalImportance(normal_importance);
        std::map<uint32_t, Supervoxel<PointT>::Ptr> supervoxel_clusters;

        console::print_info("Extracting supervoxels...\n");
        super.extract(supervoxel_clusters);
        console::print_info("Found %d supervoxels\n",
                supervoxel_clusters.size());
        PointCloudT::Ptr voxel_centroid_cloud = super.getVoxelCentroidCloud();
        PointNCloudT::Ptr sv_normal_cloud = super.makeSupervoxelNormalCloud(
                supervoxel_clusters);
        PointLCloudT::Ptr full_labeled_cloud = super.getLabeledCloud();

        console::print_info("Getting supervoxel adjacency...\n");
        std::multimap<uint32_t, uint32_t> label_adjacency;
        super.getSupervoxelAdjacency(label_adjacency);

        std::map<uint32_t, Supervoxel<PointT>::Ptr> refined_supervoxel_clusters;
        console::print_info("Refining supervoxels...\n");
        super.refineSupervoxels(3, refined_supervoxel_clusters);

        PointNCloudT::Ptr refined_sv_normal_cloud =
                super.makeSupervoxelNormalCloud(refined_supervoxel_clusters);
        PointLCloudT::Ptr refined_full_labeled_cloud = super.getLabeledCloud();
        
        console::print_info(
                "Constructing Boost Graph Library Adjacency List...\n");
        typedef adjacency_list<setS, setS, undirectedS, uint32_t, float>
                VoxelAdjacencyList;
        typedef VoxelAdjacencyList::vertex_descriptor VoxelID;
        typedef VoxelAdjacencyList::edge_descriptor EdgeID;
        VoxelAdjacencyList supervoxel_adjacency_list;
        super.getSupervoxelAdjacencyList(supervoxel_adjacency_list);

        // Voxelizing ground truth cloud
        PointCloudT::Ptr colored_truth_cloud = Clustering::label2color(
                truth_cloud);
        SupervoxelClustering<PointT> super_label(voxel_resolution,
                seed_resolution);
        super_label.setUseSingleCameraTransform(!disable_transform);
        super_label.setInputCloud(colored_truth_cloud);
        super_label.setColorImportance(color_importance);
        super_label.setSpatialImportance(spatial_importance);
        super_label.setNormalImportance(normal_importance);
        std::map<uint32_t, Supervoxel<PointT>::Ptr> supervoxel_label_clusters;
        super_label.extract(supervoxel_label_clusters);
        colored_truth_cloud = super_label.getVoxelCentroidCloud();
        truth_cloud = Clustering::color2label(colored_truth_cloud);
        colored_truth_cloud = Clustering::label2color(truth_cloud);

        ////////////////////////////////////////////////////////////
        ////// Segmentation
        ////////////////////////////////////////////////////////////

        console::print_info("Segmentation initialization...\n");

        Clustering segmentation;
        if (rgb_color_space_specified)
            segmentation.set_delta_c(RGB_EUCL);

        if (convexity_specified)
            segmentation.set_delta_g(CONVEX_NORMALS_DIFF);

        if (manual_lambda_specified) {
            segmentation.set_merging(MANUAL_LAMBDA);
            if (lambda != 0)
                segmentation.set_lambda(lambda);
        } else if (equalization_specified) {
            segmentation.set_merging(EQUALIZATION);
            if (bin_num != 0)
                segmentation.set_bins_num(bin_num);
        }
        segmentation.set_initialstate(supervoxel_clusters, label_adjacency);
        if (manual_lambda_specified || adapt_lambda_specified)
            console::print_debug("Lambda: %f\n", segmentation.get_lambda());

        if (!thresh_specified) {
            std::map<float, performanceSet> all = segmentation.all_thresh(
                    truth_cloud, start_thresh, end_thresh, step_thresh);
            all_performances.push_back(all);
            std::pair<float, performanceSet> best = segmentation.best_thresh(
                    all);
            console::print_info(
                    "Using best threshold: %f (F-score %f, voi %f)\n",
                    best.first, best.second.fscore, best.second.voi);
            thresh = best.first;
        }

        console::print_info(
                "Initialization complete\nStarting clustering...\n");

        segmentation.cluster(thresh);
        console::print_info("Clustering complete\n");
        std::pair<ClusteringT, AdjacencyMapT> s =
                segmentation.get_currentstate();

        PointCloudT::Ptr colored_voxel_cloud = segmentation.get_colored_cloud();
        label_adjacency = s.second;

        ////////////////////////////////////////////////////////////
        ////// Testing
        ////////////////////////////////////////////////////////////

        console::print_info("Initializing testing suite...\n");
        Testing test(segmentation.get_labeled_cloud(), truth_cloud);
        best_performances.push_back(test.eval_performance());

        ////////////////////////////////////////////////////////////
        ////// Visualization
        ////////////////////////////////////////////////////////////

        if (file_list.size() == 1) {
            console::print_info("Loading visualization...\n");
            visualize(supervoxel_clusters, voxel_centroid_cloud,
                    colored_voxel_cloud, colored_truth_cloud,
                    refined_sv_normal_cloud, label_adjacency);
        }
    }

    manageAllPerformances(all_performances, test_filename);

    printBestPerformances(best_performances);

    return (0);
}

void manageAllPerformances(
        std::vector<std::map<float, performanceSet> > all_performances,
        std::string filename) {
    ofstream file_voi((filename + "_voi.csv").c_str());
    ofstream file_prec((filename + "_precision.csv").c_str());
    ofstream file_recall((filename + "_recall.csv").c_str());
    ofstream file_fscore((filename + "_fscore.csv").c_str());
    ofstream file_wov((filename + "_wov.csv").c_str());
    ofstream file_fpr((filename + "_fpr.csv").c_str());
    ofstream file_fnr((filename + "_fnr.csv").c_str());

    std::vector<std::map<float, performanceSet> >::iterator p_it =
            all_performances.begin();
    for (; p_it != all_performances.end(); ++p_it) {
        std::map<float, performanceSet>::iterator m_it = p_it->begin();
        for (; m_it != p_it->end(); ++m_it) {
            file_voi << m_it->second.voi << ";";
            file_prec << m_it->second.precision << ";";
            file_recall << m_it->second.recall << ";";
            file_fscore << m_it->second.fscore << ";";
            file_wov << m_it->second.wov << ";";
            file_fpr << m_it->second.fpr << ";";
            file_fnr << m_it->second.fnr << ";";
        }
        file_voi << "\n";
        file_prec << "\n";
        file_recall << "\n";
        file_fscore << "\n";
        file_wov << "\n";
        file_fpr << "\n";
        file_fnr << "\n";

    }
    file_voi.close();
    file_prec.close();
    file_recall.close();
    file_fscore.close();
    file_wov.close();
    file_fpr.close();
    file_fnr.close();
}

void printBestPerformances(std::vector<performanceSet> best_performances) {
    if (best_performances.size() == 1) {
        performanceSet p = best_performances.back();
        console::print_info(
                "Scores:\nVOI\t%f\nPrec.\t%f\nRecall\t%f\nF-score\t%f\n"
                "WOv\t%f\nFPR\t%f\nFNR\t%f\n",
                p.voi, p.precision, p.recall, p.fscore, p.wov, p.fpr, p.fnr);
    } else {
        std::vector<performanceSet>::iterator p_it = best_performances.begin();
        float mean_v = 0;
        float mean_p = 0;
        float mean_r = 0;
        float mean_f = 0;
        float mean_w = 0;
        float mean_pr = 0;
        float mean_nr = 0;
        int count = 0;
        for (; p_it != best_performances.end(); ++p_it) {
            count++;
            mean_v = mean_v + (1 / count) * (p_it->voi - mean_v);
            mean_p = mean_p + (1 / count) * (p_it->precision - mean_p);
            mean_r = mean_r + (1 / count) * (p_it->recall - mean_r);
            mean_f = mean_f + (1 / count) * (p_it->fscore - mean_f);
            mean_w = mean_w + (1 / count) * (p_it->wov - mean_w);
            mean_pr = mean_pr + (1 / count) * (p_it->fpr - mean_pr);
            mean_nr = mean_nr + (1 / count) * (p_it->fnr - mean_nr);
            console::print_debug(
                    "Scores:\nVOI\t%f\nPrec.\t%f\nRecall\t%f\nF-score\t%f\n"
                    "WOv\t%f\nFPR\t%f\nFNR\t%f\n",
                    p_it->voi, p_it->precision, p_it->recall, p_it->fscore,
                    p_it->wov, p_it->fpr, p_it->fnr);
        }
        console::print_info(
                "Average scores:\nVOI\t%f\nPrec.\t%f\nRecall\t%f\nF-score\t%f\n"
                "WOv\t%f\nFPR\t%f\nFNR\t%f\n",
                mean_v, mean_p, mean_r, mean_f, mean_w, mean_pr, mean_nr);
    }
}

void addSupervoxelConnectionsToViewer(PointT &supervoxel_center,
        PointCloudT &adjacent_supervoxel_centers, std::string supervoxel_name,
        shared_ptr<visualization::PCLVisualizer> & viewer) {
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New();
    vtkSmartPointer<vtkPolyLine> polyLine = vtkSmartPointer<vtkPolyLine>::New();

    // Iterate through all adjacent points, and add a center point to adjacent 
    // point pair
    PointCloudT::iterator adjacent_itr = adjacent_supervoxel_centers.begin();
    for (; adjacent_itr != adjacent_supervoxel_centers.end(); ++adjacent_itr) {
        points->InsertNextPoint(supervoxel_center.data);
        points->InsertNextPoint(adjacent_itr->data);
    }
    // Create a polydata to store everything in
    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
    // Add the points to the dataset
    polyData->SetPoints(points);
    polyLine->GetPointIds()->SetNumberOfIds(points->GetNumberOfPoints());
    for (unsigned int i = 0; i < points->GetNumberOfPoints(); i++)
        polyLine->GetPointIds()->SetId(i, i);
    cells->InsertNextCell(polyLine);
    // Add the lines to the dataset
    polyData->SetLines(cells);
    viewer->addModelFromPolyData(polyData, supervoxel_name);
}

void visualize(std::map<uint32_t, Supervoxel<PointT>::Ptr> supervoxel_clusters,
        PointCloudT::Ptr colored_cloud, PointCloudT::Ptr segm_cloud,
        PointCloudT::Ptr truth_cloud, PointNCloudT::Ptr normal_cloud,
        std::multimap<uint32_t, uint32_t> adjacency) {
    shared_ptr<visualization::PCLVisualizer> viewer(
            new visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->registerKeyboardCallback(keyboard_callback, 0);

    bool graph_added = false;
    std::vector<std::string> poly_names;
    console::print_info("Loading viewer...\n");
    while (!viewer->wasStopped()) {
        if (show_voxel_centroids) {
            if (!viewer->updatePointCloud(colored_cloud, "voxel centroids"))
                viewer->addPointCloud(colored_cloud, "voxel centroids");
            viewer->setPointCloudRenderingProperties(
                    visualization::PCL_VISUALIZER_POINT_SIZE, 2.0,
                    "voxel centroids");
            if (show_segmentation)
                viewer->setPointCloudRenderingProperties(
                    visualization::PCL_VISUALIZER_OPACITY, 0.5,
                    "voxel centroids");
            else
                viewer->setPointCloudRenderingProperties(
                    visualization::PCL_VISUALIZER_OPACITY, 1.0,
                    "voxel centroids");
        } else {
            viewer->removePointCloud("voxel centroids");
        }

        if (show_segmentation) {
            if (!viewer->updatePointCloud(
                    (show_supervoxels) ? truth_cloud : segm_cloud,
                    "colored voxels"))
                viewer->addPointCloud(
                    (show_supervoxels) ? truth_cloud : segm_cloud,
                    "colored voxels");
            viewer->setPointCloudRenderingProperties(
                    visualization::PCL_VISUALIZER_POINT_SIZE, 2.0,
                    "colored voxels");
            viewer->setPointCloudRenderingProperties(
                    visualization::PCL_VISUALIZER_OPACITY, 0.9,
                    "colored voxels");
        } else {
            viewer->removePointCloud("colored voxels");
        }

        if (show_supervoxel_normals) {
            viewer->addPointCloudNormals<PointNormal>(normal_cloud, 1, 0.05f,
                    "supervoxel_normals");
        } else if (!show_supervoxel_normals) {
            viewer->removePointCloud("supervoxel_normals");
        }

        if (show_graph && !graph_added) {
            poly_names.clear();
            std::multimap<uint32_t, uint32_t>::iterator label_itr =
                    adjacency.begin();
            for (; label_itr != adjacency.end();) {
                // First get the label
                uint32_t supervoxel_label = label_itr->first;
                // Now get the supervoxel corresponding to the label
                Supervoxel<PointT>::Ptr supervoxel = supervoxel_clusters.at(
                        supervoxel_label);
                // Now we need to iterate through the adjacent supervoxels and
                // make a point cloud of them
                PointCloudT adjacent_supervoxel_centers;
                std::multimap<uint32_t, uint32_t>::iterator adjacent_itr =
                        adjacency.equal_range(supervoxel_label).first;
                for (;
                        adjacent_itr
                        != adjacency.equal_range(supervoxel_label).second;
                        ++adjacent_itr) {
                    Supervoxel<PointT>::Ptr neighbor_supervoxel =
                            supervoxel_clusters.at(adjacent_itr->second);
                    adjacent_supervoxel_centers.push_back(
                            neighbor_supervoxel->centroid_);
                }
                // Now we make a name for this polygon
                std::stringstream ss;
                ss << "supervoxel_" << supervoxel_label;
                poly_names.push_back(ss.str());
                addSupervoxelConnectionsToViewer(supervoxel->centroid_,
                        adjacent_supervoxel_centers, ss.str(), viewer);
                // Move iterator forward to next label
                label_itr = adjacency.upper_bound(supervoxel_label);
            }

            graph_added = true;
        } else if (!show_graph && graph_added) {
            for (std::vector<std::string>::iterator name_itr =
                    poly_names.begin(); name_itr != poly_names.end();
                    ++name_itr) {
                viewer->removeShape(*name_itr);
            }
            graph_added = false;
        }

        if (show_help) {
            viewer->removeShape("help_text");
            printText(viewer);
        } else {
            removeText(viewer);
            if (!viewer->updateText("Press h to show help", 5, 10, 12, 1.0, 1.0,
                    1.0, "help_text"))
                viewer->addText("Press h to show help", 5, 10, 12, 1.0, 1.0,
                    1.0, "help_text");
        }

        viewer->spinOnce(100);
        this_thread::sleep(posix_time::microseconds(100000));

    }
}

void printText(shared_ptr<visualization::PCLVisualizer> viewer) {
    std::string on_str = "ON";
    std::string off_str = "OFF";
    std::string temp =
            "Press (1-n) to show different elements, (h) to hide this";
    if (!viewer->updateText(temp, 5, 72, 12, 1.0, 1.0, 1.0, "hud_text"))
        viewer->addText(temp, 5, 72, 12, 1.0, 1.0, 1.0, "hud_text");

    temp = "(1) Voxels currently "
            + ((show_voxel_centroids) ? on_str : off_str);
    if (!viewer->updateText(temp, 5, 60, 10, 1.0, 1.0, 1.0, "voxel_text"))
        viewer->addText(temp, 5, 60, 10, 1.0, 1.0, 1.0, "voxel_text");

    temp = "(2) Segmentation currently "
            + ((show_segmentation) ? on_str : off_str);
    if (!viewer->updateText(temp, 5, 50, 10, 1.0, 1.0, 1.0, "supervoxel_text"))
        viewer->addText(temp, 5, 50, 10, 1.0, 1.0, 1.0, "supervoxel_text");

    temp = "(3) Graph currently " + ((show_graph) ? on_str : off_str);
    if (!viewer->updateText(temp, 5, 40, 10, 1.0, 1.0, 1.0, "graph_text"))
        viewer->addText(temp, 5, 40, 10, 1.0, 1.0, 1.0, "graph_text");

    temp = "(4) Supervoxel Normals currently "
            + ((show_supervoxel_normals) ? on_str : off_str);
    if (!viewer->updateText(temp, 5, 30, 10, 1.0, 1.0, 1.0,
            "supervoxel_normals_text"))
        viewer->addText(temp, 5, 30, 10, 1.0, 1.0, 1.0,
            "supervoxel_normals_text");

    temp = "(0) Toggle between supervoxels and segmentation: currently showing "
            + std::string((show_supervoxels) ? "SUPERVOXELS" : "SEGMENTATION");
    if (!viewer->updateText(temp, 5, 7, 10, 1.0, 1.0, 1.0, "refined_text"))
        viewer->addText(temp, 5, 7, 10, 1.0, 1.0, 1.0, "refined_text");

}

void removeText(shared_ptr<visualization::PCLVisualizer> viewer) {
    viewer->removeShape("hud_text");
    viewer->removeShape("voxel_text");
    viewer->removeShape("supervoxel_text");
    viewer->removeShape("graph_text");
    viewer->removeShape("voxel_normals_text");
    viewer->removeShape("supervoxel_normals_text");
    viewer->removeShape("refined_text");
}
