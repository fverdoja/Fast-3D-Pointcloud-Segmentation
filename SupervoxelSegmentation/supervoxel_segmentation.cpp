/*
 * supervoxel_segmentation.cpp
 *
 *  Created on: 10/05/2015
 *      Author: Francesco Verdoja <verdoja@di.unito.it>
 *    Based on: example_supervoxels.cpp from PointCloudLibrary:
 *              https://github.com/PointCloudLibrary/pcl/commits/master/examples/segmentation/example_supervoxels.cpp
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

#include "Clustering.h"

using namespace pcl;

// Types
typedef PointXYZRGBA PointT;
typedef PointCloud<PointT> PointCloudT;
typedef PointNormal PointNT;
typedef PointCloud<PointNT> PointNCloudT;
typedef PointXYZL PointLT;
typedef PointCloud<PointLT> PointLCloudT;

bool show_voxel_centroids = false;
bool show_segmentation = true;
bool show_supervoxels = false;
bool show_supervoxel_normals = false;
bool show_graph = true;
bool show_help = false;

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

void addSupervoxelConnectionsToViewer(PointT &supervoxel_center,
		PointCloudT &adjacent_supervoxel_centers, std::string supervoxel_name,
		boost::shared_ptr<visualization::PCLVisualizer> & viewer);

void printText(boost::shared_ptr<visualization::PCLVisualizer> viewer);
void removeText(boost::shared_ptr<visualization::PCLVisualizer> viewer);

int main(int argc, char ** argv) {
	if (argc < 3) {
		console::print_info(
				"Syntax is: %s {-p <pcd-file> OR -y <ply-file> OR -r <rgb-file> -d <depth-file>} {-t <threshold>} [options] \n"
						"\n\t"
						"SUPERVOXEL optional options: \n\t"
						" -v <voxel-resolution>          (default: 0.008) \n\t"
						" -s <seed-resolution>           (default: 0.08) \n\t"
						" -c <color-weight>              (default: 0.2) \n\t"
						" -z <spatial-weight>            (default: 0.4) \n\t"
						" -n <normal-weight>             (default: 1.0) \n\t"
						"\n\t"
						"SEGMENTATION optional options: \n\t"
						" --RGB                          (uses the RGB color space for measuring the color distance; if not given, L*A*B* color space is used) \n\t"
						" --ML [manual-lambda] *         (uses Manual Lambda as merging criterion, if no parameter is given lambda=0.5 is used) \n\t"
						" --AL                 *         (uses Adaptive lambda as merging criterion) \n\t"
						" --EQ [bins-number]   *         (uses Equalization as merging criterion, if no parameter is given 200 bins are used) \n\t"
						"  * please note that only one of this options can be passed at the same time \n\t"
						"\n\t"
						"OUTPUT optional options: \n\t"
						" -o <output-file>               (default filename: test_output.png) \n\t"
						" -O <refined-output-file>       (default filename: refined_test_output.png) \n\t"
						" -l <output-label-file>         (default filename: test_output_labels.png) \n\t"
						" -L <refined-output-label-file> (default filename: refined_test_output_labels.png) \n\t"
						"\n\t"
						"OTHER optional options: \n\t"
						" -g <ground-truth-file>         (conducts performance evaluation and prints results) NOT YET IMPLEMENTED \n\t" //TODO implement
						" --NT                           (disables use of single camera transform) \n\t"
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

	// Input parameters
	PointCloudT::Ptr cloud = boost::make_shared<PointCloudT>();

	std::string rgb_path;
	std::string depth_path;
	std::string ply_path;
	std::string pcd_path;

	bool rgb_file_specified = console::find_switch(argc, argv, "-r");
	bool depth_file_specified = console::find_switch(argc, argv, "-d");
	bool ply_file_specified = console::find_switch(argc, argv, "-y");
	bool pcd_file_specified = console::find_switch(argc, argv, "-p");

	if (pcd_file_specified)
		console::parse(argc, argv, "-p", pcd_path);
	else if (ply_file_specified)
		console::parse(argc, argv, "-y", ply_path);
	else if (rgb_file_specified && depth_file_specified) {
		console::parse(argc, argv, "-r", rgb_path);
		console::parse(argc, argv, "-d", depth_path);
	} else {
		console::print_error("No input file specified");
		return (1);
	}

	bool thresh_specified = console::find_switch(argc, argv, "-t");
	if (!thresh_specified) {
		console::print_error("No threshold specified\n");
		return (1);
	}
	float thresh;
	console::parse_argument(argc, argv, "-t", thresh);
	console::print_debug("Using threshold: %f\n", thresh);

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
	bool manual_lambda_specified = console::find_switch(argc, argv, "--ML");
	bool adapt_lambda_specified = console::find_switch(argc, argv, "--AL");
	bool equalization_specified = console::find_switch(argc, argv, "--EQ");
	if (!(manual_lambda_specified || adapt_lambda_specified
			|| equalization_specified)) {
		adapt_lambda_specified = true;
		console::print_debug(
				"No merging criterion specified, Adaptive Lambda is going to be used\n");
	} else if (!(manual_lambda_specified ^ adapt_lambda_specified
			^ equalization_specified)) {
		console::print_error(
				"Only one parameter between --ML --AL and --EQ can be specified at a time\n");
		return (1);
	}

	float lambda = 0;
	if (manual_lambda_specified)
		console::parse_argument(argc, argv, "--ML", lambda);

	int bin_num = 0;
	if (equalization_specified)
		console::parse_argument(argc, argv, "--EQ", bin_num);

	// Output parameters
	std::string out_path;
	bool output_file_specified = console::find_switch(argc, argv, "-o");
	if (output_file_specified)
		console::parse(argc, argv, "-o", out_path);
	else
		out_path = "test_output.png";

	std::string out_label_path;
	bool output_label_file_specified = console::find_switch(argc, argv, "-l");
	if (output_label_file_specified)
		console::parse(argc, argv, "-l", out_label_path);
	else
		out_label_path = "test_output_labels.png";

	std::string refined_out_path;
	bool refined_output_file_specified = console::find_switch(argc, argv, "-O");
	if (refined_output_file_specified)
		console::parse(argc, argv, "-O", refined_out_path);
	else
		refined_out_path = "refined_test_output.png";

	std::string refined_out_label_path;
	bool refined_output_label_file_specified = console::find_switch(argc, argv,
			"-L");
	if (refined_output_label_file_specified)
		console::parse(argc, argv, "-L", refined_out_label_path);
	else
		refined_out_label_path = "refined_test_output_labels.png";

	// Pointcloud loading
	if (pcd_file_specified) {
		console::print_info("Loading pointcloud from PCD file...\n");
		io::loadPCDFile(pcd_path, *cloud);
		for (PointCloudT::iterator cloud_itr = cloud->begin();
				cloud_itr != cloud->end(); ++cloud_itr)
			if (cloud_itr->z < 0)
				cloud_itr->z = std::abs(cloud_itr->z);
	} else if (ply_file_specified) {
		console::print_info("Loading pointcloud from PLY file...\n");
		PLYReader ply_reader;
		ply_reader.read(ply_path, *cloud);
		for (PointCloudT::iterator cloud_itr = cloud->begin();
				cloud_itr != cloud->end(); ++cloud_itr)
			if (cloud_itr->z < 0)
				cloud_itr->z = std::abs(cloud_itr->z);
	} else {
		console::print_info("Generating pointcloud from RGB-D image...\n");
		vtkSmartPointer<vtkImageReader2Factory> reader_factory =
				vtkSmartPointer<vtkImageReader2Factory>::New();
		vtkImageReader2* rgb_reader = reader_factory->CreateImageReader2(
				rgb_path.c_str());
		//qDebug () << "RGB File="<< QString::fromStdString(rgb_path);
		if (!rgb_reader->CanReadFile(rgb_path.c_str())) {
			console::print_error("Cannot read rgb image file\n");
			return (1);
		}
		rgb_reader->SetFileName(rgb_path.c_str());
		rgb_reader->Update();
		//qDebug () << "Depth File="<<QString::fromStdString(depth_path);
		vtkImageReader2* depth_reader = reader_factory->CreateImageReader2(
				depth_path.c_str());
		if (!depth_reader->CanReadFile(depth_path.c_str())) {
			console::print_error("Cannot read depth image file\n");
			return (1);
		}
		depth_reader->SetFileName(depth_path.c_str());
		depth_reader->Update();

		vtkSmartPointer<vtkImageFlip> flipXFilter =
				vtkSmartPointer<vtkImageFlip>::New();
		flipXFilter->SetFilteredAxis(0); // flip x axis
		flipXFilter->SetInputConnection(rgb_reader->GetOutputPort());
		flipXFilter->Update();

		vtkSmartPointer<vtkImageFlip> flipXFilter2 = vtkSmartPointer<
				vtkImageFlip>::New();
		flipXFilter2->SetFilteredAxis(0); // flip x axis
		flipXFilter2->SetInputConnection(depth_reader->GetOutputPort());
		flipXFilter2->Update();

		vtkSmartPointer<vtkImageData> rgb_image = flipXFilter->GetOutput();
		int *rgb_dims = rgb_image->GetDimensions();
		vtkSmartPointer<vtkImageData> depth_image = flipXFilter2->GetOutput();
		int *depth_dims = depth_image->GetDimensions();

		if (rgb_dims[0] != depth_dims[0] || rgb_dims[1] != depth_dims[1]) {
			console::print_error("Depth and RGB dimensions to not match:\n"
					"\tRGB Image is of size %d by %d \n"
					"\tDepth Image is of size %d by %d\n", rgb_dims[0],
					rgb_dims[1], depth_dims[0], depth_dims[1]);
			return (1);
		}

		cloud->points.reserve(depth_dims[0] * depth_dims[1]);
		cloud->width = depth_dims[0];
		cloud->height = depth_dims[1];
		cloud->is_dense = false;

		// Fill in image data
		int centerX = static_cast<int>(cloud->width / 2.0);
		int centerY = static_cast<int>(cloud->height / 2.0);
		unsigned short* depth_pixel;
		unsigned char* color_pixel;
		float scale = 1.0f / 1000.0f;
		float focal_length = 525.0f;
		float fl_const = 1.0f / focal_length;
		depth_pixel =
				static_cast<unsigned short*>(depth_image->GetScalarPointer(
						depth_dims[0] - 1, depth_dims[1] - 1, 0));
		color_pixel = static_cast<unsigned char*>(rgb_image->GetScalarPointer(
				depth_dims[0] - 1, depth_dims[1] - 1, 0));

		for (size_t y = 0; y < cloud->height; ++y) {
			for (size_t x = 0; x < cloud->width;
					++x, --depth_pixel, color_pixel -= 3) {
				PointT new_point;
				//  uint8_t* p_i = &(cloud_blob->data[y * cloud_blob->row_step + x * cloud_blob->point_step]);
				float depth = static_cast<float>(*depth_pixel) * scale;
				if (depth == 0.0f) {
					new_point.x = new_point.y = new_point.z =
							std::numeric_limits<float>::quiet_NaN();
				} else {
					new_point.x = (static_cast<float>(x) - centerX) * depth
							* fl_const;
					new_point.y = (static_cast<float>(centerY) - y) * depth
							* fl_const; // vtk seems to start at the bottom left image corner
					new_point.z = depth;
				}

				uint32_t rgb = static_cast<uint32_t>(color_pixel[0]) << 16
						| static_cast<uint32_t>(color_pixel[1]) << 8
						| static_cast<uint32_t>(color_pixel[2]);
				new_point.rgb = *reinterpret_cast<float*>(&rgb);
				cloud->points.push_back(new_point);

			}
		}
	}

	console::print_info("Pointcloud loaded\n");

	////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////
	////// Supervoxel generation
	////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////

	SupervoxelClustering<PointT> super(voxel_resolution, seed_resolution,
			!disable_transform);
	super.setInputCloud(cloud);
	super.setColorImportance(color_importance);
	super.setSpatialImportance(spatial_importance);
	super.setNormalImportance(normal_importance);
	std::map<uint32_t, Supervoxel<PointT>::Ptr> supervoxel_clusters;

	console::print_info("Extracting supervoxels...\n");
	super.extract(supervoxel_clusters);
	console::print_info("Found %d supervoxels\n", supervoxel_clusters.size());
	PointCloudT::Ptr colored_voxel_cloud = super.getColoredVoxelCloud();
	PointCloudT::Ptr voxel_centroid_cloud = super.getVoxelCentroidCloud();
	PointCloudT::Ptr full_colored_cloud = super.getColoredCloud();
	PointNCloudT::Ptr sv_normal_cloud = super.makeSupervoxelNormalCloud(
			supervoxel_clusters);
	PointLCloudT::Ptr full_labeled_cloud = super.getLabeledCloud();

	console::print_info("Getting supervoxel adjacency...\n");
	std::multimap<uint32_t, uint32_t> label_adjacency;
	super.getSupervoxelAdjacency(label_adjacency);

	std::map<uint32_t, Supervoxel<PointT>::Ptr> refined_supervoxel_clusters;
	console::print_info("Refining supervoxels...\n");
	super.refineSupervoxels(3, refined_supervoxel_clusters);

	PointCloudT::Ptr refined_colored_voxel_cloud = super.getColoredVoxelCloud();
	PointNCloudT::Ptr refined_sv_normal_cloud = super.makeSupervoxelNormalCloud(
			refined_supervoxel_clusters);
	PointLCloudT::Ptr refined_full_labeled_cloud = super.getLabeledCloud();
	PointCloudT::Ptr refined_full_colored_cloud = super.getColoredCloud();

	// THESE ONLY MAKE SENSE FOR ORGANIZED CLOUDS
	io::savePNGFile(out_path, *full_colored_cloud, "rgb");
	io::savePNGFile(refined_out_path, *refined_full_colored_cloud, "rgb");
	io::savePNGFile(out_label_path, *full_labeled_cloud, "label");
	io::savePNGFile(refined_out_label_path, *refined_full_labeled_cloud,
			"label");

	console::print_info("Constructing Boost Graph Library Adjacency List...\n");
	typedef boost::adjacency_list<boost::setS, boost::setS, boost::undirectedS,
			uint32_t, float> VoxelAdjacencyList;
	typedef VoxelAdjacencyList::vertex_descriptor VoxelID;
	typedef VoxelAdjacencyList::edge_descriptor EdgeID;
	VoxelAdjacencyList supervoxel_adjacency_list;
	super.getSupervoxelAdjacencyList(supervoxel_adjacency_list);

	////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////
	////// Segmentation
	////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////

	Clustering segmentation;
	if (rgb_color_space_specified)
		segmentation.set_delta_c(RGB_EUCL);

	if (manual_lambda_specified) {
		segmentation.set_merging(MANUAL_LAMBDA);
		if (lambda != 0)
			segmentation.set_lambda(lambda);
	} else if (equalization_specified) {
		segmentation.set_merging(EQUALIZATION);
		if (bin_num != 0)
			segmentation.set_bins_num(bin_num);
	}

	//segmentation.test_all();

	console::print_info("Segmentation initialization...\n");
	segmentation.set_initialstate(supervoxel_clusters, label_adjacency);
	if (manual_lambda_specified || adapt_lambda_specified)
		console::print_debug("Lambda: %f\n", segmentation.get_lambda());
	console::print_info("Initialization complete\nStarting clustering...\n");
	segmentation.cluster(thresh);
	console::print_info("Clustering complete\n");
	std::pair<ClusteringT, AdjacencyMapT> s = segmentation.get_currentstate();

	colored_voxel_cloud = segmentation.get_colored_cloud();
	label_adjacency = s.second;

	////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////
	////// Visualization
	////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////

	console::print_info("Loading visualization...\n");
	boost::shared_ptr<visualization::PCLVisualizer> viewer(
			new visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->registerKeyboardCallback(keyboard_callback, 0);

	bool graph_added = false;
	std::vector<std::string> poly_names;
	console::print_info("Loading viewer...");
	while (!viewer->wasStopped()) {
		if (show_voxel_centroids) {
			if (!viewer->updatePointCloud(voxel_centroid_cloud,
					"voxel centroids"))
				viewer->addPointCloud(voxel_centroid_cloud, "voxel centroids");
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
					(show_supervoxels) ?
							refined_colored_voxel_cloud : colored_voxel_cloud,
					"colored voxels"))
				viewer->addPointCloud(
						(show_supervoxels) ?
								refined_colored_voxel_cloud :
								colored_voxel_cloud, "colored voxels");
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
			viewer->addPointCloudNormals<PointNormal>(refined_sv_normal_cloud,
					1, 0.05f, "supervoxel_normals");
		} else if (!show_supervoxel_normals) {
			viewer->removePointCloud("supervoxel_normals");
		}

		if (show_graph && !graph_added) {
			poly_names.clear();
			std::multimap<uint32_t, uint32_t>::iterator label_itr =
					label_adjacency.begin();
			for (; label_itr != label_adjacency.end();) {
				//First get the label
				uint32_t supervoxel_label = label_itr->first;
				//Now get the supervoxel corresponding to the label
				Supervoxel<PointT>::Ptr supervoxel = supervoxel_clusters.at(
						supervoxel_label);
				//Now we need to iterate through the adjacent supervoxels and make a point cloud of them
				PointCloudT adjacent_supervoxel_centers;
				std::multimap<uint32_t, uint32_t>::iterator adjacent_itr =
						label_adjacency.equal_range(supervoxel_label).first;
				for (;
						adjacent_itr
								!= label_adjacency.equal_range(supervoxel_label).second;
						++adjacent_itr) {
					Supervoxel<PointT>::Ptr neighbor_supervoxel =
							supervoxel_clusters.at(adjacent_itr->second);
					adjacent_supervoxel_centers.push_back(
							neighbor_supervoxel->centroid_);
				}
				//Now we make a name for this polygon
				std::stringstream ss;
				ss << "supervoxel_" << supervoxel_label;
				poly_names.push_back(ss.str());
				addSupervoxelConnectionsToViewer(supervoxel->centroid_,
						adjacent_supervoxel_centers, ss.str(), viewer);
				//Move iterator forward to next label
				label_itr = label_adjacency.upper_bound(supervoxel_label);
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
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));

	}
	return (0);
}

void addSupervoxelConnectionsToViewer(PointT &supervoxel_center,
		PointCloudT &adjacent_supervoxel_centers, std::string supervoxel_name,
		boost::shared_ptr<visualization::PCLVisualizer> & viewer) {
	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New();
	vtkSmartPointer<vtkPolyLine> polyLine = vtkSmartPointer<vtkPolyLine>::New();

	//Iterate through all adjacent points, and add a center point to adjacent point pair
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

void printText(boost::shared_ptr<visualization::PCLVisualizer> viewer) {
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

void removeText(boost::shared_ptr<visualization::PCLVisualizer> viewer) {
	viewer->removeShape("hud_text");
	viewer->removeShape("voxel_text");
	viewer->removeShape("supervoxel_text");
	viewer->removeShape("graph_text");
	viewer->removeShape("voxel_normals_text");
	viewer->removeShape("supervoxel_normals_text");
	viewer->removeShape("refined_text");
}
