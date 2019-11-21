# Fast 3D Pointcloud Segmentation

This is the code used in the article:

F. Verdoja, D. Thomas, and A. Sugimoto, “Fast 3D point cloud segmentation using supervoxels with geometry and color for 3D scene understanding,” in _IEEE International Conference on Multimedia and Expo (ICME 2017)_, Hong Kong, 2017, pp. 1285–1290.

Article versions: [[preprint](https://iris.unito.it/bitstream/2318/1647554/1/article_icme2017.pdf)] [[published](https://ieeexplore.ieee.org/abstract/document/8019382)]

## Paper abstract

Segmentation of 3D colored point clouds is a research field with renewed interest thanks to recent availability of inexpensive consumer RGB-D cameras and its importance as an unavoidable low-level step in many robotic applications. However, 3D data’s nature makes the task challenging and, thus, many different techniques are being proposed, all of which require expensive computational costs. This paper presents a novel fast method for 3D colored point cloud segmentation. It starts with supervoxel partitioning of the cloud, i.e., an oversegmentation of the points in the cloud. Then it leverages on a novel metric exploiting both geometry and color to iteratively merge the supervoxels to obtain a 3D segmentation where the hierarchical structure of partitions is maintained. The algorithm also presents computational complexity linear to the size of the input. Experimental results over two publicly available datasets demonstrate that our proposed method outperforms state-of-the-art techniques.

## Dependencies

The code is based on C++ and requires PCL 1.8+ and OpenCV4 to work.

Optionally, the code supports ROS integration. If ROS support is required, Catkin is required.

## Compilation

### With ROS support

The default option for this code is to compile with ROS support enabled. In this case, placing this package inside a Catkin workspace should work.

Be mindful that `git clone` by default will put this package in a folder called _Fast-3D-Pointcloud-Segmentation_, might be necessary to rename that
folder to _supervoxel\_clustering_.

Then, the workspace can be compiled like usual with Catkin.

### Without ROS support

To use this package without ROS the option `-DUSE_CATKIN=OFF` must be used while calling `cmake`, like this:
```
mkdir build
cd build
cmake -DUSE_CATKIN=OFF ..
make
```

## Use

```
Syntax is: ./supervoxel_clustering {-d <direcory-of-pcd-files> OR -p <pcd-file>} [arguments] 

        SUPERVOXEL optional arguments: 
         -v <voxel-resolution>          (default: 0.008) 
         -s <seed-resolution>           (default: 0.08) 
         -c <color-weight>              (default: 0.2) 
         -z <spatial-weight>            (default: 0.4) 
         -n <normal-weight>             (default: 1.0) 

        SEGMENTATION optional arguments: 
         -t <threshold>                 (default: auto)
         --RGB                          (uses the RGB color space for measuring the color distance; if not given, L*A*B* color space is used) 
         --CVX                          (uses the convexity criterion to weigh the geometric distance; if not given, convexity is not considered) 
         --ML [manual-lambda] *         (uses Manual Lambda as merging criterion; if no parameter is given, lambda=0.5 is used) 
         --AL                 *         (uses Adaptive lambda as merging criterion) 
         --EQ [bins-number]   *         (uses Equalization as merging criterion; if no parameter is given, 200 bins are used) 
          * please note that only one of these arguments can be passed at the same time 

        OTHER optional arguments: 
         -r <label-to-be-removed>       (if ground-truth is provided, removes all points with the given label from the ground-truth)
         -f <test-results-filename>     (uses the given name as filename for all test results files; if not given, 'test' is going to be used)
         --NT                           (disables use of single camera transform) 
         --V                            (verbose)
```

### From ROS

If used with ROS support enabled, the executable can be called from launch files. One example launch file is provided in the _launch_ folder.

For a more complete ROS node implementation, please have a look at [this repository](https://github.com/aalto-intelligent-robotics/point_cloud_segmentation).
