#pragma once
#ifndef UTILITIES_H
#define UTILITIES_H
// #include <fstream>
// #include <boost/algorithm/string/case_conv.hpp>
// #include <boost/algorithm/string.hpp>
// #include <X11/X.h>

// see https://stackoverflow.com/questions/22400905/eigen-and-cimg-compatibility-issues
// #include <X11/Xlib.h>

// #include <eigen3/Eigen/Eigen>
// #ifdef Success
//   #undef Success
// #endif

// #include <X11/Xutil.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// #include <X11/Xlib.h>
// #include <pcl/common/common.h>
#include <pcl/common/geometry.h>
#include <pcl/common/transforms.h>
// #include <pcl/console/parse.h>
// #include <pcl/console/print.h>
// #include <pcl/console/time.h>
// #include <pcl/conversions.h>
// #include <pcl/features/gasd.h>
// #include <pcl/features/normal_3d.h>
// #include <pcl/features/normal_3d_omp.h>
// #include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
// #include <pcl/filters/radius_outlier_removal.h>
// #include <pcl/filters/statistical_outlier_removal.h>
// #include <pcl/filters/voxel_grid.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/io/ply_io.h>
// #include <pcl/io/vtk_lib_io.h>
// #include <pcl/point_types.h>
// #include <pcl/search/kdtree.h>
// #include <pcl/search/search.h>
// #include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
// #include <pcl/surface/bilateral_upsampling.h>
// #include <pcl/surface/gp3.h>
// #include <pcl/surface/mls.h>
// #include <pcl/surface/poisson.h>
// #include <pcl/surface/simplification_remove_unused_vertices.h>
// #include <pcl/surface/vtk_smoothing/vtk_utils.h>
#include <pcl/visualization/pcl_visualizer.h>

// #include <vtkActor.h>
// #include <vtkAxesActor.h>
// #include <vtkCallbackCommand.h>
// #include <vtkCamera.h>
// #include <vtkCaptionActor2D.h>
// #include <vtkCommand.h>
// #include <vtkGraphicsFactory.h>
// #include <vtkInteractorStyleSwitch.h>
// #include <vtkInteractorStyleTrackballActor.h>
// #include <vtkInteractorStyleTrackballCamera.h>
// #include <vtkInteractorStyleUser.h>
// #include <vtkLineSource.h>
// #include <vtkNamedColors.h>
// #include <vtkOrientationMarkerWidget.h>
// #include <vtkPNGWriter.h>
// #include <vtkPlaneSource.h>
// #include <vtkPolyData.h>
// #include <vtkPolyDataMapper.h>
// #include <vtkProperty.h>
// #include <vtkRegularPolygonSource.h>
// #include <vtkRenderWindow.h>
// #include <vtkRenderWindowInteractor.h>
// #include <vtkRenderer.h>
// #include <vtkSimplePointsReader.h>
// #include <vtkSmartPointer.h>
// #include <vtkSphereSource.h>
// #include <vtkTextActor.h>
// #include <vtkTextProperty.h>
// #include <vtkTransform.h>
// #include <vtkVertexGlyphFilter.h>
// #include <vtkWindowToImageFilter.h>

// #include <boost/algorithm/algorithm.hpp>
// #include <boost/algorithm/string.hpp>
// #include <boost/algorithm/string/case_conv.hpp>
// #include <boost/filesystem.hpp>
// #include <boost/thread/thread.hpp>
#include <chrono>
// #include <fstream>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
// #include <opencv2/highgui.hpp>
// #include <opencv2/imgcodecs.hpp>
// #include <opencv2/imgproc.hpp>

#include <string>
// #include <opencv2/core/types.hpp>
// #include <opencv2/core/mat.hpp>
// #include <core/include/opencv2/core/matx.hpp>

// #include "argparse/argparse.hpp"
// #include "progress_display.hpp"
// #include "argparse/argparse.hpp"
#include "3DMapper/Mapping.hpp"
#include "itreemap/scale_factor.hpp"

class Utilities {
 private:
 public:
  Utilities() {}
  ~Utilities() {}

  // struct Point3DInMap {
  //   cv::Point3d pt;
  //   std::map<const int, std::map<const int, cv::Point2d>> feat_ref;  // [id image,[id pt2d, pt2d]]
  // };

  static bool run_openMVG();
  static bool createPMVS_Files();
  static bool densifyWithPMVS(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output_cloud, bool& projectFound);
  static bool uniformScaling(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_scaled, bool& projectFound, double& scale);
  static void setup_argparse(int argc, char** argv);

  static bool getScaleFactor(double& scale_factor);

  static bool loadSFM_XML_Data(cv::Mat_<double>& intrinsic, std::vector<cv::Matx34d>& cameras_poses, std::string& output_dir);
  // static bool alignCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
  //                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_align);

  // static void create_mesh(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PolygonMesh& mesh, int nThreads = 8,
  //                         int setKsearch = 10, int depth = 10, float pointWeight = 4.0, float samplePNode = 1.5,
  //                         float scale = 1.1, int isoDivide = 8, bool confidence = true, bool outputPolygons = true,
  //                         bool manifold = true, int solverDivide = 8);
  // static void createMeshFromCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PolygonMesh& triangles);
  // static void vizualizeMesh(vtkSmartPointer<vtkPolyData>& vtkCloud1, vtkSmartPointer<vtkPolyData>& vtkCloud2);
  // static void vtkVisualizer(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointXYZ& pt1, pcl::PointXYZ& pt2);
  static bool is_number(const std::string& s);

  // static void alignCloudSFM(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
  //                           pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_transformed, Eigen::Affine3f& transform_1,
  //                           Eigen::Affine3f& transform_2, Eigen::Affine3f& transform_3);

  // static void getEllipses(std::vector<std::vector<cv::Point>>& contours, std::vector<cv::RotatedRect>& ellipses);
};
#endif