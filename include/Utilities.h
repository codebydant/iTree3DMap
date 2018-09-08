//***********************************************
//HEADERS
//***********************************************
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>
#include <string>
#include <chrono>

#include <boost/filesystem.hpp>
#include <boost/algorithm/algorithm.hpp>
#include <boost/thread/thread.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/common/geometry.h>
#include <pcl/common/common.h>

#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/gasd.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/simplification_remove_unused_vertices.h>

#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <vtkActor.h>
#include <vtkCallbackCommand.h>
#include <vtkCommand.h>
#include <vtkPolyData.h>
#include <vtkLineSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkSmartPointer.h>
#include <vtkSimplePointsReader.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkAxesActor.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkCaptionActor2D.h>
#include <vtkNamedColors.h>
#include <vtkTransform.h>

#include <ros/ros.h>
#include <tinyxml2.h>
#include <X11/Xlib.h>

class Utilities{


private:


public:

   Utilities(){}
  ~Utilities(){}   

   struct Point3DInMap{
     cv::Point3d pt;
     std::map<const int,std::map<const int,cv::Point2d>> feat_ref;// [id image,[id pt2d, pt2d]]
   };

   static bool run_openMVG();
   static bool createPMVS_Files();
   static bool densifyWithPMVS(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output_cloud);
   static bool uniformScaling(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
                                  pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_scaled,const double scale=2);
   static void help();
   static bool getScaleFactor(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& Map3D, double& scale_factor,std::string& output_path);

   static bool loadSFM_XML_Data(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pts3d,
                                cv::Mat_<double>& intrinsic,
                                std::vector<cv::Matx34d>& cameras_poses);
   static bool alignCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
                              pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_align);
                              
   static void create_mesh(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,pcl::PolygonMesh &mesh);
   static void vizualizeMesh(pcl::PolygonMesh &mesh);
   static void vtkVisualizer(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,pcl::PointXYZ& pt1,pcl::PointXYZ& pt2);

};


