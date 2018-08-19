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
   static void createPMVS_Files();
   static void densifyWithPMVS(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output_cloud);
   static void uniformScaling(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
                                  pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_scaled,const double scale=2);
   static void help();
   static bool getScaleFactor(pcl::PointCloud<pcl::PointXYZ>::Ptr& Map3D, double& scale_factor,std::string& output_path);
   static void fromPoint3DToPCLCloud(const std::vector<Point3DInMap> &input_cloud,
                              pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudPCL);
   static bool loadSFM_XML_Data(std::vector<Point3DInMap>& cloud,
                                cv::Mat_<double>& intrinsic,
                                std::vector<cv::Matx34d>& cameras_poses);
};


