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

class Utilities{


private:


public:

   Utilities(){}
  ~Utilities(){}

   static void run_openMVG();
   static void createPMVS_Files();
   static void densifyWithPMVS(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output_cloud);
   static void uniformScaling(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
                                  pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_scaled,const float scale=2,
                                  bool show=false);
   static void help();
   static void getScaleFactor(float& scale_factor);

};


