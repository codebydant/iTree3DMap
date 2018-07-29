/*********************************
           HEADERS
**********************************/
#include "include/DendrometryE.h"
#include <sstream>

/*
void subscriberCallback(const cv::Mat& img){
   // ROS_INFO("I heard: [%s]", img.size);
}
*/
/*********************************
      MAIN FUNCTION
**********************************/
int main(int argc, char **argv){

  Utilities::help();
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPCL (new pcl::PointCloud<pcl::PointXYZ>());
  bool success = Utilities::loadSFM_XML_Data(cloudPCL,false);

  std::ifstream file("IMG_20180629_161757_DRO.feat");
  if(!file.is_open()){
    std::cout << "Error: Could not find feature file." << std::endl;
  }

  float x, y,s,orientation;
  std::vector<cv::Point2f> image_points;

  while(file >> x >> y >> s >> orientation){

      //std::cout << "x:" << x << " y:" << y << std::endl;
      image_points.push_back(cv::Point2f(x,y));
  }

  std::cout << "image points:" << image_points.size()<< std::endl;




  float scale;
  Utilities::getScaleFactor(scale);

  /*************************
  STEP 1: ROS INTERFACE
  **************************/
  /*
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("input", 1000, subscriberCallback);
  ros::spinOnce();
  */

  /*************************
  STEP 2: 3D MAPPING
  **************************/
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_3dMap (new pcl::PointCloud<pcl::PointXYZRGB>());


  pcl::io::loadPCDFile("binary.pcd",*cloud_3dMap);
 // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_3dMap_xyz (new pcl::PointCloud<pcl::PointXYZ>());
 // pcl::copyPointCloud(*cloud_3dMap,*cloud_3dMap_xyz);

  //Utilities::run_openMVG();
 // Utilities::createPMVS_Files();
 // Utilities::densifyWithPMVS(cloud_3dMap);

  /*************************
  STEP 3: SEGMENTATION
  **************************/  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr tree_segmented (new pcl::PointCloud<pcl::PointXYZRGB>());
  auto start = std::chrono::high_resolution_clock::now();
  Segmentation::extractTree(cloud_3dMap,tree_segmented);
  auto end = std::chrono::high_resolution_clock::now();
  auto difference = std::chrono::duration_cast<std::chrono::seconds>(end - start).count();
  if(difference >=60){
    std::cout << "Segmentation Time: " << difference/60 << " minutes" << std::endl;
  }else{
    std::cout << "Segmentation Time: " << difference << " seconds" << std::endl;
  }

  /*************************
  STEP 4: UNIFORM SCALING
  **************************/  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_3dMap_scaled (new pcl::PointCloud<pcl::PointXYZRGB>());
  Utilities::uniformScaling(tree_segmented,cloud_3dMap_scaled,scale,true);

  /*************************
  STEP 5: DENDROMETRY MEASUREMENTS
  **************************/
  Dendrometry::estimate(cloud_3dMap_scaled);

  return 0;
}

