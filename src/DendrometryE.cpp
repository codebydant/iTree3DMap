#include "include/DendrometryE.h"

void Dendrometry::estimate(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloudPCL){

  std::cout << "************************************************" << std::endl;
  std::cout << "              DENDROMETRY ESTIMATION            " << std::endl;
  std::cout << "************************************************" << std::endl;

  pcl::PointXYZRGB minPt,maxPt;
  pcl::getMinMax3D(*cloudPCL,minPt,maxPt);

  cv::Point3f min=cv::Point3f(minPt.x,minPt.y,minPt.z);
  cv::Point3f max=cv::Point3f(maxPt.x,maxPt.y,maxPt.z);
  //std::cout << "Max: " << max << std::endl;
  //std::cout << "Min: " << min << std::endl;

  std::cout << "*** Measurements ***" << std::endl;
  std::cout << "---------------------------------------" << std::endl;
  std::cout << "---------------------------------------" << std::endl;
  std::cout << "------ Total height => " << cv::norm(max-min) << "m" << std::endl;
  std::cout << "------ Crop base height =>  " << 5.0 << "m" << std::endl;
  std::cout << "------ Height DBH => " << 1.3 << "cm" << std::endl;
  std::cout << "------ DBH => " << 30.0 << "cm" << std::endl;
  std::cout << "---------------------------------------" << std::endl;
  std::cout << "---------------------------------------" << std::endl;
  std::cout << "************************************************" << std::endl;
  std::cout << "************************************************" << std::endl;

}
