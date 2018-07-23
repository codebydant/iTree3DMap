#include "include/DendrometryE.h"

void Dendrometry::estimate(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloudPCL){

  std::cout << "************************************************" << std::endl;
  std::cout << "              DENDROMETRY ESTIMATION            " << std::endl;
  std::cout << "************************************************" << std::endl;

  float distance_error;

  pcl::PointXYZRGB minPt,maxPt;
  pcl::getMinMax3D(*cloudPCL,minPt,maxPt);
  distance_error = pcl::geometry::distance(minPt,maxPt);

  std::cout << "Max: (" << maxPt.x << "," << maxPt.y << "," << maxPt.z << ")" << std::endl;
  std::cout << "Min: (" << minPt.x << "," << minPt.y << "," << minPt.z << ")" << std::endl;

  std::cout << "*** Measurements ***" << std::endl;
  std::cout << "---------------------------------------" << std::endl;
  std::cout << "---------------------------------------" << std::endl;
  std::cout << "------ Total height => " << distance_error << "m" << std::endl;
  std::cout << "------ Crop base height =>  " << 5.0 << "m" << std::endl;
  std::cout << "------ Height DBH => " << 1.3 << "cm" << std::endl;
  std::cout << "------ DBH => " << 30.0 << "cm" << std::endl;
  std::cout << "---------------------------------------" << std::endl;
  std::cout << "---------------------------------------" << std::endl;
  std::cout << "************************************************" << std::endl;
  std::cout << "************************************************" << std::endl;

}
