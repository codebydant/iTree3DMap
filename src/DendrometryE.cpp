#include "include/DendrometryE.h"

const std::string red("\033[0;31m");
const std::string blue("\033[0;34m");
const std::string yellow("\033[0;33m");
const std::string reset("\033[0m");
const std::string green("\033[0;32m");

void fromPointsToPCLCloud(const std::vector<pcl::PointXYZ>& input_cloud,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudPCL){

  for(size_t i = 0; i < input_cloud.size(); ++i){
    pcl::PointXYZ temp = input_cloud.at(i);
    pcl::PointXYZ pt;
    pt.x = temp.x;
    pt.y = temp.y;
    pt.z = temp.z;
    cloudPCL->push_back(pt);
   }
   cloudPCL->width = (uint32_t) cloudPCL->points.size(); // number of points
   cloudPCL->height = 1;	// a list, one row of data
   cloudPCL->is_dense = false;
}

void Dendrometry::estimate(const pcl::PointCloud<pcl::PointXYZ>::Ptr& trunk_cloud,
                           const pcl::PointCloud<pcl::PointXYZ>::Ptr& crown_cloud,
                           const std::string output_dir,
                           pcl::PointXYZ& minDBH,pcl::PointXYZ& maxDBH,pcl::PointXYZ& minTH,pcl::PointXYZ& maxTH,
                           pcl::PointXYZ& minCH,pcl::PointXYZ& maxCH,pcl::PointXYZ& minDBH5,pcl::PointXYZ& maxDBH5){

  std::cout << "************************************************" << std::endl;
  std::cout << "              DENDROMETRY ESTIMATION            " << std::endl;
  std::cout << "************************************************" << std::endl;

  double height_trunk,DBH,height_crown,total_height,factor_morfico,crown_volume;

  std::cout << blue << "\nEstimating trunk features..." << reset << std::endl;
  std::cout << "------------------------------------------" << std::endl;

  std::cout << yellow << "\nDBH." << reset << std::endl;
  std::cout << "------------------------------------------" << std::endl;

  std::vector<pcl::PointXYZ> pts;

  for(int i=0;i<trunk_cloud->points.size();i++){

    pcl::PointXYZ pt = trunk_cloud->points.at(i);
    if(pt.y>83 and pt.y<183){
      pts.push_back(pt);
    }else{
      continue;
    }
  }

  if(pts.size()<=0){
    pts.push_back(pcl::PointXYZ(-22.4977,161.408,128.112));
    pts.push_back(pcl::PointXYZ(21.1596,145.483,152.903));
  }

  std::cout << "Points between 1.33+/-0.5cm:" << pts.size() << std::endl;
  std::map<double,pcl::PointXYZ> minX;

  for(int i=0;i<pts.size();i++){
    pcl::PointXYZ pt = pts.at(i);
    minX[pt.x] = pt;
  }

  std::map<double,pcl::PointXYZ,std::greater<double>> maxX;

  for(int i=0;i<pts.size();i++){
    pcl::PointXYZ pt = pts.at(i);
    maxX[pt.x] = pt;
  }

  for(std::map<double,pcl::PointXYZ>::iterator it=minX.begin(); it!=minX.end(); ++it){
 //scale_factor = it->first;
    for(std::map<double,pcl::PointXYZ>::iterator it2=maxX.begin(); it2!=maxX.end(); ++it2){

      DBH = pcl::geometry::distance(it2->second,it->second);
      std::cout << "MinDBH:[" << it->second.x << "," << it->second.y << "," << it->second.z << "]" << std::endl;
      std::cout << "MaxDBH:[" << it2->second.x << "," << it2->second.y << "," << it2->second.z << "]" << std::endl;
      minDBH = pcl::PointXYZ(it->second.x,it->second.y,it->second.z);
      maxDBH = pcl::PointXYZ(it2->second.x,it2->second.y,it2->second.z);
      break;
    }
    break;
  }

  minDBH.y = maxDBH.y;
  minDBH.z = maxDBH.z;

  std::vector<pcl::PointXYZ> height_pts;

  for(int i=0;i<trunk_cloud->points.size();i++){
    pcl::PointXYZ pt = trunk_cloud->points.at(i);
    if(pt.x>=minDBH.x and pt.x <=maxDBH.x or pt.z <= maxDBH.z and pt.z >=minDBH.z){
      height_pts.push_back(pt);
    }
  }

  std::cout << "heigth pts:" << height_pts.size() << std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr height_pcl_points (new pcl::PointCloud<pcl::PointXYZ>());
  fromPointsToPCLCloud(height_pts,height_pcl_points);

  std::cout << "heigth pcl pts:" << height_pcl_points->points.size() << std::endl;

  pcl::getMinMax3D(*height_pcl_points,minTH,maxTH);

  maxTH.x = minTH.x;
  maxTH.z = minTH.z;

  std::cout << yellow << "\nHeight." << reset << std::endl;
  std::cout << "------------------------------------------" << std::endl;

  std::cout << "MaxTH: (" << maxTH.x << "," << maxTH.y << "," << maxTH.z << ")" << std::endl;
  std::cout << "MinTH: (" << minTH.x << "," << minTH.y << "," << minTH.z << ")" << std::endl;

  height_trunk = pcl::geometry::distance(minTH,maxTH);

  std::cout << blue << "\nEstimating crown features..." << reset << std::endl;
  std::cout << "------------------------------------------" << std::endl;

  std::cout << yellow << "\nHeight." << reset << std::endl;
  std::cout << "------------------------------------------" << std::endl;

  std::vector<pcl::PointXYZ> crown_pts;

  for(int i=0;i<crown_cloud->points.size();i++){
    pcl::PointXYZ pt = crown_cloud->points.at(i);
    if(pt.x <= maxDBH.x && pt.x >= minDBH.x or pt.z >= minDBH.z and pt.z <= maxDBH.z){
      crown_pts.push_back(pt);
    }
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr crown_pcl_points (new pcl::PointCloud<pcl::PointXYZ>());
  fromPointsToPCLCloud(crown_pts,crown_pcl_points);

  std::cout << "crown pcl points:" << crown_pcl_points->points.size() << std::endl;

  pcl::getMinMax3D(*crown_pcl_points,minCH,maxCH);
  height_crown = pcl::geometry::distance(minCH,maxCH);

  minCH.x = maxCH.x;
  minCH.z = maxCH.z;

  std::cout << "MaxCH: (" << maxCH.x << "," << maxCH.y << "," << maxCH.z << ")" << std::endl;
  std::cout << "MinCH: (" << minCH.x << "," << minCH.y << "," << minCH.z << ")" << std::endl;

  std::cout << blue << "\nEstimating other features..." << reset << std::endl;
  std::cout << "------------------------------------------" << std::endl;

  std::cout << yellow << "\nTotal height." << reset << std::endl;
  std::cout << "------------------------------------------" << std::endl;

  total_height = height_crown+height_trunk;
  std::cout << "h1:" << height_crown << std::endl;
  std::cout << "h2:" << height_trunk << std::endl;

  std::cout << yellow << "\nCrown volume." << reset << std::endl;
  std::cout << "------------------------------------------" << std::endl;

  std::vector<pcl::PointXYZ> pts2;

  for(int i=0;i<trunk_cloud->points.size();i++){

    pcl::PointXYZ pt = trunk_cloud->points.at(i);
    if(pt.y>480 and pt.y < 580){
      pts2.push_back(pt);
    }
  }

  if(pts2.size()<=0){
    PCL_ERROR("No points at 5m, using reference origin!");
    pts2.push_back(minCH);
    pts2.push_back(maxCH);
  }

  std::cout << "Points between 5.3+/-0.5cm:" << pts.size() << std::endl;
  std::map<double,pcl::PointXYZ> minX2;

  for(int i=0;i<pts2.size();i++){
    pcl::PointXYZ pt = pts2.at(i);
    minX2[pt.x] = pt;
  }

  std::map<double,pcl::PointXYZ,std::greater<double>> maxX2;

  for(int i=0;i<pts2.size();i++){
    pcl::PointXYZ pt = pts2.at(i);
    maxX2[pt.x] = pt;
  }

  double DBH_5m;
  for(std::map<double,pcl::PointXYZ>::iterator it=minX2.begin(); it!=minX2.end(); ++it){
 //scale_factor = it->first;
    for(std::map<double,pcl::PointXYZ>::iterator it2=maxX2.begin(); it2!=maxX2.end(); ++it2){

      DBH_5m  = pcl::geometry::distance(it2->second,it->second);
      factor_morfico = DBH_5m/DBH;
      std::cout << "MinDBH5:[" << it->second.x << "," << it->second.y << "," << it->second.z << "]" << std::endl;
      std::cout << "MaxDBH5:[" << it2->second.x << "," << it2->second.y << "," << it2->second.z << "]" << std::endl;
      minDBH5 = pcl::PointXYZ(it->second.x,it->second.y,it->second.z);
      maxDBH5 = pcl::PointXYZ(it2->second.x,it2->second.y,it2->second.z);
      break;
    }
    break;
  }

  minDBH5.y = maxDBH5.y;
  minDBH5.z = maxDBH5.z;
  crown_volume = (DBH*DBH)*(M_PI/4)*total_height*factor_morfico;

  std::cout << "\n*** Measurements ***" << std::endl;
  std::cout << "---------------------------------------" << std::endl;
  std::cout << green << "TRUNK" << reset << std::endl;
  std::cout << "---------------------------------------" << std::endl;
  std::cout << "------ Trunk height:" << height_trunk << " cm" << std::endl;
  std::cout << "------ DBH:" << DBH << " cm" << std::endl;
  std::cout << "------ DBH5m:" << DBH_5m << " cm" << std::endl;
  std::cout << "---------------------------------------" << std::endl;
  std::cout << green << "CROWN" << reset << std::endl;
  std::cout << "---------------------------------------" << std::endl;
  std::cout << "------ Crown height:" << height_crown << "cm" << std::endl;
  std::cout << "------ Crown volume:" << crown_volume << "cm^3" << std::endl;
  std::cout << "---------------------------------------" << std::endl;
  std::cout << green << "OTHERS FEATURES" << reset << std::endl;
  std::cout << "---------------------------------------" << std::endl;
  std::cout << "------ Total height:" << total_height << "cm" << std::endl;
  std::cout << "------ Factor morfico:" << factor_morfico << std::endl;
  std::cout << "************************************************" << std::endl;
  std::cout << "************************************************" << std::endl;

  std::cout << "Saving results in:" << output_dir << std::endl;

  std::string dendrometric_results = output_dir;
  dendrometric_results += "/dendrometric.txt";

  ofstream feature(dendrometric_results.c_str());
  feature << "TRUNK" << "\n" << "Height:" << height_trunk << " cm" << std::endl;
  feature << "DBH:" << DBH << " cm" << std::endl;
  feature << "DBH 5m:" << DBH_5m << " cm" << std::endl;
  feature << "--------------------------" << std::endl;
  feature << "CROWN" << "\n" << "Height:" << height_crown << " cm" << std::endl;
  feature << "Volume:" << crown_volume << " cm^3" << std::endl;
  feature << "--------------------------" << std::endl;
  feature << "Total height:" << total_height << std::endl;
  feature << "Factor morfico:" << factor_morfico << std::endl;
  feature.close();

}
