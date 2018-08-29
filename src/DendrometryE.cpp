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

  std::string dendrometric_results = output_dir;
  dendrometric_results += "/dendrometric.txt";

  ofstream feature(dendrometric_results.c_str());

  double height_trunk,DBH,height_crown,total_height,factor_morfico,crown_volume;

  std::cout << blue << "\nEstimating trunk features..." << reset << std::endl;
  std::cout << "------------------------------------------" << std::endl;

  std::cout << yellow << "\nDBH." << reset << std::endl;
  std::cout << "------------------------------------------" << std::endl;


/*

  std::map<double,pcl::PointXYZ> minMaxValues2;

  for(pcl::PointCloud<pcl::PointXYZ>::iterator it=cloud_trunk->begin(); it!=cloud_trunk->end(); ++it){
    pcl::PointXYZ p = pcl::PointXYZ(it->x,it->y,it->z);
    minMaxValues2[p.y] = p;
  }

  pcl::PointXYZ minH1,maxH2;

  std::map<double,pcl::PointXYZ>::iterator it11 = minMaxValues2.begin();
  minH1 = it11->second;

  std::map<double,pcl::PointXYZ>::iterator it22 = std::prev(minMaxValues2.end());
  maxH2 = it22->second;

*
*/





  std::vector<pcl::PointXYZ> pts_for_DBH;

  for(pcl::PointCloud<pcl::PointXYZ>::iterator it=trunk_cloud->begin();it!=trunk_cloud->end(); ++it){

    pcl::PointXYZ pt = pcl::PointXYZ(it->x,it->y,it->z);
    if(pt.y>120 and pt.y<140){
      pts_for_DBH.push_back(pt);
    }else{
      continue;
    }
  }

  if(pts_for_DBH.size()<=0){
    ROS_ERROR("No points at 1.33m. Using random points!");
    pts_for_DBH.push_back(pcl::PointXYZ(-22.4977,161.408,128.112));
    pts_for_DBH.push_back(pcl::PointXYZ(21.1596,145.483,152.903));
  }

  ROS_INFO("Points between 1.33+/-0.5cm: %lu", pts_for_DBH.size());

  feature << "ANALYSIS" << std::endl;
  feature << "Points between 1.33m +/- 0.5cm:" << pts_for_DBH.size() << std::endl;

  std::map<double,pcl::PointXYZ> min_max_DBH;

  for(std::vector<pcl::PointXYZ>::iterator it=pts_for_DBH.begin();it!=pts_for_DBH.end(); ++it){
    pcl::PointXYZ pt = pcl::PointXYZ(it->x,it->y,it->z);
    min_max_DBH[pt.x] = pt;
  }

  std::map<double,pcl::PointXYZ>::iterator it1 = min_max_DBH.begin();
  minDBH = it1->second;

  std::map<double,pcl::PointXYZ>::iterator it2 = std::prev(min_max_DBH.end());
  maxDBH = it2->second;

  std::cout << "MinDBH:" << minDBH << std::endl;
  std::cout << "MaxDBH:" << maxDBH << std::endl;

  feature << "MinDBH pt3d:" << minDBH << std::endl;
  feature << "MaxDBH pt3d:" << maxDBH << std::endl;

  minDBH.y = maxDBH.y;
  minDBH.z = maxDBH.z;

  DBH = pcl::geometry::distance(minDBH,maxDBH);

  std::cout << yellow << "\nHeight." << reset << std::endl;
  std::cout << "------------------------------------------" << std::endl;

  std::map<double,pcl::PointXYZ> min_max_trunkHeight;

  for(pcl::PointCloud<pcl::PointXYZ>::iterator it=trunk_cloud->begin();it!=trunk_cloud->end(); ++it){
    pcl::PointXYZ pt = pcl::PointXYZ(it->x,it->y,it->z);
    min_max_trunkHeight[pt.y]=pt;
  }

  it1 = min_max_trunkHeight.begin();
  minTH = it1->second;

  it2 = std::prev(min_max_trunkHeight.end());
  maxTH = it2->second;

  feature << "minTH pt3d:" << minTH << std::endl;
  feature << "maxTH pt3d:" << maxTH << std::endl;

  maxTH.x = minTH.x;
  maxTH.z = minTH.z;

  //minTH.x = maxTH.x;
  //minTH.z = maxTH.z;

  std::cout << "MaxTH: " << maxTH << std::endl;
  std::cout << "MinTH: " << minTH << std::endl;

  height_trunk = pcl::geometry::distance(minTH,maxTH);

  std::cout << blue << "\nEstimating crown features..." << reset << std::endl;
  std::cout << "------------------------------------------" << std::endl;

  std::cout << yellow << "\nHeight." << reset << std::endl;
  std::cout << "------------------------------------------" << std::endl;

  std::map<double,pcl::PointXYZ> min_max_crownHeight;

  for(pcl::PointCloud<pcl::PointXYZ>::iterator it=crown_cloud->begin();it!=crown_cloud->end(); ++it){
    pcl::PointXYZ pt = pcl::PointXYZ(it->x,it->y,it->z);
    min_max_crownHeight[pt.y]=pt;
  }

  it1 = min_max_crownHeight.begin();
  minCH = it1->second;

  it2 = std::prev(min_max_crownHeight.end());
  maxCH = it2->second;

  feature << "minCH pt3d:" << minCH << std::endl;
  feature << "maxCH pt3d:" << maxCH << std::endl;

  minCH.x = maxCH.x;
  minCH.z = maxCH.z;

  height_crown = pcl::geometry::distance(minCH,maxCH);

  std::cout << "MaxCH: " << maxCH << std::endl;
  std::cout << "MinCH: " << minCH << std::endl;

  std::cout << blue << "\nEstimating other features..." << reset << std::endl;
  std::cout << "------------------------------------------" << std::endl;

  std::cout << yellow << "\nTotal height." << reset << std::endl;
  std::cout << "------------------------------------------" << std::endl;

  total_height = height_crown+height_trunk;
  ROS_INFO("H1: %f",height_crown);
  ROS_INFO("H2: %f",height_trunk);

  std::cout << yellow << "\nCrown volume." << reset << std::endl;
  std::cout << "------------------------------------------" << std::endl;

  std::vector<pcl::PointXYZ> pts_for_DBH_5m;

  for(pcl::PointCloud<pcl::PointXYZ>::iterator it=trunk_cloud->begin();it!=trunk_cloud->end(); ++it){

    pcl::PointXYZ pt = pcl::PointXYZ(it->x,it->y,it->z);
    if(pt.y>480 and pt.y < 580){
      pts_for_DBH_5m.push_back(pt);
    }else{
      continue;
    }
  }

  if(pts_for_DBH_5m.size()<=0){
    ROS_ERROR("No points at 5m, using reference 1.3m!\n");
    pts_for_DBH_5m.push_back(minDBH);
    pts_for_DBH_5m.push_back(maxDBH);
  }

  ROS_INFO("Points between 5.3+/-0.5cm: %lu",pts_for_DBH_5m.size());
  std::map<double,pcl::PointXYZ> min_max_DBH_5m;

  for(std::vector<pcl::PointXYZ>::iterator it=pts_for_DBH_5m.begin();it!=pts_for_DBH_5m.end(); ++it){
    pcl::PointXYZ pt = pcl::PointXYZ(it->x,it->y,it->z);
    min_max_DBH_5m[pt.x] = pt;
  }

  it1 = min_max_DBH_5m.begin();
  minDBH5 = it1->second;

  it2 = std::prev(min_max_DBH_5m.end());
  maxDBH5 = it2->second;

  std::cout << "MinDBH5:" << minDBH5 << std::endl;
  std::cout << "MaxDBH5:" << maxDBH5 << std::endl;

  feature << "minDBH5m pt3d:" << minDBH5 << std::endl;
  feature << "maxDBH5m pt3d:" << maxDBH5 << std::endl;
  feature << "--------------------------" << std::endl;

  minDBH5.y = maxDBH5.y;
  minDBH5.z = maxDBH5.z;
  double DBH_5m  = pcl::geometry::distance(minDBH5,maxDBH5);

  factor_morfico = DBH_5m/DBH;
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
  std::cout<< std::fixed;
  std::cout << "------ Crown volume:" << crown_volume << "cm^3" << std::endl;
  std::cout << "---------------------------------------" << std::endl;
  std::cout << green << "OTHERS FEATURES" << reset << std::endl;
  std::cout << "---------------------------------------" << std::endl;
  std::cout << "------ Total height:" << total_height << "cm" << std::endl;
  std::cout << "------ Factor morfico:" << factor_morfico << std::endl;
  std::cout << "************************************************" << std::endl;
  std::cout << "************************************************" << std::endl;

  std::cout << "Saving results in:" << output_dir << std::endl;

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
