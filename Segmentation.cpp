#include "include/Segmentation.h"

pcl::PointXYZ trunkMin;
pcl::PointXYZ trunkMax;

bool Segmentation::extractTree(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
                               const std::string& output_path,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr& trunk_cloud,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr& tree_segmented,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr& crown_segmented){

  std::cout << "\n************************************************" << std::endl;
  std::cout << "              SEGMENTATION                      " << std::endl;
  std::cout << "************************************************" << std::endl;

  if(cloud->points.size() <= 0){
     PCL_ERROR("Input point cloud has no data!\n");
     return false;
  }

  cloud->header.frame_id = "principal_cloud";
  trunk_cloud->header.frame_id = "trunk_cloud";
  crown_segmented->header.frame_id = "crown_cloud";
  tree_segmented->header.frame_id = "tree_without_trunk";
  /*
  ros::Time time_st = ros::Time::now();

  cloud->header.stamp= time_st.toNSec()/1e3;  
  //cloud->header.stamp = ros::Time::now().toNSec();
  trunk_cloud->header.stamp = time_st.toNSec()/1e3;  
  tree_segmented->header.stamp = time_st.toNSec()/1e3;  
  crown_segmented->header.stamp = time_st.toNSec()/1e3;  
*/
  /*CONVERT XYZRGB TO XYZ*/
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp (new pcl::PointCloud<pcl::PointXYZ>());
  pcl::copyPointCloud(*cloud,*temp);

  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(temp);
  sor.setMeanK(200);
  sor.setStddevMulThresh(1.0);
  sor.filter(*cloud_xyz);

  /*TRUNK SEGMENTATION*/
  std::cout << "Trunk cloud segmentation with: Cylinder Model Segmentation..." << std::endl;
  trunkSegmentation(cloud_xyz,tree_segmented,trunk_cloud);

  std::cout << "Crown cloud segmentation with: Euclidean Cluster Extraction..." << std::endl;
  crownSegmentation(tree_segmented,crown_segmented);

   pcl::console::TicToc tt2;
  PCL_INFO("\nSaving 3D mapping segmented...");

  std::string prefix = output_path;
  prefix += "/";
  prefix += "3D_Mapping";
  prefix += "/";

  std::string prefix1 = prefix;
  prefix1 += "MAP3D_trunk_segmented.pcd";

  std::string prefix2 = prefix;
  prefix2 += "MAP3D_trunk_segmented.ply";

  std::string prefix3 = prefix;
  prefix3 += "MAP3D_tree_segmented.pcd";

  std::string prefix4 = prefix;
  prefix4 += "MAP3D_tree_segmented.ply";

  std::string prefix5 = prefix;
  prefix5 += "MAP3D_crown_segmented.pcd";

  std::string prefix6 = prefix;
  prefix6 += "MAP3D_crown_segmented.ply";

  pcl::io::savePCDFileBinary(prefix1.c_str(), *trunk_cloud);
  pcl::io::savePLYFileBinary(prefix2.c_str(), *trunk_cloud);
  pcl::io::savePCDFileBinary(prefix3.c_str(), *tree_segmented);
  pcl::io::savePLYFileBinary(prefix4.c_str(), *tree_segmented);
  pcl::io::savePCDFileBinary(prefix5.c_str(), *crown_segmented);
  pcl::io::savePLYFileBinary(prefix6.c_str(), *crown_segmented);

  pcl::console::print_info ("[done, ");
  pcl::console::print_value ("%g", tt2.toc ());
  pcl::console::print_info (" ms : ");
  pcl::console::print_value ("%d", trunk_cloud->points.size ());
  pcl::console::print_info (" points]\n");

  std::cout << "Segmentation proccess --> [OK]" << std::endl;  

  return true;

}

bool Segmentation::trunkSegmentation(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                     pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_without_trunk,
                                     pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_trunk){

  if(cloud->points.size() <= 0){
     PCL_ERROR("Input point cloud has no data!\n");
     return false;
  }

  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;

  pcl::ExtractIndices<pcl::PointXYZ> extract;
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());

 // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr trunk_seg (new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr rest_seg (new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>());
  pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);

  // Estimate point normals
  ne.setSearchMethod(tree);
  ne.setInputCloud(cloud);
  ne.setKSearch(50);
  ne.compute(*cloud_normals);

  //Create the segmentation object for the planar model and set all the parameters
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
  seg.setNormalDistanceWeight(0.1);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(500);
  seg.setDistanceThreshold(0.5);
  seg.setInputCloud(cloud);
  seg.setInputNormals(cloud_normals);

  // Obtain the plane inliers and coefficients
  seg.segment(*inliers_plane, *coefficients_plane);
  std::cout << "Plane coefficients: " << *coefficients_plane << std::endl;

  // Extract the planar inliers from the input cloud
  extract.setInputCloud(cloud);
  extract.setIndices(inliers_plane);
  extract.setNegative(false);

  // Remove the planar inliers, extract the rest
  extract.setNegative(true);
  extract.filter(*cloud_filtered2);
  extract_normals.setNegative(true);
  extract_normals.setInputCloud(cloud_normals);
  extract_normals.setIndices(inliers_plane);
  extract_normals.filter(*cloud_normals2);

  // Create the segmentation object for cylinder segmentation and set all the parameters
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_CYLINDER);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight(0.5);
  seg.setMaxIterations(10000);
  seg.setDistanceThreshold(8);
  seg.setRadiusLimits(0, 60);
  seg.setInputCloud(cloud_filtered2);
  seg.setInputNormals(cloud_normals2);

  // Obtain the cylinder inliers and coefficients
  seg.segment(*inliers_cylinder, *coefficients_cylinder);
  std::cout << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

  extract.setInputCloud(cloud_filtered2);
  extract.setIndices(inliers_cylinder);
  extract.setNegative(false);
  extract.filter(*trunk_seg);

  extract.setNegative(true);
  extract.filter(*rest_seg);
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr trunk_seg_filtered (new pcl::PointCloud<pcl::PointXYZ>());  
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor2;
  sor2.setInputCloud(trunk_seg);
  sor2.setMeanK(80);
  sor2.setStddevMulThresh(1.0);
  sor2.filter(*trunk_seg_filtered);

  Eigen::Matrix4f align_cloud = Eigen::Matrix4f::Identity();

  pcl::PointXYZ minH,maxH;
  std::map<double,pcl::PointXYZ> minMaxValues;

  for(pcl::PointCloud<pcl::PointXYZ>::iterator it=trunk_seg_filtered->begin(); it!=trunk_seg_filtered->end(); ++it){
    pcl::PointXYZ p = pcl::PointXYZ(it->x,it->y,it->z);
    minMaxValues[p.y] = p;
  }

  std::map<double,pcl::PointXYZ>::iterator it1 = minMaxValues.begin();
  minH = it1->second;

  std::map<double,pcl::PointXYZ>::iterator it2 = std::prev(minMaxValues.end());
  maxH = it2->second;

  std::cout << "Min trunk it:" << minH << std::endl;
  std::cout << "Max trunk it:" << maxH << std::endl;

  double offsetY = std::abs(minH.y - 0);
  std::cout << "Offset trunk Y:" << offsetY << std::endl;

  if(minH.y < 0){


    align_cloud << 1,   0,    0,    0,           //       |1  0    0   x       |
                   0,   1,    0,    offsetY,     //   t = |0   1   0   y+offset| => traslation matrix
                   0,   0,    1,    0,           //       |0   0   1   z       |
                   0,   0,    0,    1;           //       |0   0   0   1       |

  }else{


    align_cloud << 1,   0,    0,    0,
                   0,   1,    0,    -offsetY,
                   0,   0,    1,    0,
                   0,   0,    0,    1;
  }

  pcl::console::TicToc tt;
  std::cout << "\nHere is the translation matrix transform:\n" << align_cloud << "\n" << std::endl;
  PCL_INFO("Executing the transformation...");

  pcl::transformPointCloud(*trunk_seg_filtered, *cloud_trunk, align_cloud);
  pcl::transformPointCloud(*rest_seg, *cloud_without_trunk, align_cloud);

  pcl::console::print_info ("[done, ");
  pcl::console::print_value ("%g", tt.toc ());
  pcl::console::print_info (" ms : ");
  pcl::console::print_value ("%d", cloud_trunk->points.size());
  pcl::console::print_info (" points]\n");

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

  std::cout << "New min:" << minH1 << std::endl;
  std::cout << "New max:" << maxH2 << std::endl;
  trunkMin = minH1;
  trunkMax = maxH2;
  trunkMin.x=trunkMax.x;
  trunkMin.z=trunkMax.z;
  double dist = pcl::geometry::distance(trunkMin,trunkMax);
  std::cout << "Mean:" << dist << "\n" << std::endl;

  return true;

}

bool Segmentation::crownSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_without_trunk,
                                     pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_crown){

  if(cloud_without_trunk->points.size() <= 0){
    PCL_ERROR("Input point cloud has no data!");
    return false;
  }

  std::cout << "Filtering points data with: PassThrough filter..." << std::endl;

  PCL_INFO("PointCloud before filtering has %lu %s", cloud_without_trunk->points.size()," data points.\n");

  double meanMedian = pcl::geometry::distance(trunkMin,trunkMax);
  PCL_INFO("\nMean median: %f",meanMedian);
  std::cout << std::endl;
  PCL_INFO("Mean median/2: %f",meanMedian/2);
  std::cout << std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>());

  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud_without_trunk);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(-900, meanMedian/2);
  pass.setFilterLimitsNegative(true);
  pass.filter(*cloud_filtered);

  std::cout << "\nPointCloud after filtering has: " << cloud_filtered->points.size ()
            << " data points." << std::endl;  

  std::cout << "\nCreating the KdTree object for the search method of the extraction..." << std::endl;

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(14); // 2cm
  ec.setMinClusterSize(30);
  ec.setMaxClusterSize(25000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud_filtered);
  ec.extract(cluster_indices);

  std::cout << "Cluster tolerance:" << 14 << std::endl
            << "Min cluster size:" << 30 << std::endl
            << "Max cluster size:" << 25000 << std::endl
            << "Search method: KdTree" << std::endl;

  PCL_INFO("clusters:%lu", cluster_indices.size());
  std::cout << std::endl;

  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> vecClusters;

  for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end(); ++it){

      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
      for(std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){

      cloud_cluster->points.push_back(cloud_filtered->points[*pit]);

      }

      cloud_cluster->width = cloud_cluster->points.size ();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;

      vecClusters.push_back(cloud_cluster);
    }

 pcl::PointCloud<pcl::PointXYZ>::Ptr temp3 (new pcl::PointCloud<pcl::PointXYZ>());
    for(size_t i=0;i<vecClusters.size()-1;i++){
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
      cloud_cluster = vecClusters.at(i);

      for(size_t j=0;j< cloud_cluster->points.size();j++){

      temp3->points.push_back(cloud_cluster->points.at(j));

    }

      temp3->width = cloud_crown->points.size ();
      temp3->height = 1;
      temp3->is_dense = true;

    }

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(temp3);
    sor.setMeanK(100);
    sor.setStddevMulThresh(0.2);
    sor.filter(*cloud_crown);

}










