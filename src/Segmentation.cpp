#include "include/Segmentation.h"

pcl::PointXYZ trunkMin;
pcl::PointXYZ trunkMax;

void Segmentation::extractTree(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
                               const std::string& output_path,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr& trunk_cloud,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr& tree_segmented,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr& crown_segmented){

  std::cout << "\n************************************************" << std::endl;
  std::cout << "              SEGMENTATION                      " << std::endl;
  std::cout << "************************************************" << std::endl;

  if(cloud->points.size() <= 0){
     PCL_ERROR("Input point cloud has no data!");
     std::exit(-1);
  }

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
  std::cout << "Trunk cloud segmentation with cylinder segmentation plane..." << std::endl;
  trunkSegmentation(cloud_xyz,tree_segmented,trunk_cloud);
  std::cout << "Crown cloud segmentation with SACSegmentationSphere..." << std::endl;
  crownSegmentation(tree_segmented,crown_segmented);

  std::cout << "Saving 3D mapping segmented!" << std::endl;

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
  pcl::io::savePCDFileBinary(prefix5.c_str(), *tree_segmented);
  pcl::io::savePLYFileBinary(prefix6.c_str(), *tree_segmented);

  std::cout << "Segmentation proccess --> [OK]" << std::endl;  

}

void Segmentation::trunkSegmentation(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                     pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_without_trunk,
                                     pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_trunk){

  if(cloud->points.size() <= 0){
     PCL_ERROR("Input point cloud has no data!");
     std::exit(-1);
  }

  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;

  pcl::ExtractIndices<pcl::PointXYZ> extract;
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());

 // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp (new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp2 (new pcl::PointCloud<pcl::PointXYZ>());
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
  extract.filter(*temp);

  extract.setNegative(true);
  extract.filter(*temp2);

  Eigen::Matrix4f align_cloud = Eigen::Matrix4f::Identity();

  pcl::PointXYZ minH,maxH;
  pcl::getMinMax3D(*temp,minH,maxH);

  double offsetY = std::abs(minH.y - 0);
  double offsetX = std::abs(0 - minH.x);
  double offsetZ = std::abs(0 - minH.z);

  std::cout << "offsetY:" << offsetY << std::endl;
  std::cout << "Min:" << minH << std::endl;
  std::cout << "Max:" << maxH << std::endl;

  if(minH.y < 0 ){

                                            //Uniform scaling: vx = vy = vz = s --> Common scale factor
  align_cloud << 1,   0,    0,    0,    //       |vx  0   0   0|
                 0,   1,    0,    offsetY,    //  Sv = |0   vy  0   0| => Scale matrix
                 0,   0,    1,    0,    //       |0   0   vz  0|
                 0,   0,    0,    1;    //       |0   0   0   1|
                                            //https://en.wikipedia.org/wiki/Scaling_(geometry)

  }else{

    //Uniform scaling: vx = vy = vz = s --> Common scale factor
align_cloud << 1,   0,    0,    0,    //       |vx  0   0   0|
0,   1,    0,    -offsetY,    //  Sv = |0   vy  0   0| => Scale matrix
0,   0,    1,    0,    //       |0   0   vz  0|
0,   0,    0,    1;    //       |0   0   0   1|
    //https://en.wikipedia.org/wiki/Scaling_(geometry)

  }

  //std::cout << "Here is the matrix transform:\n" << align_cloud << std::endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp3 (new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp4 (new pcl::PointCloud<pcl::PointXYZ>());

  ROS_INFO("Executing the transformation...");
  pcl::transformPointCloud(*temp, *temp4, align_cloud);
  pcl::transformPointCloud(*temp2, *temp3, align_cloud);

  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor2;
  sor2.setInputCloud(temp4);
  sor2.setMeanK(10);
  sor2.setStddevMulThresh(1.0);
  sor2.filter(*cloud_trunk);


  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(temp3);
  sor.setMeanK(50);
  sor.setStddevMulThresh(1.0);
  sor.filter(*cloud_without_trunk);

  trunkMin = minH;
  trunkMax = maxH;

}

void Segmentation::crownSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_without_trunk,
                                     pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_crown){

  if(cloud_without_trunk->points.size() <= 0){
    PCL_ERROR("Input point cloud has no data!");
    std::exit(-1);
  }

  std::cout << "PointCloud before filtering has: " << cloud_without_trunk->points.size ()
            << " data points." << std::endl;

  double meanMedian = pcl::geometry::distance(trunkMin,trunkMax);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud_without_trunk);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(-900, meanMedian/2);
  pass.setFilterLimitsNegative(true);
  pass.filter(*cloud_filtered);


/*

      for(int it =0; it< cloud_filtered->points.size(); it++){

        cloud_crown->points.push_back(cloud_filtered->points.at(it));

      }

      cloud_crown->width = cloud_filtered->points.size ();
      cloud_crown->height = 1;
      cloud_crown->is_dense = true;

*/

    // Creating the KdTree object for the search method of the extraction
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

    std::cout << "clusters:" << cluster_indices.size() << std::endl;
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










