#include "include/Segmentation.h"
#include <pcl/segmentation/extract_clusters.h>

void Segmentation::extractTree(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
                               const std::string& output_path,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr& trunk_cloud,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr& tree_segmented,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr& crown_segmented){

  std::cout << "\n************************************************" << std::endl;
  std::cout << "              SEGMENTATION                      " << std::endl;
  std::cout << "************************************************" << std::endl;

  if(cloud->size() <= 0){
     std::cout << "Cloud reading failed. no data points found" << std::endl;
     std::exit(-1);
  }

  /*CONVERT XYZRGB TO XYZ*/
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>());
  pcl::copyPointCloud(*cloud,*cloud_xyz);

  /*TRUNK SEGMENTATION*/
  std::cout << "Trunk cloud segmentation with cylinder segmentation plane..." << std::endl;
  trunkSegmentation(cloud_xyz,tree_segmented,trunk_cloud,false);
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
                                     pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_trunk,bool show){

  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;

  pcl::ExtractIndices<pcl::PointXYZ> extract;
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());

 // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
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
  extract.filter(*cloud_trunk);

  extract.setNegative(true);
  extract.filter(*cloud_without_trunk);  

}

void Segmentation::crownSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_without_trunk,
                                     pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_crown){

/*
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_SPHERE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold (5);

    seg.setInputCloud(cloud_without_trunk);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size () == 0){
      PCL_ERROR ("Could not estimate a planar model for the given dataset.");

    }

    std::cout << "Model coefficients: " << coefficients->values[0] << " "
                                        << coefficients->values[1] << " "
                                        << coefficients->values[2] << " "
                                        << coefficients->values[3] << std::endl;

    std::cout << "Model inliers: " << inliers->indices.size () << std::endl;


    extract.setInputCloud(cloud_without_trunk);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_crown);
*/



    std::cout << "PointCloud before filtering has: " << cloud_without_trunk->points.size () << " data points." << std::endl; //*
/*
    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    vg.setInputCloud (cloud_without_trunk);
    vg.setLeafSize (0.01f, 0.01f, 0.01f);
    vg.filter (*cloud_filtered);
    std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*
*/
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);
    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PCDWriter writer;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.02);

    int i=0, nr_points = (int) cloud_without_trunk->points.size ();
    /*
    while (cloud_without_trunk->points.size () > 0.7 * nr_points)
    {
      // Segment the largest planar component from the remaining cloud
      seg.setInputCloud (cloud_without_trunk);
      seg.segment (*inliers, *coefficients);
      if (inliers->indices.size () == 0)
      {
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
        break;
      }

      // Extract the planar inliers from the input cloud
      pcl::ExtractIndices<pcl::PointXYZ> extract;
      extract.setInputCloud (cloud_without_trunk);
      extract.setIndices (inliers);
      extract.setNegative (false);

      // Get the points associated with the planar surface
      extract.filter (*cloud_plane);
      std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

      // Remove the planar inliers, extract the rest
      extract.setNegative (true);
      extract.filter (*cloud_f);
      *cloud_without_trunk = *cloud_f;
    }
*/
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud_without_trunk);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (20); // 2cm
    ec.setMinClusterSize (7000);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_without_trunk);
    ec.extract (cluster_indices);

    std::cout << "clusters:" << cluster_indices.size() << std::endl;

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){

      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
      for(std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)

      cloud_crown->points.push_back(cloud_without_trunk->points[*pit]);

      cloud_crown->width = cloud_cluster->points.size ();
      cloud_crown->height = 1;
      cloud_crown->is_dense = true;

    }
}
