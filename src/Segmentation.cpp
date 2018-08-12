#include "include/Segmentation.h"
#include <pcl/2d/morphology.h>

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

  std::cout << "Saving segmentation 3d mapping!" << std::endl;

  std::string output_pcd_files = "3D_Mapping";
  std::string prefix1 = output_path;
  prefix1 += "/";
  prefix1 += output_pcd_files;
  prefix1 += "/";
  prefix1 += "MAP3D_trunk_segmented.pcd";

  std::string prefix2 = output_path;
  prefix2 += "/";
  prefix2 += output_pcd_files;
  prefix2 += "/";
  prefix2 += "MAP3D_trunk_segmented.ply";

  std::string prefix3 = output_path;
  prefix3 += "/";
  prefix3 += output_pcd_files;
  prefix3 += "/";
  prefix3 += "MAP3D_tree_segmented.pcd";

  std::string prefix4 = output_path;
  prefix4 += "/";
  prefix4 += output_pcd_files;
  prefix4 += "/";
  prefix4 += "MAP3D_tree_segmented.ply";

  std::string prefix5 = output_path;
  prefix5 += "/";
  prefix5 += output_pcd_files;
  prefix5 += "/";
  prefix5 += "MAP3D_crown_segmented.pcd";

  std::string prefix6 = output_path;
  prefix6 += "/";
  prefix6 += output_pcd_files;
  prefix6 += "/";
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
  seg.setDistanceThreshold(0.05);
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
  seg.setNormalDistanceWeight(0.01);
  seg.setMaxIterations(10000);
  seg.setDistanceThreshold(8);
  seg.setRadiusLimits(0, 180);
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

  if(show){

    pcl::visualization::PCLVisualizer viewer = pcl::visualization::PCLVisualizer("Trunk cloud segmented",true);

    viewer.setPosition(640,0);
    viewer.setSize(640,480);
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
    viewer.addCoordinateSystem();
    viewer.setCameraPosition(0,0,1,0,0,0);
    pcl::PointXYZ p1, p2, p3;
    p1.getArray3fMap() << 1, 0, 0;
    p2.getArray3fMap() << 0, 1, 0;
    p3.getArray3fMap() << 0,0.1,1;

    viewer.addText3D("x", p1, 0.2, 1, 0, 0, "x_");
    viewer.addText3D("y", p2, 0.2, 0, 1, 0, "y_");
    viewer.addText3D ("z", p3, 0.2, 0, 0, 1, "z_");
    viewer.addPointCloud<pcl::PointXYZ>(cloud_without_trunk,"trunk_cloud");
    viewer.resetCamera();

    std::cout << "Press [q] to continue segmentation process!" << std::endl;

    while(!viewer.wasStopped ()) {
           viewer.spin();
    }
  }

}

void Segmentation::crownSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_without_trunk,
                                     pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_crown){


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

}
