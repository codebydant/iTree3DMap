#include "include/Segmentation.h"
#include <pcl/common/time.h>

void Segmentation::extractTree(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
                               pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_segmented){

  std::cout << "************************************************" << std::endl;
  std::cout << "              SEGMENTATION                      " << std::endl;
  std::cout << "************************************************" << std::endl;

  if(cloud->size() <= 0){
     std::cout << "Cloud reading failed. no data points found" << std::endl;
     std::exit(-1);
  }     

  /*OUTLIER FILTER*/
  std::cout << "Removing outliers points..." << std::endl;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>());
  /*
  pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;
  outrem.setInputCloud(cloud);
  outrem.setRadiusSearch(10);
  outrem.setMinNeighborsInRadius(10);
  outrem.filter(*cloud_filtered);
  */
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
  sor.setInputCloud (cloud);
  sor.setMeanK (50);
  sor.setStddevMulThresh (1.0);
  sor.filter (*cloud_filtered);
  std::cout << "Outliers points --> [FILTER]." << std::endl;

  /*GROUND SEGMENTATION*/
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ground_remove (new pcl::PointCloud<pcl::PointXYZRGB>());
  groundModelSegmentation(cloud_filtered,cloud_ground_remove,false);

  /*TREE SEGMENTATION*/
  std::cout << "Tree cloud segmentation with color base growing..." << std::endl;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr tree_segmented (new pcl::PointCloud<pcl::PointXYZRGB>());
  color_based_growing_segmentation(cloud_ground_remove,tree_segmented,true);

  /*CONVERT XYZRGB TO XYZ*/
  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ground_xyz (new pcl::PointCloud<pcl::PointXYZRGB>());
  //pcl::copyPointCloud(*cloud_ground_remove,*cloud_ground_xyz);

  std::cout << "Segmentation proccess --> [OK]" << std::endl;
  std::cout << "Saving segmentation 3d mapping file with prefix --> MAP3D_segmented.pcd" << std::endl;

  /*
  std::string prefix2 = output_dir;
  prefix2 += "/";
  prefix2 += output_pcd_files;
  prefix2 += "/";
  prefix2 += "MAP3D_segmented.pcd";
  */
  //pcl::io::savePCDFileBinary("best_tree_segmented.pcd", *cloud_ground_xyz);

}

void Segmentation::groundModelSegmentation(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
                                           pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_plane_segmented,
                                           bool show){

  std::cout << "Ground model segmentation..." << std::endl;

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

  std::cout << "Preparing options for segmentation..." << std::endl;
  int maxIterations = 1000;
  double distanceThresh = 0.2;
  bool optimizeCoeff = true;
  int modelType = pcl::SACMODEL_PLANE;
  int methodType = pcl::SAC_RANSAC;

  std::cout << "OPTIONS:\n"
            << "Input cloud:" << cloud->size() << "\n"
            << "Optimize coefficients:" << optimizeCoeff << "\n"
            << "Model type:" << modelType << "\n"
            << "Method type:" << methodType << "\n"
            << "Max iterations:" << maxIterations << "\n"
            << "Distance threshold:" << distanceThresh  << std::endl;

  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  seg.setOptimizeCoefficients(optimizeCoeff);
  seg.setModelType(modelType);
  seg.setMethodType(methodType);
  seg.setMaxIterations(maxIterations);
  seg.setDistanceThreshold(distanceThresh);
  //seg.setIndices(indices);
  seg.setInputCloud(cloud);

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;

  int nr_points = (int)cloud->points.size();

  size_t tam;
  size_t temp;
  // While 30% of the original cloud is still there
  while(cloud->points.size() > 0.1 * nr_points){

    tam = temp;
    temp = cloud->points.size();
    if(tam == cloud->points.size()){break;}

    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud(cloud);
    pcl::ScopeTime scopeTime("Test loop");
    seg.segment(*inliers, *coefficients);

    if(inliers->indices.size() == 0){

      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the inliers
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud);
    std::cout << "PointCloud representing the planar component: " << cloud->width * cloud->height << " data points."
              << std::endl;
  }

  pcl::copyPointCloud(*cloud,*cloud_plane_segmented);
  std::cout << "------------------------------------------------" << std::endl;

  if(show){

    pcl::visualization::PCLVisualizer viewer = pcl::visualization::PCLVisualizer("Ground Model Segmented",true);

    viewer.setPosition(0,0);
    viewer.setSize(640,480);
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0);
    viewer.addCoordinateSystem(1.0, "ucs", 0);
    viewer.setCameraPosition(0,0,1,0,0,0);
    pcl::PointXYZ p1, p2, p3;
    p1.getArray3fMap() << 1, 0, 0;
    p2.getArray3fMap() << 0, 1, 0;
    p3.getArray3fMap() << 0,0.1,1;

    viewer.addText3D("x", p1, 0.2, 1, 0, 0, "x_");
    viewer.addText3D("y", p2, 0.2, 0, 1, 0, "y_");
    viewer.addText3D ("z", p3, 0.2, 0, 0, 1, "z_");
    viewer.addPointCloud(cloud_plane_segmented,"groundModelSegmentation");
    viewer.resetCamera();

    std::cout << "Press [q] to continue process segmentation!" << std::endl;

    while(!viewer.wasStopped ()) {
           viewer.spin();
    }
  }
}

void Segmentation::based_growing_segmentation(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                              pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_normal_segmented){

  pcl::search::Search<pcl::PointXYZ>::Ptr tree =
  boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);

  pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod(tree);
  normal_estimator.setInputCloud(cloud);
  normal_estimator.setKSearch(50);
  normal_estimator.compute(*normals);

  pcl::IndicesPtr indices (new std::vector <int>);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.0, 10.0);
  pass.filter(*indices);

  pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
  reg.setMinClusterSize(50);
  reg.setMaxClusterSize(1000000);
  reg.setSearchMethod(tree);
  reg.setNumberOfNeighbours(30);
  reg.setInputCloud(cloud);
  //reg.setIndices (indices);
  reg.setInputNormals(normals);
  reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
  reg.setCurvatureThreshold(1.0);

  std::vector <pcl::PointIndices> clusters;
  reg.extract(clusters);

  std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
  std::cout << "First cluster has " << clusters[0].indices.size () << " points." << endl;
  std::cout << "These are the indices of the points of the initial" <<
  std::endl << "cloud that belong to the first cluster:" << std::endl;

  int counter = 0;
  while(counter < clusters[0].indices.size()){

    std::cout << clusters[0].indices[counter] << ", ";
    counter++;
    if(counter % 10 == 0){
      std::cout << std::endl;
    }
  }

  std::cout << std::endl;

  cloud_normal_segmented = reg.getColoredCloud ();
}

void Segmentation::color_based_growing_segmentation(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
                                                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& tree_cloud_segmented,
                                                    bool show){

  pcl::search::Search<pcl::PointXYZRGB>::Ptr tree =
  boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> > (new pcl::search::KdTree<pcl::PointXYZRGB>);

  std::cout << "Preparing options for segmentation..." << std::endl;
  int minClusters = 600;
  int maxClusters = 1000000;
  int numberNeighbours = 100;
  int pointColorThresh = 6;
  int regioncolorThresh = 5;
  double distanceThresh = 30.0;

  std::cout << "OPTIONS:\n"
            << "Input cloud:" << cloud->size() << "\n"
            << "Point color threshold:" << pointColorThresh << "\n"
            << "Number of neighbours:" << numberNeighbours << "\n"
            << "Region color threshold:" << regioncolorThresh << "\n"
            << "Distance threshold:" << distanceThresh  << "\n"
            << "Min clusters:" << minClusters << "\n"
            << "Max clusters:" << maxClusters << std::endl;

  pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
  reg.setInputCloud(cloud);
  //reg.setIndices(indices);
  reg.setSearchMethod(tree);
  reg.setDistanceThreshold(distanceThresh);
  reg.setPointColorThreshold(pointColorThresh);
  reg.setRegionColorThreshold(regioncolorThresh);

  //reg.setInputNormals (normals);
  reg.setNumberOfNeighbours(numberNeighbours);
  reg.setMinClusterSize(minClusters); // It means that after the segmentation is done all clusters that have less points
  reg.setMaxClusterSize(maxClusters);// then was set as minimum(or have more than maximum) will be discarded.

  std::cout << "Extracting clusters..." << std::endl;
  std::vector <pcl::PointIndices> clusters;
  reg.extract(clusters);

  if(clusters.size()<=0){
       std::cerr << "Error: could not extract enough clusters." << std::endl;
       std::cout << "Extract:" << clusters.size() << " clusters. Min=600" << std::endl;
       std::exit(-1);
  }

  std::cout << "Extract:" << clusters.size() << " clusters" << std::endl;
  std::cout << "Getting color cloud..." << std::endl;

  tree_cloud_segmented = reg.getColoredCloud();
  std::cout << "------------------------------------------------" << std::endl;

  if(show){

    pcl::visualization::PCLVisualizer viewer = pcl::visualization::PCLVisualizer("Tree cloud segmented",true);

    viewer.setPosition(640,0);
    viewer.setSize(640,480);
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
    viewer.addCoordinateSystem(1.0, "ucs", 0);
    viewer.setCameraPosition(0,0,1,0,0,0);
    pcl::PointXYZ p1, p2, p3;
    p1.getArray3fMap() << 1, 0, 0;
    p2.getArray3fMap() << 0, 1, 0;
    p3.getArray3fMap() << 0,0.1,1;

    viewer.addText3D("x", p1, 0.2, 1, 0, 0, "x_");
    viewer.addText3D("y", p2, 0.2, 0, 1, 0, "y_");
    viewer.addText3D ("z", p3, 0.2, 0, 0, 1, "z_");
    viewer.addPointCloud(tree_cloud_segmented,"tree_cloud");
    viewer.resetCamera();

    std::cout << "Press [q] to continue segmentation process!" << std::endl;

    while(!viewer.wasStopped ()) {
           viewer.spin();
    }
  }
}

void Segmentation::trunkSegmentation(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                     pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_trunk){

  pcl::PassThrough<pcl::PointXYZ> pass;
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;

  pcl::ExtractIndices<pcl::PointXYZ> extract;
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
  pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);

  //Build a passthrough filter to remove spurious NaNs
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.0, 20.0);
  pass.filter(*cloud_filtered);
  std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;

  // Estimate point normals
  ne.setSearchMethod(tree);
  ne.setInputCloud(cloud_filtered);
  ne.setKSearch(50);
  ne.compute(*cloud_normals);

  //Create the segmentation object for the planar model and set all the parameters
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
  seg.setNormalDistanceWeight(0.1);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(100);
  seg.setDistanceThreshold(0.03);
  seg.setInputCloud(cloud_filtered);
  seg.setInputNormals(cloud_normals);

  // Obtain the plane inliers and coefficients
  seg.segment(*inliers_plane, *coefficients_plane);
  std::cout << "Plane coefficients: " << *coefficients_plane << std::endl;

  // Extract the planar inliers from the input cloud
  extract.setInputCloud(cloud_filtered);
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
  seg.setNormalDistanceWeight(0.1);
  seg.setMaxIterations(10000);
  seg.setDistanceThreshold(0.05);
  seg.setRadiusLimits(0, 0.1);
  seg.setInputCloud(cloud_filtered2);
  seg.setInputNormals(cloud_normals2);

  // Obtain the cylinder inliers and coefficients
  seg.segment(*inliers_cylinder, *coefficients_cylinder);
  std::cout << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

  extract.setInputCloud(cloud_filtered2);
  extract.setIndices(inliers_cylinder);
  extract.setNegative(false);
  extract.filter(*cloud_trunk);
}
