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
     ROS_ERROR("Input point cloud has no data!");
     return false;
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

  return true;

}

bool Segmentation::trunkSegmentation(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                     pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_without_trunk,
                                     pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_trunk){

  if(cloud->points.size() <= 0){
     ROS_ERROR("Input point cloud has no data!");
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
  std::map<double,pcl::PointXYZ> minMaxValues;

  for(pcl::PointCloud<pcl::PointXYZ>::iterator it=temp->begin(); it!=temp->end(); ++it){
    pcl::PointXYZ p = pcl::PointXYZ(it->x,it->y,it->z);
    minMaxValues[p.y] = p;
  }

  std::map<double,pcl::PointXYZ>::iterator it1 = minMaxValues.begin();
  minH = it1->second;

  std::map<double,pcl::PointXYZ>::iterator it2 = std::prev(minMaxValues.end());
  maxH = it2->second;

  std::cout << "min trunk it:" << minH << std::endl;
  std::cout << "max trunk it:" << maxH << std::endl;

  double offsetY = std::abs(minH.y - 0);
  std::cout << "offset trunk Y:" << offsetY << std::endl;

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

  //std::cout << "Here is the matrix transform:\n" << align_cloud << std::endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp3 (new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp4 (new pcl::PointCloud<pcl::PointXYZ>());

  ROS_INFO("Executing the transformation...");
  pcl::transformPointCloud(*temp, *cloud_trunk, align_cloud);
  pcl::transformPointCloud(*temp2, *cloud_without_trunk, align_cloud);

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
  std::cout << "mean:" << dist << std::endl;
/*
  vtkSmartPointer<vtkPolyData> cloudVTK = vtkSmartPointer<vtkPolyData>::New();
  vtkSmartPointer<vtkPoints> pts = vtkSmartPointer<vtkPoints>::New();

  for(int n=0;n<cloud_trunk->points.size();n++){
    pcl::PointXYZ p = cloud_trunk->points.at(n);
    pts->InsertNextPoint(p.x,p.y,p.z);
  }
  cloudVTK->SetPoints(pts);

  vtkSmartPointer<vtkVertexGlyphFilter> vertexFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
  vertexFilter->SetInputData(cloudVTK);
  vertexFilter->Update();

  vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
  polydata->ShallowCopy(vertexFilter->GetOutput());

  // Create two points, P0 and P1
  double p0[3] = {trunkMin.x, trunkMin.y, trunkMin.z};
  double p1[3] = {trunkMax.x, trunkMax.y, trunkMax.z};

  std::cout << "p1:" << p0[0] << "," << p0[1] << "," << p0[2] << std::endl;
   std::cout << "p2:" << p1[0] << "," << p1[1] << "," << p1[2] << std::endl;

  vtkSmartPointer<vtkLineSource> lineSource = vtkSmartPointer<vtkLineSource>::New();
  lineSource->SetPoint1(p0);
  lineSource->SetPoint2(p1);
  lineSource->Update();

  // Create a mapper and actor
  vtkSmartPointer<vtkPolyDataMapper> mapper1 = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper1->SetInputData(polydata);

  vtkSmartPointer<vtkActor> actor1 = vtkSmartPointer<vtkActor>::New();
  actor1->SetMapper(mapper1);
  actor1->GetProperty()->SetColor(1.0, 1.0, 1.0);
  actor1->GetProperty()->SetPointSize(1);

  // Create a mapper and actor
  vtkSmartPointer<vtkPolyDataMapper> mapper2 = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper2->SetInputConnection(lineSource->GetOutputPort());

  vtkSmartPointer<vtkActor> actor2 = vtkSmartPointer<vtkActor>::New();
  actor2->SetMapper(mapper2);
  actor2->GetProperty()->SetColor(0.0, 1.0, 0.0);
  actor2->GetProperty()->SetPointSize(1);

  // Create a renderer, render window, and interactor
  vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
  renderer->SetBackground(0.0, 0.0, 0.0);
  // Zoom in a little by accessing the camera and invoking its "Zoom" method.
  renderer->ResetCamera();
  vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();

  renderWindow->SetSize(800, 600);
  renderWindow->AddRenderer(renderer);

  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);

  vtkSmartPointer<vtkNamedColors> colors =
      vtkSmartPointer<vtkNamedColors>::New();

  // Add the actor to the scene
  renderer->AddActor(actor1);
  renderer->AddActor(actor2);

  vtkSmartPointer<vtkAxesActor> axes = vtkSmartPointer<vtkAxesActor>::New();
  axes->GetXAxisCaptionActor2D()->GetCaptionTextProperty()->SetColor(colors->GetColor3d("Red").GetData());
  axes->SetScale(3000,3000,3000);

  renderer->AddActor(axes);

  // Render and interact
  renderWindow->SetWindowName("VISUALIZER");
  //renderWindow->SetFullScreen(true);
  renderWindow->Render();

  vtkSmartPointer<vtkInteractorStyleTrackballCamera> style =
      vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New(); //like paraview
  renderWindowInteractor->SetInteractorStyle(style);
  std::cout << "Press [q] to continue" << std::endl;

  vtkSmartPointer<vtkOrientationMarkerWidget> widget =
      vtkSmartPointer<vtkOrientationMarkerWidget>::New();
  widget->SetOutlineColor( 0.9300, 0.5700, 0.1300 );
  widget->SetOrientationMarker( axes );
  widget->SetInteractor( renderWindowInteractor );
  //widget->SetViewport( 0.0, 0.0, 0.4, 0.4 );
  widget->SetEnabled( 1 );
  widget->InteractiveOn();

  renderer->ResetCamera();
  renderWindowInteractor->Start();
*/

/*
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor2;
  sor2.setInputCloud(temp4);
  sor2.setMeanK(80);
  sor2.setStddevMulThresh(1.0);
  sor2.filter(*cloud_trunk);
*/
/*

  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(temp3);
  sor.setMeanK(50);
  sor.setStddevMulThresh(1.0);
  sor.filter(*cloud_without_trunk);
*/

  return true;



}

bool Segmentation::crownSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_without_trunk,
                                     pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_crown){

  if(cloud_without_trunk->points.size() <= 0){
    ROS_ERROR("Input point cloud has no data!");
    return false;
  }

  ROS_INFO("PointCloud before filtering has %lu %s", cloud_without_trunk->points.size()," data points.");

  double meanMedian = pcl::geometry::distance(trunkMin,trunkMax);
  ROS_INFO("Mean median: %f",meanMedian);
  ROS_INFO("Mean median/2: %f",meanMedian/2);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud_without_trunk);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(-900, meanMedian/2);
  pass.setFilterLimitsNegative(true);
  pass.filter(*cloud_filtered);

  std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()
            << " data points." << std::endl;


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

    ROS_INFO("clusters:%lu", cluster_indices.size());
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










