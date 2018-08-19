/*********************************
           HEADERS
**********************************/
#include "include/DendrometryE.h"

/*********************************
      MAIN FUNCTION
**********************************/
int main(int argc, char **argv){

  Utilities::help();

  /*************************
  STEP 1: ROS INTERFACE
  **************************/
  /*
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("input", 1000, subscriberCallback);
  ros::spinOnce();
  */

  /*************************
  STEP 2: 3D MAPPING
  **************************/
  bool success = Utilities::run_openMVG();
  if(not success){
    std::cout << "Could not get 3D Model. Failed dendrometric estimation." << std::endl;
    return -1;
  }

  /*************************
  STEP 3: GET SCALE FACTOR
  **************************/
  double scale; std::string output_dir;
  pcl::PointCloud<pcl::PointXYZ>::Ptr Map3D (new pcl::PointCloud<pcl::PointXYZ>());
  success = Utilities::getScaleFactor(Map3D,scale,output_dir);
  if(not success or scale <=0){
    std::cout << "Using scale factor = 2" << std::endl;
    scale = 2;
  }
  std::cout << "Map3D points:" << Map3D->points.size() << std::endl;

  /*************************
  STEP 4: DENSIFICATION
  **************************/
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr Map3DDense (new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::io::loadPCDFile("MAP3D_dense.pcd",*Map3DDense);
  auto start = std::chrono::high_resolution_clock::now();
  //Utilities::createPMVS_Files();
  //Utilities::densifyWithPMVS(Map3DDense);
  auto end = std::chrono::high_resolution_clock::now();
  auto difference = std::chrono::duration_cast<std::chrono::seconds>(end - start).count();
  std::cout << "Dense Map Time: " << difference << " seconds" << std::endl;

  /*************************
  STEP 5: UNIFORM SCALING
  **************************/
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_3dMap_scaled (new pcl::PointCloud<pcl::PointXYZRGB>());
  Utilities::uniformScaling(Map3DDense,cloud_3dMap_scaled,scale);

  /*************************
  STEP 6: SEGMENTATION
  **************************/  
  pcl::PointCloud<pcl::PointXYZ>::Ptr trunk_segmented (new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr tree_segmented (new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr crown_segmented (new pcl::PointCloud<pcl::PointXYZ>());
  start = std::chrono::high_resolution_clock::now();
  Segmentation::extractTree(cloud_3dMap_scaled,output_dir,trunk_segmented,tree_segmented,crown_segmented);
  end = std::chrono::high_resolution_clock::now();
  difference = std::chrono::duration_cast<std::chrono::seconds>(end - start).count();
  std::cout << "Trunk segmentation time: " << difference << " seconds" << std::endl;

  /*************************
  STEP 7: DENDROMETRY MEASUREMENTS
  **************************/
  pcl::PointXYZ minDBH,maxDBH,minTH,maxTH,minCH,maxCH,minDBH5,maxDBH5;
  Dendrometry::estimate(trunk_segmented,crown_segmented,output_dir,minDBH,maxDBH,minTH,maxTH,minCH,maxCH,minDBH5,maxDBH5);

  /*************************
  PCL VISUALIZER
  **************************/
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("VISUALIZER",true));

  Display* d = XOpenDisplay(NULL);
  Screen*  s = DefaultScreenOfDisplay(d);

  int y = s->height;
  int x = s->width;

  viewer->setSize(x,y);

  int PORT1 = 0;
  viewer->createViewPort(0.0, 0.5, 0.33, 1.0, PORT1);
  viewer->setBackgroundColor (0, 0, 0, PORT1);
  viewer->addText("MAPPING", 10, 10, "PORT1", PORT1);

  int PORT2 = 0;
  viewer->createViewPort(0.33, 0.5, 0.66, 1.0, PORT2);
  viewer->setBackgroundColor (0, 0, 0, PORT2);
  viewer->addText("DENSE MAPPING", 10, 10, "PORT2", PORT2);

  int PORT3 = 0;
  viewer->createViewPort(0.66, 0.5, 1.0, 1.0, PORT3);
  viewer->setBackgroundColor (0, 0, 0, PORT3);
  //viewer->addLine(ptt1,pt2,0,255,0 ,"lenght",PORT3);
  viewer->addText("SCALE MAPPING", 10, 10, "PORT3", PORT3);

  int PORT4 = 0;
  viewer->createViewPort(0.0, 0.0, 0.33, 0.5, PORT4);
  viewer->setBackgroundColor (0, 0, 0, PORT4);
  viewer->addText("CROWN", 10, 10, "PORT4", PORT4);

  int PORT5 = 0;
  viewer->createViewPort(0.33, 0.0, 0.66, 0.5, PORT5);
  viewer->setBackgroundColor (0, 0, 0, PORT5);
  viewer->addText("TREE SEGMENTED", 10, 10, "PORT5", PORT5);

  int PORT6 = 0;
  viewer->createViewPort(0.66, 0.0, 1.0, 0.5, PORT6);
  viewer->setBackgroundColor (0, 0, 0, PORT6);
  viewer->addText("TRUNK", 10, 10, "PORT6", PORT6);

  viewer->addPointCloud(Map3D, "Map3d", PORT1);
  viewer->addPointCloud(Map3DDense, "Map3dDense", PORT2);
  viewer->addPointCloud(cloud_3dMap_scaled, "Map3dScaled", PORT3);

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> trunk_color(trunk_segmented, 0,255, 0);
  viewer->addPointCloud(trunk_segmented,trunk_color, "Trunk", PORT6);
  viewer->addLine(minDBH,maxDBH,255,0,0,"DBH",PORT6);
  viewer->addLine(minTH,maxTH,255,0,0,"TH",PORT6);
  viewer->addLine(minDBH5,maxDBH5,255,0,0,"DBH5m",PORT6);

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tree_color(tree_segmented, 255, 255, 0);
  viewer->addPointCloud(tree_segmented,tree_color, "tree_segmented", PORT5);

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> crown_color(trunk_segmented, 255, 0, 255);
  viewer->addPointCloud(crown_segmented,crown_color,"crown", PORT4);
  viewer->addLine(minCH,maxCH,255,255,0,"CH",PORT4);

  viewer->setPosition(0,0);
  viewer->addCoordinateSystem ();
  pcl::PointXYZ p1, p2, p3;
  p1.getArray3fMap() << 1, 0, 0;
  p2.getArray3fMap() << 0, 1, 0;
  p3.getArray3fMap() << 0,0.1,1;

  viewer->addText3D("x", p1, 0.2, 1, 0, 0, "x_");
  viewer->addText3D("y", p2, 0.2, 0, 1, 0, "y_");
  viewer->addText3D ("z", p3, 0.2, 0, 0, 1, "z_");
  viewer->initCameraParameters();
  viewer->resetCamera();

  std::cout << "Press [q] to exit" << std::endl;

  while(!viewer->wasStopped ()) {
         viewer->spin();
  }

  return 0;
}



