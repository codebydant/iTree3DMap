/*********************************
           HEADERS
**********************************/
#include "include/DendrometryE.h"
#include <X11/Xlib.h>

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
  //Utilities::run_openMVG();

  /*************************
  STEP 3: GET SCALE FACTOR
  **************************/
  float scale; 
  pcl::PointCloud<pcl::PointXYZ>::Ptr Map3D (new pcl::PointCloud<pcl::PointXYZ>());
  pcl::io::loadPCDFile("MAP3D_dense.pcd",*Map3D);
  //Utilities::getScaleFactor(Map3D,scale);

  std::cout << "Map3D points:" << Map3D->points.size() << std::endl;

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
  viewer->createViewPort(0.0, 0.5, 0.5, 1.0, PORT1);
  viewer->setBackgroundColor (0, 0, 0, PORT1);
  viewer->addText("MAPPING", 10, 10, "PORT1", PORT1);

  int PORT2 = 0;
  viewer->createViewPort(0.5, 0.5, 1.0, 1.0, PORT2);
  viewer->setBackgroundColor (0.3, 0.3, 0.3, PORT2);
  viewer->addText("DENSE MAPPING", 10, 10, "PORT2", PORT2);

  int PORT3 = 0;
  viewer->createViewPort(0.0, 0.0, 0.5, 0.5, PORT3);
  viewer->setBackgroundColor (0.2, 0.2, 0.2, PORT3);
  viewer->addText("SCALE MAPPING", 10, 10, "PORT3", PORT3);

  int PORT4 = 0;
  viewer->createViewPort(0.5, 0.0, 1.0, 0.5, PORT4);
  viewer->setBackgroundColor (0.1, 0.1, 0.1, PORT4);
  viewer->addText("TRUNK AND CROWN", 10, 10, "PORT4", PORT4);

  viewer->addPointCloud<pcl::PointXYZ>(Map3D, "sample cloud1", PORT1);

  /*
  HWND hWnd = (HWND)viewer3d->getRenderWindow()->GetGenericWindowId();
  delete viewer3d;
  DestroyWindow(hWnd);
  */
  /*************************
  STEP 4: DENSIFICATION
  **************************/
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr Map3DDense (new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::io::loadPCDFile("MAP3D_dense.pcd",*Map3DDense);
 //
 // pcl::copyPointCloud(*cloud_3dMap,*cloud_3dMap_xyz);
  /*
   * auto start = std::chrono::high_resolution_clock::now();
  Utilities::createPMVS_Files();
  Utilities::densifyWithPMVS(cloud_3dMap);
  end = std::chrono::high_resolution_clock::now();
  difference = std::chrono::duration_cast<std::chrono::seconds>(end - start).count();
  if(difference >=60){
    std::cout << "Dense Map Time: " << difference/60 << " minutes" << std::endl;
  }else{
    std::cout << "Dense Map Time: " << difference << " seconds" << std::endl;
  }
 */
  viewer->addPointCloud<pcl::PointXYZRGB>(Map3DDense, "sample cloud2", PORT2);
  viewer->addPointCloud<pcl::PointXYZRGB>(Map3DDense, "sample cloud3", PORT3);
  viewer->addPointCloud<pcl::PointXYZRGB>(Map3DDense, "sample cloud4", PORT4);


  /*************************
  STEP 5: UNIFORM SCALING
  **************************/
  /*
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_3dMap_scaled (new pcl::PointCloud<pcl::PointXYZRGB>());
  Utilities::uniformScaling(tree_segmented,cloud_3dMap_scaled,scale,true);
  */
  /*************************
  STEP 6: SEGMENTATION
  **************************/  
  /*
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr tree_segmented (new pcl::PointCloud<pcl::PointXYZRGB>());
  auto start = std::chrono::high_resolution_clock::now();
  Segmentation::extractTree(cloud_3dMap,tree_segmented);
  auto end = std::chrono::high_resolution_clock::now();
  auto difference = std::chrono::duration_cast<std::chrono::seconds>(end - start).count();
  if(difference >=60){
    std::cout << "Segmentation Time: " << difference/60 << " minutes" << std::endl;
  }else{
    std::cout << "Segmentation Time: " << difference << " seconds" << std::endl;
  }
  */

  /*************************
  STEP 7: DENDROMETRY MEASUREMENTS
  **************************/
  //Dendrometry::estimate(cloud_3dMap_scaled);

  viewer->setPosition(0,0);
  viewer->addCoordinateSystem();
  //viewer.setCameraPosition(0,0,1,0,0,0);
  pcl::PointXYZ p1, p2, p3;
  p1.getArray3fMap() << 1, 0, 0;
  p2.getArray3fMap() << 0, 1, 0;
  p3.getArray3fMap() << 0,0.1,1;

  viewer->addText3D("x", p1, 0.2, 1, 0, 0, "x_");
  viewer->addText3D("y", p2, 0.2, 0, 1, 0, "y_");
  viewer->addText3D ("z", p3, 0.2, 0, 0, 1, "z_");

  viewer->initCameraParameters();
  viewer->resetCamera();

  std::cout << "Press [q] to continue!" << std::endl;

  while(!viewer->wasStopped())
  {
    viewer->spin();
    boost::this_thread::sleep(boost::posix_time::microseconds(100000));
  }
  return 0;
}



