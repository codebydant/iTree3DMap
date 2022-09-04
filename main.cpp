// #include "include/DendrometryE.h"
#include "Utilities.h"
// #include "visualizer.hpp"

int main(int argc, char **argv) {
  /* setup command line */
  Utilities::setup_argparse(argc, argv);

  /* starts the 3D mapping module */
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr Map3D(new pcl::PointCloud<pcl::PointXYZRGB>());
  bool success = Utilities::run_openMVG();
  if (not success) {
    PCL_ERROR("Could not get 3D Model. Failed dendrometric estimation.\n");
    return -1;
  }

  /*************************
  STEP 2: GET SCALE FACTOR
  **************************/
  double scale = 0;
  // success = Utilities::getScaleFactor(scale);
  // if(not success or scale <=0){
  //   PCL_WARN("Using scale factor = 120.128\n");
  //   scale = 127.128;
  // }
  scale = 127.128;

  /*************************
  STEP 3: DENSIFICATION
  **************************/
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr Map3DDense(new pcl::PointCloud<pcl::PointXYZRGB>());
  auto start = std::chrono::high_resolution_clock::now();
  success = Utilities::createPMVS_Files();
  bool projectFound = false;
  success = Utilities::densifyWithPMVS(Map3DDense, projectFound);
  if (not success or Map3DDense->points.empty()) {
    PCL_ERROR("Could not densify the points.");
    return -1;
  }
  auto end = std::chrono::high_resolution_clock::now();
  auto difference = std::chrono::duration_cast<std::chrono::seconds>(end - start).count();
  std::cout << "Dense map time: " << difference << " seconds" << std::endl;

  /*************************
  STEP 4: UNIFORM SCALING
  **************************/
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_3dMap_scaled(new pcl::PointCloud<pcl::PointXYZRGB>());
  Utilities::uniformScaling(Map3DDense, cloud_3dMap_scaled, projectFound, scale);

  /*************************
  STEP 5: SEGMENTATION
  **************************/
  // pcl::PointCloud<pcl::PointXYZ>::Ptr trunk_segmented (new pcl::PointCloud<pcl::PointXYZ>());
  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_aligned (new pcl::PointCloud<pcl::PointXYZRGB>());
  // pcl::PointCloud<pcl::PointXYZ>::Ptr tree_segmented (new pcl::PointCloud<pcl::PointXYZ>());
  // pcl::PointCloud<pcl::PointXYZ>::Ptr crown_segmented (new pcl::PointCloud<pcl::PointXYZ>());
  // start = std::chrono::high_resolution_clock::now();
  // Segmentation::extractTree(cloud_3dMap_scaled,output_dir,cloud_aligned,trunk_segmented,tree_segmented,crown_segmented);
  // end = std::chrono::high_resolution_clock::now();
  // difference = std::chrono::duration_cast<std::chrono::seconds>(end - start).count();
  // std::cout << "Segmentation time: " << difference << " seconds" << std::endl;
  // std::cout << std::endl;

  //   /*************************
  //   STEP 6: DENDROMETRY MEASUREMENTS
  //   **************************/
  //   pcl::PointXYZ minDBH,maxDBH,minTH,maxTH,minCH,maxCH,minDBH5,maxDBH5;
  //   pcl::PolygonMesh mesh1;
  //   pcl::PolygonMesh mesh2;
  //   Dendrometry::estimate(trunk_segmented,crown_segmented,output_dir,minDBH,maxDBH,minTH,maxTH,minCH,maxCH,minDBH5,maxDBH5,mesh1,mesh2);

  //   pcl::PointCloud<pcl::PointXYZ>::Ptr tree_cloud (new pcl::PointCloud<pcl::PointXYZ>());

  //   //Add trunk cloud
  //   for(pcl::PointCloud<pcl::PointXYZ>::iterator it=trunk_segmented->begin();it!=trunk_segmented->end(); ++it){
  //     pcl::PointXYZ pt = pcl::PointXYZ(it->x,it->y,it->z);
  //     tree_cloud->points.push_back(pt);
  //   }

  //   //Add crown cloud
  //   for(pcl::PointCloud<pcl::PointXYZ>::iterator it=crown_segmented->begin();it!=crown_segmented->end(); ++it){
  //     pcl::PointXYZ pt = pcl::PointXYZ(it->x,it->y,it->z);
  //     tree_cloud->points.push_back(pt);
  //   }

  //   tree_segmented->points.clear();
  //   pcl::copyPointCloud(*tree_cloud,*tree_segmented);

  //   std::map<double,pcl::PointXYZ> min_max_totalHeight;

  //   for(pcl::PointCloud<pcl::PointXYZ>::iterator it=tree_segmented->begin();it!=tree_segmented->end(); ++it){
  //     pcl::PointXYZ pt = pcl::PointXYZ(it->x,it->y,it->z);
  //     min_max_totalHeight[pt.y]=pt;
  //   }

  //   std::map<double,pcl::PointXYZ>::iterator it1 = min_max_totalHeight.begin();
  //   pcl::PointXYZ minTotalH = it1->second;

  //   std::map<double,pcl::PointXYZ>::iterator it2 = std::prev(min_max_totalHeight.end());
  //   pcl::PointXYZ maxTotalH = it2->second;

  //   maxTotalH.x = minTotalH.x;
  //   maxTotalH.z = minTotalH.z;

  /*************************
  PCL VISUALIZER
  **************************/
  // display_cloud(Map3D);
  //   boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("VISUALIZER"));

  //   Display* d = XOpenDisplay(NULL);
  //   Screen*  s = DefaultScreenOfDisplay(d);

  //   int y = s->height;
  //   int x = s->width;

  //   viewer->setSize(x,y);

  //   int PORT1 = 0;
  //   viewer->createViewPort(0.0, 0.5, 0.5, 1.0, PORT1);
  //   viewer->setBackgroundColor (0, 0, 0, PORT1);
  //   viewer->addText("ORIGINAL", 10, 10, "PORT1", PORT1);

  //   int PORT2 = 0;
  //   viewer->createViewPort(0.5, 0.5, 1.0, 1.0, PORT2);
  //   viewer->setBackgroundColor (0, 0, 0, PORT2);
  //   viewer->addText("TRUNK", 10, 10, "PORT2", PORT2);

  //   int PORT3 = 0;
  //   viewer->createViewPort(0.0, 0.0, 0.5, 0.5, PORT3);
  //   viewer->setBackgroundColor (0, 0, 0, PORT3);
  //   //viewer->addLine(ptt1,pt2,0,255,0 ,"lenght",PORT3);
  //   viewer->addText("VOLUME", 10, 10, "PORT3", PORT3);

  //   int PORT4 = 0;
  //   viewer->createViewPort(0.5, 0.0, 1.0, 0.5, PORT4);
  //   viewer->setBackgroundColor (0, 0, 0, PORT4);
  //   viewer->addText("CROWN", 10, 10, "PORT4", PORT4);
  // /*
  //   int PORT5 = 0;
  //   viewer->createViewPort(0.33, 0.0, 0.66, 0.5, PORT5);
  //   viewer->setBackgroundColor (0, 0, 0, PORT5);
  //   viewer->addText(, 10, 10, "PORT5", PORT5);

  //   int PORT6 = 0;
  //   viewer->createViewPort(0.66, 0.0, 1.0, 0.5, PORT6);
  //   viewer->setBackgroundColor (0, 0, 0, PORT6);
  //   viewer->addText(, 10, 10, "PORT6", PORT6);
  // */
  //   viewer->addPointCloud(Map3D, "Map3d", PORT1);
  //   //viewer->addPointCloud(Map3DDense, "Map3dDense", PORT2);
  //   // viewer->addPointCloud(cloud_3dMap_scaled, "Map3dScaled", PORT1);

  //   // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> trunk_color(trunk_segmented, 0,255, 0);
  //   // viewer->addPointCloud(trunk_segmented,trunk_color, "Trunk", PORT2);
  //   // viewer->addLine(minDBH,maxDBH,255,0,0,"DBH",PORT2);
  //   // viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,3,"DBH",PORT2);
  //   // viewer->addLine(minTH,maxTH,255,0,0,"TH",PORT2);
  //   // viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,3,"TH",PORT2);
  //   // viewer->addLine(minDBH5,maxDBH5,255,255,255,"DBH5m",PORT2);
  //   // viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,3,"DBH5m",PORT2);
  //  /*
  //   pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tree_color(tree_segmented, 255, 255, 0);
  //   viewer->addPointCloud(tree_segmented,tree_color, "tree_segmented", PORT4);
  //   viewer->addLine(minTotalH,maxTotalH,255,0,0,"TotalH",PORT4);
  //   viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,3,"TotalH",PORT4);
  //  */
  //   // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> crown_color(trunk_segmented, 255, 0, 255);
  //   // viewer->addPointCloud(crown_segmented,crown_color,"crown", PORT4);

  //   //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> mesh_color1(trunk_segmented, 255, 0, 0);
  //   // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> mesh_color2(trunk_segmented, 0, 255,0);
  //   // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_mesh (new pcl::PointCloud<pcl::PointXYZ>());
  //   // vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
  //   // pcl::io::mesh2vtk(mesh2,polydata);
  //   // pcl::io::vtkPolyDataToPointCloud(polydata,*cloud_mesh);
  //   // viewer->addPolygonMesh(mesh1,"mesh1", PORT3);
  //   // viewer->addPointCloud(cloud_mesh,mesh_color2,"mesh2", PORT3);

  //   pcl::PointXYZ p1, p2, p3;
  //   p1.getArray3fMap() << 100, 0, 0;
  //   p2.getArray3fMap() << 0, 100, 0;
  //   p3.getArray3fMap() << 0,0.1,100;

  //   viewer->addCoordinateSystem(100,"scale_ucs",PORT1);
  //   viewer->addText3D("x", p1, 15, 1, 0, 0, "x_",PORT1);
  //   viewer->addText3D("y", p2, 15, 0, 1, 0, "y_",PORT1);
  //   viewer->addText3D ("z", p3, 15, 0, 0, 1, "z_",PORT1);

  //   viewer->addCoordinateSystem(100,"trunk_ucs",PORT2);
  //   viewer->addText3D("x", p1, 15, 1, 0, 0, "x_",PORT2);
  //   viewer->addText3D("y", p2, 15, 0, 1, 0, "y_",PORT2);
  //   viewer->addText3D ("z", p3, 15, 0, 0, 1, "z_",PORT2);

  //   viewer->addCoordinateSystem(100,"crown_ucs",PORT4);
  //   viewer->addText3D("x", p1, 15, 1, 0, 0, "x_",PORT4);
  //   viewer->addText3D("y", p2, 15, 0, 1, 0, "y_",PORT4);
  //   viewer->addText3D ("z", p3, 15, 0, 0, 1, "z_",PORT4);

  //   viewer->addCoordinateSystem(100,"volume_ucs",PORT3);
  //   viewer->addText3D("x", p1, 15, 1, 0, 0, "x_",PORT3);
  //   viewer->addText3D("y", p2, 15, 0, 1, 0, "y_",PORT3);
  //   viewer->addText3D ("z", p3, 15, 0, 0, 1, "z_",PORT3);

  //   viewer->setPosition(0,0);
  //   viewer->addCoordinateSystem();

  //   viewer->initCameraParameters();
  //   viewer->resetCamera();

  //   std::cout << "Press [q] to exit" << std::endl;

  //   while(!viewer->wasStopped ()) {
  //          viewer->spin();
  //   }

  return 0;
}
