#include "include/Segmentation.h"

pcl::PointXYZ trunkMin;
pcl::PointXYZ trunkMax;

const std::string red("\033[0;31m");
const std::string blue("\033[0;34m");
const std::string yellow("\033[0;33m");
const std::string reset("\033[0m");
const std::string green("\033[0;32m");

std::string output_dir_path;

bool is_empty(std::ifstream& pFile){
    return pFile.peek() == std::ifstream::traits_type::eof();
}

bool Segmentation::extractTree(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
                               const std::string& output_path,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr& trunk_cloud,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr& tree_segmented,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr& crown_cloud_segmented){

  std::cout << "\n************************************************" << std::endl;
  std::cout << "              SEGMENTATION                      " << std::endl;
  std::cout << "************************************************" << std::endl;

  if(cloud->points.size() <= 0){
     PCL_ERROR("Input point cloud has no data!\n");
     return false;
  }

  cloud->header.frame_id = "principal_cloud";
  trunk_cloud->header.frame_id = "trunk_cloud";
  crown_cloud_segmented->header.frame_id = "crown_cloud";
  tree_segmented->header.frame_id = "tree_without_trunk";

  output_dir_path = output_path;

  /*CONVERT XYZRGB TO XYZ*/
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp (new pcl::PointCloud<pcl::PointXYZ>());
  pcl::copyPointCloud(*cloud,*temp);

  std::cout << "Applying:" << yellow << "StatisticalOutlierRemoval filter..." << reset << std::endl;
  std::cout << "Points before filter:" << yellow << temp->points.size() << reset<< std::endl;

  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(temp);
  sor.setMeanK(200);
  sor.setStddevMulThresh(1.0);
  sor.filter(*cloud_xyz);

  std::cout << "Points after filter:" << yellow << cloud_xyz->points.size() << reset<< std::endl;

  /*TRUNK SEGMENTATION*/
  std::cout << "Trunk cloud segmentation with:" << yellow << "Cylinder Model Segmentation..." << reset << "please wait..." << std::endl;

  bool trunkGood = false;
  std::string answer;
  bool setGui = false;

  Eigen::Matrix4f align_cloud;

  while(true){

      if(trunk_cloud->points.size()<=0){

       bool success = trunkSegmentation(cloud_xyz,tree_segmented,trunk_cloud,setGui,align_cloud);
       if(not success){
           trunkGood = false;
           trunk_cloud->points.clear();
           answer.clear();
           setGui= true;

           std::string callParametersGui = "/home/daniel/Documents/iTree3DMap/libraries/gui_Trunk_SegParam/build/interfaz ";
           callParametersGui += output_dir_path;
           callParametersGui += "/trunk_parameters.txt ";
           callParametersGui += output_dir_path;

           int dont_care = std::system(callParametersGui.c_str());
           if(dont_care > 0){
             std::cout << red << "Failed. interfaz not found" << reset << std::endl;
             return false;
           }
           continue;


         // pcl::PointXYZ pt1(0,0,0);
         // pcl::PointXYZ pt2(0,0,0);
         // Utilities::vtkVisualizer(trunk_cloud,pt1,pt2);
            }
      }

      if(trunk_cloud->points.size()>0){

        MultipleViewportsVTK(cloud_xyz,trunk_cloud);

        std::cout << blue << "\nTrunk segmentation Good?(yes/no)" << reset << std::endl;
        std::cout << "->" << std::flush;
        std::getline(std::cin, answer);
        if(answer.empty()){
          PCL_ERROR("Nothing entered.\n");
          answer.clear();
          continue;
        }
        if(answer == "yes"){
          trunkGood = true;
        }else if(answer == "no"){
          trunkGood = false;
          trunk_cloud->points.clear();          
          answer.clear();

          std::cout << "Trunk cloud re-segmenting..."<< yellow << "please wait" << reset << "..."<< std::endl;

          setGui = true;

          std::string callParametersGui = "/home/daniel/Documents/iTree3DMap/libraries/gui_Trunk_SegParam/build/interfaz ";
          callParametersGui += output_dir_path;
          callParametersGui += "/trunk_parameters.txt ";
          callParametersGui += output_dir_path;

          int dont_care = std::system(callParametersGui.c_str());
          if(dont_care > 0){
            std::cout << red << "Failed. interfaz not found" << reset << std::endl;
            return false;
          }
          continue;
        }else{
          PCL_ERROR("%s %s",answer.c_str(),"is not a valid answer.\n");
          trunkGood = false;
          answer.clear();
          continue;
        }
      }

      if(trunkGood){
        break;
      }
  }

  bool crownGood = false;
  bool setGui2 = false;
  std::string answer2;  

  std::string prefix = output_path;
  prefix += "/3D_Mapping/";

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

  std::cout << "prefix1:" << prefix1 << std::endl;

  std::cout << "Filtering points data with: PassThrough filter..." << std::endl;
  PCL_INFO("PointCloud before filtering has %lu %s", tree_segmented->points.size()," data points.\n");

  double meanMedian = pcl::geometry::distance(trunkMin,trunkMax);
  //PCL_INFO("\nMean median: %f",meanMedian);
  //std::cout << std::endl;
  //PCL_INFO("Mean median/2: %f",meanMedian/2);
  //std::cout << std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr crown_cloud (new pcl::PointCloud<pcl::PointXYZ>());

  pcl::PointXYZ pt1(0,0,0);
  pcl::PointXYZ pt2(0,0,0);
 // Utilities::vtkVisualizer(tree_segmented,pt1,pt2);

  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(tree_segmented);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(-900, meanMedian/2);
  pass.setFilterLimitsNegative(true);
  pass.filter(*crown_cloud);

  float min,max;
  pass.getFilterLimits(min,max);

  std::cout << "PointCloud after filtering has: " << crown_cloud->points.size ()
            << " data points." << std::endl;

  std::cout << "The limits are: min=" << min << ", max=" << max << std::endl;

 // Utilities::vtkVisualizer(crown_cloud,pt1,pt2);

  pcl::PLYWriter writer;
  writer.write(prefix2.c_str(), *trunk_cloud, false, false);
  writer.write(prefix4.c_str(), *tree_segmented, false, false);
  writer.write(prefix6.c_str(), *crown_cloud, false, false);

  pcl::io::savePCDFileBinary(prefix1.c_str(), *trunk_cloud);
  pcl::io::savePCDFileBinary(prefix3.c_str(), *tree_segmented);
  pcl::io::savePCDFileBinary(prefix5.c_str(), *crown_cloud);


  while(true){

      if(crown_cloud_segmented->points.size()<=0){

      bool success = crownSegmentation(crown_cloud,crown_cloud_segmented,setGui2);
      if(not success){
          crownGood = false;
          crown_cloud_segmented->points.clear();
          answer2.clear();
          setGui2= true;

          int dont_care;
          std::string callParametersGui = "/home/daniel/Documents/iTree3DMap/libraries/gui_Crown_SegParam/build/interfaz ";
          callParametersGui += output_dir_path;
          callParametersGui += "/crown_parameters.txt ";
          callParametersGui += output_dir_path;

          dont_care = std::system(callParametersGui.c_str());
          continue;

        }
         // pcl::PointXYZ pt1(0,0,0);
         // pcl::PointXYZ pt2(0,0,0);
         // Utilities::vtkVisualizer(crown_cloud,pt1,pt2);
      }

      if(crown_cloud_segmented->points.size()>0){

        MultipleViewportsVTK(cloud_xyz,crown_cloud_segmented);

        std::cout << blue <<  "\nCrown segmentation Good?(yes/no)" << reset << std::endl;
        std::cout << "->" << std::flush;
        std::getline(std::cin, answer2);
        if(answer.empty()){
          PCL_ERROR("Nothing entered.\n");
          answer2.clear();
          continue;
        }
        if(answer2 == "yes"){
          crownGood = true;
        }else if(answer2 == "no"){
          crownGood = false;
          crown_cloud_segmented->points.clear();
          answer2.clear();

          std::cout << "Crown cloud re-segmenting..."<< yellow << "please wait" << reset << "..."<< std::endl;

          setGui2= true;

          int dont_care;
          std::string callParametersGui = "/home/daniel/Documents/iTree3DMap/libraries/gui_Crown_SegParam/build/interfaz ";
          callParametersGui += output_dir_path;
          callParametersGui += "/crown_parameters.txt ";
          callParametersGui += output_dir_path;

          dont_care = std::system(callParametersGui.c_str());
          continue;
        }else{
          PCL_ERROR("%s %s",answer2.c_str(),"is not a valid answer.\n");
          crownGood = false;
          answer2.clear();
          continue;
        }
      }

      if(crownGood){
        break;
      }
  }

  //std::cout << "Crown cloud segmentation with:" << yellow <<  "Euclidean Cluster Extraction..." << reset << std::endl;

  pcl::console::TicToc tt2;
  PCL_INFO("\nSaving 3D mapping segmented...");

  pcl::PLYWriter writer2;
  writer2.write(prefix2.c_str(), *trunk_cloud, false, false);
  writer2.write(prefix4.c_str(), *tree_segmented, false, false);
  writer2.write(prefix6.c_str(), *crown_cloud_segmented, false, false);

  pcl::io::savePCDFileBinary(prefix1.c_str(), *trunk_cloud);
  pcl::io::savePCDFileBinary(prefix3.c_str(), *tree_segmented);
  pcl::io::savePCDFileBinary(prefix5.c_str(), *crown_cloud_segmented);

  pcl::console::print_info ("[done, ");
  pcl::console::print_value ("%g", tt2.toc ());
  pcl::console::print_info (" ms : ");
  pcl::console::print_value ("%d", trunk_cloud->points.size ());
  pcl::console::print_info (" points]\n");
/*
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temporal (new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::copyPointCloud(*cloud,*cloud_temporal);
  cloud->points.clear();
  pcl::transformPointCloud(*cloud_temporal, *cloud, align_cloud);
  */
/*
  for(int i=0;i<cloud->points.size();i++){

      cloud->points[i].x=cloud_xyz->points[i].x;
      cloud->points[i].y=cloud_xyz->points[i].y;
      cloud->points[i].z=cloud_xyz->points[i].z;
    }

*/
  std::cout << "Segmentation proccess --> [OK]" << std::endl;  

  return true;

}

bool Segmentation::trunkSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                     pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_without_trunk,
                                     pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_trunk,bool setGUI,
                                     Eigen::Matrix4f align_cloud,int K,double distanceWeight,
                                     int maxIterations,double distanceThreshold, double distanceWeight_cylinder,
                                     int maxIterations_cylinder,double distanceThreshold_cylinder,
                                     double minRadius, double maxRadius){

  if(cloud->points.size() <= 0){
     PCL_ERROR("Input point cloud has no data!\n");
     return false;
  }

  if(setGUI){

      std::string trunkParameters = output_dir_path;
      trunkParameters += "/trunk_parameters.txt";

      std::ifstream file(trunkParameters.c_str());

      while(file >> K >> distanceWeight >> maxIterations >> distanceThreshold
            >> distanceWeight_cylinder >> maxIterations_cylinder >> distanceThreshold_cylinder
            >> minRadius >> maxRadius){
      }

      file.close();
  }

  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
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
  pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients);
  pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);

  // Estimate point normals
  ne.setSearchMethod(tree);
  ne.setInputCloud(cloud);
  ne.setKSearch(K);//50
  ne.compute(*cloud_normals);

  //Create the segmentation object for the planar model and set all the parameters
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
  seg.setNormalDistanceWeight(distanceWeight);//0.1
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(maxIterations);//500
  seg.setDistanceThreshold(distanceThreshold);//0.5
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
  seg.setNormalDistanceWeight(distanceWeight_cylinder);//0.5
  seg.setMaxIterations(maxIterations_cylinder);//10000
  seg.setDistanceThreshold(distanceThreshold_cylinder);//8
  seg.setRadiusLimits(minRadius,maxRadius);//(0,60)
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

  std::string prefix = output_dir_path;
  prefix += "/3D_Mapping/";

  std::string prefix1 = prefix;
  prefix1 += "MAP3D_trunk_segmented.pcd";

  pcl::io::savePCDFileBinary(prefix1.c_str(), *trunk_seg_filtered);  
  
  std::map<double,pcl::PointXYZ> min_max_HeightTrunk;

  C_Progress_display my_progress_bar( trunk_seg_filtered->size(),std::cout, "\n- Getting trunk height -\n" );
  for(pcl::PointCloud<pcl::PointXYZ>::iterator it = trunk_seg_filtered->begin();
    it != trunk_seg_filtered->end();++it, ++my_progress_bar){

      pcl::PointXYZ pt = pcl::PointXYZ(it->x,it->y,it->z);
      min_max_HeightTrunk[pt.y] = pt;

    }

/*
  for(pcl::PointCloud<pcl::PointXYZ>::iterator it=trunk_seg_filtered->begin();it!=trunk_seg_filtered->end(); ++it){
    pcl::PointXYZ pt = pcl::PointXYZ(it->x,it->y,it->z);
    min_max_HeightTrunk[pt.y] = pt;
  } 
*/
  std::map<double,pcl::PointXYZ>::iterator it1_minH = min_max_HeightTrunk.begin();
  pcl::PointXYZ minDBH_inX = it1_minH->second;
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr trunk_seg_filtered_DBScan (new pcl::PointCloud<pcl::PointXYZ>());   
  DBScan(trunk_seg_filtered_DBScan,minDBH_inX);

  align_cloud = Eigen::Matrix4f::Identity();

  pcl::PointXYZ minH,maxH;
  std::map<double,pcl::PointXYZ> minMaxValues;

  for(pcl::PointCloud<pcl::PointXYZ>::iterator it=trunk_seg_filtered_DBScan->begin(); it!=trunk_seg_filtered_DBScan->end(); ++it){
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

  pcl::PointCloud<pcl::PointXYZ>::Ptr temp (new pcl::PointCloud<pcl::PointXYZ>());

  pcl::copyPointCloud(*cloud,*temp);
  cloud->points.clear();
  pcl::transformPointCloud(*temp, *cloud, align_cloud);
  pcl::transformPointCloud(*trunk_seg_filtered_DBScan, *cloud_trunk, align_cloud);
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

  std::string trunkParameters = output_dir_path;
  trunkParameters += "/trunk_parameters.txt";

  std::ofstream ofs(trunkParameters.c_str());
  ofs << K << std::endl

      << distanceWeight << std::endl
      << maxIterations << std::endl
      << distanceThreshold << std::endl

      << distanceWeight_cylinder << std::endl
      << maxIterations_cylinder << std::endl
      << distanceThreshold_cylinder << std::endl
      << minRadius << std::endl
      << maxRadius << std::endl;

  ofs.close();

  return true;

}

bool Segmentation::crownSegmentation(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_crown,
                                     pcl::PointCloud<pcl::PointXYZ>::Ptr& crown_better_segmented,bool setGUI,int octreeResolution, double eps,int minPtsAux,int minPts){

  if(cloud_crown->points.size() <= 0){
    PCL_ERROR("Input point cloud has no data!");
    return false;
  }

  if(setGUI){

      std::string crownParameters = output_dir_path;
      crownParameters += "/crown_parameters.txt";

      std::ifstream file(crownParameters.c_str());

      while(file >> octreeResolution >> eps >> minPtsAux >> minPts){
        }

      file.close();
  }

  std::string command = "/home/daniel/Documents/iTree3DMap/libraries/DBScan_Octrees-master/build/bin/dbscan ";
  command += output_dir_path;
  command += "/3D_Mapping/MAP3D_crown_segmented.pcd ";
  command += std::to_string(octreeResolution);
  command += " ";
  command += std::to_string(eps);
  command += " ";
  command += std::to_string(minPtsAux);
  command += " ";
  command += std::to_string(minPts);
  command += " ";
  command += output_dir_path;

  std::cout << "\nApplying DBSCAN..." << std::endl;
  int dont_care = -1;
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds_vector;
  //if(cloud_crown->points.size()>10000){
      dont_care = std::system(command.c_str());

      if(dont_care > 0){
        std::cout << red << "Failed. dbscan not found" << reset << std::endl;
        return false;
      }

      std::string numCluster = output_dir_path;
      numCluster += "/clusters_number.txt";
      std::ifstream file(numCluster.c_str());
      if(!file.is_open()){
          std::cout << "Error: Could not find " << numCluster << std::endl;
          return false;
      }

      if(is_empty(file)){
          PCL_WARN("Could not generated cluster. skip!\n");
          return false;
      }

          unsigned int totalClusters;

           while(file >> totalClusters){
            //std::cout << "getting total clusters:" << totalClusters << std::endl;
          }

           if(totalClusters<=0){
               PCL_WARN("Could not generated cluster. skip!\n");
               return false;
             }
      file.close();
      for(int i=0;i<totalClusters; i++){

          std::string feature_path=output_dir_path;
          feature_path +="/cloud_cluster_";
          feature_path += std::to_string(i);
          feature_path += ".xyz";
         // std::cout << "Using file:" << feature_path << std::endl;

          float x_, y_,z_;

          std::ifstream file(feature_path.c_str());
          if(!file.is_open() ){
            std::cout << "Error: Could not find " << feature_path << std::endl;
            feature_path.clear();
            return false;

          }

          pcl::PointCloud<pcl::PointXYZ>::Ptr crown_cloud_segmented (new pcl::PointCloud<pcl::PointXYZ>());

          while(file >> x_ >> y_ >> z_){
            pcl::PointXYZ pt = pcl::PointXYZ(x_,y_,z_);
              crown_cloud_segmented->points.push_back(pt);
          }

          crown_cloud_segmented->width = crown_cloud_segmented->points.size();
          crown_cloud_segmented->height = 1;
          crown_cloud_segmented->is_dense = true;

          clouds_vector.push_back(crown_cloud_segmented);

      }
      //std::cout << "Crown cloud segmented with DBScan:" << clouds_vector.size() << " clusters" << std::endl;
  //}

  std::map<double,pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters_dbscan_map;

 // if(cloud_crown->points.size()>10000){
    for(int i=0;i<clouds_vector.size(); i++){

        Eigen::Vector4f centroid;

        pcl::compute3DCentroid(*clouds_vector.at(i),centroid);
        pcl::PointXYZ pt2 = pcl::PointXYZ(centroid[0],centroid[1],centroid[2]);
        //std::cout << "centroid:" << pt2 << std::endl;

        double error = pcl::geometry::distance(trunkMax,pt2);
        //std::cout << "Error distance centroid trunk-crown:" << error << std::endl;
        clusters_dbscan_map[error]= clouds_vector.at(i);
    }

    std::map<double,pcl::PointCloud<pcl::PointXYZ>::Ptr>::iterator crown_dbscan_segmented = clusters_dbscan_map.begin();
    crown_better_segmented->points.clear();
    pcl::copyPointCloud(*crown_dbscan_segmented->second,*crown_better_segmented);
 //  }

  std::cout << "Crown DBScan segmented:" << crown_better_segmented->points.size() << std::endl;

  std::string guardar= output_dir_path;
  guardar += "/3D_Mapping/cloud_crown_segmented_DBScan.pcd";

  pcl::io::savePCDFileBinary(guardar.c_str(),*crown_better_segmented);

  std::string crownParameters = output_dir_path;
  crownParameters += "/crown_parameters.txt";

  std::ofstream ofs(crownParameters.c_str());

  ofs << octreeResolution << std::endl
      << eps << std::endl
      << minPtsAux << std::endl
      << minPts << std::endl;
  ofs.close();

/*
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
    */

  return true;

}

bool Segmentation::DBScan(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_segmented,pcl::PointXYZ& minTrunkHeight){

  std::string command = "/home/daniel/Documents/iTree3DMap/libraries/DBScan_Octrees-master/build/bin/dbscan ";
  command += output_dir_path;
  command += "/3D_Mapping/MAP3D_trunk_segmented.pcd 3 40 5 10 ";//40 10 1000
  command += output_dir_path;
  
  std::cout << "\nApplying DBSCAN..." << std::endl;

  std::cout << "command:" << command << std::endl;
  int dont_care = std::system(command.c_str());

  if(dont_care > 0){
    std::cout << red << "Failed. dbscan not found" << reset << std::endl;
    return false;
  } 
  
  std::string numCluster = output_dir_path;
  numCluster += "/clusters_number.txt";
  std::ifstream file(numCluster.c_str());
  if(!file.is_open()){
      std::cout << "Error: Could not find " << numCluster << std::endl;
      return false;
  }

  if(is_empty(file)){
      PCL_WARN("Could not generated cluster. skip!\n");
      return false;
  }

  unsigned int totalClusters;

  while(file >> totalClusters){
      //std::cout << "getting total clusters:" << totalClusters << std::endl;
    }

  if(totalClusters<=0){
      PCL_WARN("Could not generated cluster. skip!\n");
      return false;
  }

  file.close();
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>());
  
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds_vector;
  
  for(int i=0;i<totalClusters; i++){

      std::string feature_path = output_dir_path;
      feature_path +="/cloud_cluster_";
      feature_path += std::to_string(i);
      feature_path += ".xyz";  
      //std::cout << "Using file:" << feature_path << std::endl;

      float x_, y_,z_;

      std::ifstream file(feature_path.c_str());
      if(!file.is_open() ){
        std::cout << "Error: Could not find " << feature_path << std::endl;
        feature_path.clear();
        return false;
      }

      if(is_empty(file)){
          PCL_WARN("Could not generated cluster. skip!\n");
          return false;
      }

      while(file >> x_ >> y_ >> z_){
        pcl::PointXYZ pt = pcl::PointXYZ(x_,y_,z_);
        cloud_cluster->points.push_back(pt);
      }

      cloud_cluster->width = cloud_cluster->points.size();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;
      /*
      std::map<double,pcl::PointXYZ> min_max_Height;

      for(pcl::PointCloud<pcl::PointXYZ>::iterator it=cloud_cluster->begin();it!=cloud_cluster->end(); ++it){
        pcl::PointXYZ pt = pcl::PointXYZ(it->x,it->y,it->z);
        min_max_Height[pt.y]=pt;
      }

      std::map<double,pcl::PointXYZ>::iterator it3 = min_max_Height.begin();
      pcl::PointXYZ minH = it3->second;
      
      std::map<double,pcl::PointXYZ>::iterator it4 = std::prev(min_max_Height.end());
      pcl::PointXYZ maxH = it4->second; 
      
      //if(minH.y != minTrunkHeight.y)continue;
      */
      clouds_vector.push_back(cloud_cluster);  
  }
  
  std::cout << "Crown cloud segmented with DBScan:" << clouds_vector.size() << " clusters" << std::endl;
  pcl::copyPointCloud(*clouds_vector.at(0),*cloud_segmented);
  /*
  std::map<double,pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters_dbscan_map;
  
  for(int i=0;i<clouds_vector.size(); i++){   
  
      Eigen::Vector4f centroid;
     
      pcl::compute3DCentroid(*clouds_vector.at(i),centroid);
      pcl::PointXYZ pt2 = pcl::PointXYZ(centroid[0],centroid[1],centroid[2]);
      //std::cout << "centroid:" << pt2 << std::endl;
            
      double error = pcl::geometry::distance(maxTH,pt2);
      //std::cout << "Error distance centroid trunk-crown:" << error << std::endl;
      clusters_dbscan_map[error]= clouds_vector.at(i);  
  
  }

*/

}

void Segmentation::MultipleViewportsVTK(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud1,
                                        pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud2){

  if(cloud1->points.size()<=0 or cloud2->points.size()<=0){
      PCL_ERROR("Input point cloud has no data!");
      return std::exit(-1);
    }

  vtkSmartPointer<vtkPolyData> cloud1VTK = vtkSmartPointer<vtkPolyData>::New();
  vtkSmartPointer<vtkPolyData> cloud2VTK = vtkSmartPointer<vtkPolyData>::New();

  vtkSmartPointer<vtkPoints> pts1 = vtkSmartPointer<vtkPoints>::New();

  for(int n=0;n<cloud1->points.size();n++){
      pcl::PointXYZ pt = cloud1->points.at(n);
      pts1->InsertNextPoint(pt.x,pt.y,pt.z);
    }

  cloud1VTK->SetPoints(pts1);

  vtkSmartPointer<vtkPoints> pts2 = vtkSmartPointer<vtkPoints>::New();

  for(int n=0;n<cloud2->points.size();n++){
      pcl::PointXYZ pt = cloud2->points.at(n);
      pts2->InsertNextPoint(pt.x,pt.y,pt.z);
    }

  cloud2VTK->SetPoints(pts2);

  vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();

  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
      vtkSmartPointer<vtkRenderWindowInteractor>::New();

  renderWindowInteractor->SetRenderWindow(renderWindow);

  // Define viewport ranges
   double xmins[2] = {0.0,0.5};
   double xmaxs[2] = {0.5,1.0};
   double ymins[2] = {0.0,0.0};
   double ymaxs[2] = {1.0,1.0};

   vtkCamera* camera;

   for(unsigned i = 0; i < 2; i++){

       vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();

       renderWindow->AddRenderer(renderer);

       if(i == 0){
           camera = renderer->GetActiveCamera();
       }else{
           renderer->SetActiveCamera(camera);
       }

       renderer->SetViewport(xmins[i],ymins[i],xmaxs[i],ymaxs[i]);

       if(i==0){

           vtkSmartPointer<vtkVertexGlyphFilter> vertexFilter1 = vtkSmartPointer<vtkVertexGlyphFilter>::New();
           vertexFilter1->SetInputData(cloud1VTK);
           vertexFilter1->Update();

           vtkSmartPointer<vtkPolyData> polydata1 = vtkSmartPointer<vtkPolyData>::New();
           polydata1->ShallowCopy(vertexFilter1->GetOutput());

           // Create a mapper and actor
           vtkSmartPointer<vtkPolyDataMapper> mapper1 = vtkSmartPointer<vtkPolyDataMapper>::New();
           mapper1->SetInputData(polydata1);

           vtkSmartPointer<vtkActor> actor1 = vtkSmartPointer<vtkActor>::New();
           actor1->SetMapper(mapper1);
           actor1->GetProperty()->SetColor(1.0, 1.0, 1.0);
           actor1->GetProperty()->SetPointSize(1);
           renderer->AddActor(actor1);

           vtkSmartPointer<vtkTextActor> textActor = vtkSmartPointer<vtkTextActor>::New();
           textActor->SetInput("Original");
           textActor->SetPosition2(20,60);
           textActor->GetTextProperty()->SetFontSize(12);
           textActor->GetTextProperty()->SetColor(1.0, 1.0, 1.0);
           renderer->AddActor2D(textActor);
       }else

       //---------------------------------------
       //---------------------------------------
       {

           vtkSmartPointer<vtkVertexGlyphFilter> vertexFilter2 = vtkSmartPointer<vtkVertexGlyphFilter>::New();
           vertexFilter2->SetInputData(cloud2VTK);
           vertexFilter2->Update();

           vtkSmartPointer<vtkPolyData> polydata2 = vtkSmartPointer<vtkPolyData>::New();
           polydata2->ShallowCopy(vertexFilter2->GetOutput());

           // Create a mapper and actor
           vtkSmartPointer<vtkPolyDataMapper> mapper2 = vtkSmartPointer<vtkPolyDataMapper>::New();
           mapper2->SetInputData(polydata2);

           vtkSmartPointer<vtkActor> actor2 = vtkSmartPointer<vtkActor>::New();
           actor2->SetMapper(mapper2);
           actor2->GetProperty()->SetColor(0.0, 1.0, 0.0);
           actor2->GetProperty()->SetPointSize(1);
           renderer->AddActor(actor2);

           vtkSmartPointer<vtkTextActor> textActor = vtkSmartPointer<vtkTextActor>::New();
           textActor->SetInput("Segmented");
           textActor->SetPosition2(20,60);
           textActor->GetTextProperty()->SetFontSize(12);
           textActor->GetTextProperty()->SetColor(1.0, 1.0, 1.0);
           renderer->AddActor2D(textActor);
       }

       // Create a renderer, render window, and interactor

       renderer->SetBackground(0.0, 0.0, 0.0);
       renderer->ResetCamera();

       vtkSmartPointer<vtkAxesActor> axes = vtkSmartPointer<vtkAxesActor>::New();

       renderWindow->SetSize(800, 600);
       renderWindow->AddRenderer(renderer);

       renderWindowInteractor->SetRenderWindow(renderWindow);

       //renderer->AddActor(axes);
       renderer->ResetCamera();
       renderer->SetViewPoint(0,0,0);

       renderWindow->Render();
       renderWindow->SetWindowName("Segmentation");

       vtkSmartPointer<vtkInteractorStyleTrackballCamera> style = vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
       renderWindowInteractor->SetInteractorStyle(style);

     }

   PCL_INFO("\nPress [q] to close visualizer");
   renderWindowInteractor->Start();


}










