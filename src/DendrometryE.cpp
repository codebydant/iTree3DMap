#include "include/DendrometryE.h"

const std::string red("\033[0;31m");
const std::string blue("\033[0;34m");
const std::string yellow("\033[0;33m");
const std::string reset("\033[0m");
const std::string green("\033[0;32m");

static float signedVolumeOfTriangle(pcl::PointXYZ p1, pcl::PointXYZ p2, pcl::PointXYZ p3) 
{
    float v321 = p3.x*p2.y*p1.z;
    float v231 = p2.x*p3.y*p1.z;
    float v312 = p3.x*p1.y*p2.z;
    float v132 = p1.x*p3.y*p2.z;
    float v213 = p2.x*p1.y*p3.z;
    float v123 = p1.x*p2.y*p3.z;
    return (1.0f/6.0f)*(-v321 + v231 + v312 - v132 - v213 + v123);
}

static float volumeOfMesh(pcl::PolygonMesh mesh){
    float vols = 0.0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>());
     vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
    pcl::io::mesh2vtk(mesh,polydata);
    pcl::io::vtkPolyDataToPointCloud(polydata,*cloud);
    

    for(int triangle=0;triangle<mesh.polygons.size();triangle++)
    {
        pcl::PointXYZ pt1 = cloud->points[mesh.polygons[triangle].vertices[0]];
        pcl::PointXYZ pt2 = cloud->points[mesh.polygons[triangle].vertices[1]];
        pcl::PointXYZ pt3 = cloud->points[mesh.polygons[triangle].vertices[2]];
        vols += signedVolumeOfTriangle(pt1, pt2, pt3);
    }

    return std::abs(vols);
}

void fromPointsToPCLCloud(const std::vector<pcl::PointXYZ>& input_cloud,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudPCL){

  for(size_t i = 0; i < input_cloud.size(); ++i){
    pcl::PointXYZ temp = input_cloud.at(i);
    pcl::PointXYZ pt;
    pt.x = temp.x;
    pt.y = temp.y;
    pt.z = temp.z;
    cloudPCL->push_back(pt);
   }
   cloudPCL->width = (uint32_t) cloudPCL->points.size(); // number of points
   cloudPCL->height = 1;	// a list, one row of data
   cloudPCL->is_dense = false;
}

void Dendrometry::estimate(const pcl::PointCloud<pcl::PointXYZ>::Ptr& trunk_cloud,
                           const pcl::PointCloud<pcl::PointXYZ>::Ptr& crown_cloud,
                           const std::string output_dir,
                           pcl::PointXYZ& minDBH,pcl::PointXYZ& maxDBH,pcl::PointXYZ& minTH,pcl::PointXYZ& maxTH,
                           pcl::PointXYZ& minCH,pcl::PointXYZ& maxCH,pcl::PointXYZ& minDBH5,pcl::PointXYZ& maxDBH5
                           ){

  std::cout << "************************************************" << std::endl;
  std::cout << "              DENDROMETRY ESTIMATION            " << std::endl;
  std::cout << "************************************************" << std::endl;

  std::string command = "../../libraries/DBScan_Octrees-master/src/build/bin/dbscan ";
  command += output_dir;
  command += "/3D_Mapping/MAP3D_crown_segmented.pcd 40 10 1000 ";
  command += output_dir;
  
  std::cout << "Applying DBSCAN..." << std::endl;

  int dont_care = std::system(command.c_str());

  if(dont_care > 0){
    std::cout << red << "Failed. dbscan not found" << reset << std::endl;
    return std::exit(-1);
  } 
  
  std::string numCluster = output_dir;
  numCluster += "/clusters_number.txt";
  std::ifstream file(numCluster.c_str());
      if(!file.is_open()){
        std::cout << "Error: Could not find " << numCluster << std::endl;        
        return exit(-1);

      }
      
      unsigned int totalClusters;
      
       while(file >> totalClusters){
        std::cout << "getting total clusters:" << totalClusters << std::endl;
      }
  file.close();  
  
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds_vector;
  
  for(int i=0;i<totalClusters; i++){
  
      std::string feature_path=output_dir;
      feature_path +="/cloud_cluster_";
      feature_path += std::to_string(i);
      feature_path += ".xyz";  
      std::cout << "Using file:" << feature_path << std::endl;

      float x_, y_,z_;

      std::ifstream file(feature_path.c_str());
      if(!file.is_open() ){
        std::cout << "Error: Could not find " << feature_path << std::endl;
        feature_path.clear();
        return exit(-1);

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
  
  std::cout << "Crown cloud segmented with DBScan:" << clouds_vector.size() << " clusters" << std::endl;
  std::map<double,pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters_dbscan_map;

  std::string dendrometric_results = output_dir;
  dendrometric_results += "/dendrometric.txt";

  ofstream feature(dendrometric_results.c_str());

  double height_trunk,DBH,height_crown,total_height,factor_morfico,crown_volume;

  std::cout << blue << "Estimating trunk features..." << reset << std::endl;
  std::cout << "------------------------------------------" << std::endl;

  std::cout << yellow << "\nDBH." << reset << std::endl;
  std::cout << "------------------------------------------" << std::endl;

  std::vector<pcl::PointXYZ> pts_for_DBH;

  for(pcl::PointCloud<pcl::PointXYZ>::iterator it=trunk_cloud->begin();it!=trunk_cloud->end(); ++it){

    pcl::PointXYZ pt = pcl::PointXYZ(it->x,it->y,it->z);
    if(pt.y>120 and pt.y<140){
      pts_for_DBH.push_back(pt);
    }else{
      continue;
    }
  }

  if(pts_for_DBH.size()<=0){
    PCL_ERROR("\nNo points at 1.33m. Using random points!");
    pts_for_DBH.push_back(pcl::PointXYZ(-22.4977,161.408,128.112));
    pts_for_DBH.push_back(pcl::PointXYZ(21.1596,145.483,152.903));
  }

  PCL_INFO("\nPoints between 1.33+/-0.5cm: %lu", pts_for_DBH.size());

  feature << "ANALYSIS" << std::endl;
  feature << "Points between 1.33m +/- 0.5cm:" << pts_for_DBH.size() << std::endl;

  std::map<double,pcl::PointXYZ> min_max_DBH_inX;

  for(std::vector<pcl::PointXYZ>::iterator it=pts_for_DBH.begin();it!=pts_for_DBH.end(); ++it){
    pcl::PointXYZ pt = pcl::PointXYZ(it->x,it->y,it->z);
    min_max_DBH_inX[pt.x] = pt;
  }
  
  std::map<double,pcl::PointXYZ> min_max_DBH_inZ;

  for(std::vector<pcl::PointXYZ>::iterator it=pts_for_DBH.begin();it!=pts_for_DBH.end(); ++it){
    pcl::PointXYZ pt = pcl::PointXYZ(it->x,it->y,it->z);
    min_max_DBH_inZ[pt.z] = pt;
  } 

  std::map<double,pcl::PointXYZ>::iterator it1 = min_max_DBH_inX.begin();
  pcl::PointXYZ minDBH_inX = it1->second;

  std::map<double,pcl::PointXYZ>::iterator it2 = std::prev(min_max_DBH_inX.end());
  pcl::PointXYZ maxDBH_inX = it2->second;

  std::cout << "\nMinDBH in X:" << minDBH_inX.x << std::endl;
  std::cout << "MaxDBH in X:" << maxDBH_inX.x << std::endl;
  
  std::cout << "MinDBH pt3d in X:" << minDBH_inX << std::endl;
  std::cout << "MaxDBH pt3d in X:" << maxDBH_inX << std::endl;

  feature << "MinDBH pt3d in X:" << minDBH_inX << std::endl;
  feature << "MaxDBH pt3d in X:" << maxDBH_inX << std::endl;

  minDBH_inX.y = maxDBH_inX.y;
  minDBH_inX.z = maxDBH_inX.z;

  double DBH_inX = pcl::geometry::distance(minDBH_inX,maxDBH_inX);
  std::cout << "DBH in X:" << DBH_inX << std::endl;
  
  //------------------------------------------
  //------------------------------------------
  
  std::map<double,pcl::PointXYZ>::iterator it3 = min_max_DBH_inZ.begin();
  pcl::PointXYZ minDBH_inZ = it3->second;

  std::map<double,pcl::PointXYZ>::iterator it4 = std::prev(min_max_DBH_inZ.end());
  pcl::PointXYZ maxDBH_inZ = it4->second;

  std::cout << "MinDBH in Z:" << minDBH_inZ.z << std::endl;
  std::cout << "MaxDBH in Z:" << maxDBH_inZ.z << std::endl;

  feature << "MinDBH pt3d in Z:" << minDBH_inZ << std::endl;
  feature << "MaxDBH pt3d in Z:" << maxDBH_inZ << std::endl;
  
  std::cout << "MinDBH pt3d in Z:" << minDBH_inZ << std::endl;
  std::cout << "MaxDBH pt3d in Z:" << maxDBH_inZ << std::endl;

  minDBH_inZ.y = maxDBH_inZ.y;
  minDBH_inZ.x = maxDBH_inZ.x;

  double DBH_inZ = pcl::geometry::distance(minDBH_inZ,maxDBH_inZ);
  std::cout << "DBH in Z:" << DBH_inZ << std::endl;
  
  if(DBH_inX < DBH_inZ){
    minDBH = minDBH_inX;
    maxDBH = maxDBH_inX;   
    DBH = DBH_inX;
  }else{  
    minDBH = minDBH_inZ;
    maxDBH = maxDBH_inZ;  
    DBH = DBH_inZ;    
  }
  
  std::cout << "MinDBH pt3d:" << minDBH << std::endl;
  std::cout << "MaxDBH pt3d:" << maxDBH << std::endl;
  
  //------------------------------------------
  //------------------------------------------
  

  std::cout << yellow << "\nHeight." << reset << std::endl;
  std::cout << "------------------------------------------" << std::endl;

  std::map<double,pcl::PointXYZ> min_max_trunkHeight;

  for(pcl::PointCloud<pcl::PointXYZ>::iterator it=trunk_cloud->begin();it!=trunk_cloud->end(); ++it){
    pcl::PointXYZ pt = pcl::PointXYZ(it->x,it->y,it->z);
    min_max_trunkHeight[pt.y]=pt;
  }

  it1 = min_max_trunkHeight.begin();
  minTH = it1->second;

  it2 = std::prev(min_max_trunkHeight.end());
  maxTH = it2->second;

  feature << "minTH pt3d:" << minTH << std::endl;
  feature << "maxTH pt3d:" << maxTH << std::endl;

  maxTH.x = minTH.x;
  maxTH.z = minTH.z;

  //minTH.x = maxTH.x;
  //minTH.z = maxTH.z;

  std::cout << "MaxTH: " << maxTH << std::endl;
  std::cout << "MinTH: " << minTH << std::endl;

  height_trunk = pcl::geometry::distance(minTH,maxTH);
  
  
  //-------------------------------------------------
  //-------------------------------------------------  
  
  for(int i=0;i<clouds_vector.size(); i++){   
  
      Eigen::Vector4f centroid;
     
      pcl::compute3DCentroid(*clouds_vector.at(i),centroid);
      pcl::PointXYZ pt2 = pcl::PointXYZ(centroid[0],centroid[1],centroid[2]);
      std::cout << "centroid:" << pt2 << std::endl;
      
      
      double error = pcl::geometry::distance(maxTH,pt2);
      std::cout << "Error distance centroid trunk-crown:" << error << std::endl;
      clusters_dbscan_map[error]= clouds_vector.at(i);  
  
  }
  
  std::map<double,pcl::PointCloud<pcl::PointXYZ>::Ptr>::iterator crown_better_segmented = clusters_dbscan_map.begin();
  crown_cloud->points.clear();
  pcl::copyPointCloud(*crown_better_segmented->second,*crown_cloud);
  
  std::cout << "Crown DBScan segmented:" << crown_cloud->points.size() << std::endl;
  
  std::string guardar= output_dir;
  guardar += "/3D_Mapping/cloud_crown_segmented_DBScan.pcd";
   
  pcl::io::savePCDFileBinary(guardar.c_str(),*crown_cloud); 
  
  
  //---------------------------------------------
  //---------------------------------------------
  

  std::cout << blue << "\nEstimating crown features..." << reset << std::endl;
  std::cout << "------------------------------------------" << std::endl;

  std::cout << yellow << "\nHeight." << reset << std::endl;
  std::cout << "------------------------------------------" << std::endl;

  std::map<double,pcl::PointXYZ> min_max_crownHeight;

  for(pcl::PointCloud<pcl::PointXYZ>::iterator it=crown_cloud->begin();it!=crown_cloud->end(); ++it){
    pcl::PointXYZ pt = pcl::PointXYZ(it->x,it->y,it->z);
    min_max_crownHeight[pt.y]=pt;
  }

  it1 = min_max_crownHeight.begin();
  minCH = it1->second;

  it2 = std::prev(min_max_crownHeight.end());
  maxCH = it2->second;

  feature << "minCH pt3d:" << minCH << std::endl;
  feature << "maxCH pt3d:" << maxCH << std::endl;

  minCH.x = maxCH.x;
  minCH.z = maxCH.z;

  height_crown = pcl::geometry::distance(minCH,maxCH);

  std::cout << "MaxCH: " << maxCH << std::endl;
  std::cout << "MinCH: " << minCH << std::endl;

  std::cout << blue << "\nEstimating other features..." << reset << std::endl;
  std::cout << "------------------------------------------" << std::endl;

  std::cout << yellow << "\nTotal height." << reset << std::endl;
  std::cout << "------------------------------------------" << std::endl;

  total_height = height_crown+height_trunk;
  PCL_INFO("H1: %f",height_crown);
  PCL_INFO("\nH2: %f",height_trunk);

  std::cout << yellow << "\nCrown volume." << reset << std::endl;
  std::cout << "------------------------------------------" << std::endl;

  std::vector<pcl::PointXYZ> pts_for_DBH_5m;

  for(pcl::PointCloud<pcl::PointXYZ>::iterator it=trunk_cloud->begin();it!=trunk_cloud->end(); ++it){

    pcl::PointXYZ pt = pcl::PointXYZ(it->x,it->y,it->z);
    if(pt.y>480 and pt.y < 580){
      pts_for_DBH_5m.push_back(pt);
    }else{
      continue;
    }
  }
  double DBH_5m=0;

  if(pts_for_DBH_5m.size()<=0){
    PCL_WARN("No points at 5m, using morphic factor = 0.75\n");
    factor_morfico = 0.75;

  }else{

    pts_for_DBH_5m.push_back(minDBH);
    pts_for_DBH_5m.push_back(maxDBH);

    PCL_INFO("\nPoints between 5.3+/-0.5cm: %lu",pts_for_DBH_5m.size());
    std::map<double,pcl::PointXYZ> min_max_DBH_5m;

    for(std::vector<pcl::PointXYZ>::iterator it=pts_for_DBH_5m.begin();it!=pts_for_DBH_5m.end(); ++it){
      pcl::PointXYZ pt = pcl::PointXYZ(it->x,it->y,it->z);
      min_max_DBH_5m[pt.x] = pt;
    }

    it1 = min_max_DBH_5m.begin();
    minDBH5 = it1->second;

    it2 = std::prev(min_max_DBH_5m.end());
    maxDBH5 = it2->second;

    std::cout << "\nMinDBH5:" << minDBH5 << std::endl;
    std::cout << "MaxDBH5:" << maxDBH5 << std::endl;

    feature << "minDBH5m pt3d:" << minDBH5 << std::endl;
    feature << "maxDBH5m pt3d:" << maxDBH5 << std::endl;
    feature << "--------------------------" << std::endl;

    minDBH5.y = maxDBH5.y;
    minDBH5.z = maxDBH5.z;
    DBH_5m = pcl::geometry::distance(minDBH5,maxDBH5);
    factor_morfico = DBH_5m/DBH;
  }

  crown_volume = (DBH*DBH)*(M_PI/4)*total_height*factor_morfico;

  pcl::PolygonMesh mesh;

  Utilities::create_mesh(crown_cloud,mesh);

  std::string mesh_path = output_dir;
  mesh_path += "3D_Mapping/mesh.ply";

  vtkSmartPointer<vtkPolyData> vtkCloud = vtkSmartPointer<vtkPolyData>::New();
  pcl::VTKUtils::convertToVTK(mesh,vtkCloud);


  Utilities::vizualizeMesh(vtkCloud);

  float volume = volumeOfMesh(mesh);

  std::cout << "\n*** Measurements ***" << std::endl;
  std::cout << "---------------------------------------" << std::endl;
  std::cout << green << "TRUNK" << reset << std::endl;
  std::cout << "---------------------------------------" << std::endl;
  std::cout << "------ Trunk height:" << height_trunk << " cm" << std::endl;
  std::cout << "------ DBH:" << DBH << " cm" << std::endl;
  std::cout << "------ DBH5m:" << DBH_5m << " cm" << std::endl;
  std::cout << "---------------------------------------" << std::endl;
  std::cout << green << "CROWN" << reset << std::endl;
  std::cout << "---------------------------------------" << std::endl;
  std::cout << "------ Crown height:" << height_crown << " cm" << std::endl;
  std::cout<< std::fixed;
  std::cout << "------ Crown volume (eq method):" << crown_volume << " cm^3" << std::endl;
  std::cout << "------ Crown volume (mesh):" << volume << " cm^3" << std::endl;
  std::cout << "---------------------------------------" << std::endl;
  std::cout << green << "OTHERS FEATURES" << reset << std::endl;
  std::cout << "---------------------------------------" << std::endl;
  std::cout << "------ Total height:" << total_height << " cm" << std::endl;
  std::cout << "------ Morphic factor:" << factor_morfico << std::endl;
  std::cout << "************************************************" << std::endl;
  std::cout << "************************************************" << std::endl;

  std::cout << "Saving results in:" << output_dir << std::endl;

  feature << "TRUNK" << "\n" << "Height:" << height_trunk << " cm" << std::endl;
  feature << "DBH:" << DBH << " cm" << std::endl;
  feature << "DBH 5m:" << DBH_5m << " cm" << std::endl;
  feature << "--------------------------" << std::endl;
  feature << "CROWN" << "\n" << "Height:" << height_crown << " cm" << std::endl;
  feature << "Volume (MESH):" << volume << " cm^3" << std::endl;
  feature << "Volume (Eq):" << crown_volume << " cm^3" << std::endl;
  feature << "--------------------------" << std::endl;
  feature << "Total height:" << total_height << std::endl;
  feature << "Morphic factor:" << factor_morfico << std::endl;
  feature.close();

}





