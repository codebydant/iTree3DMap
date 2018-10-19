

#include "Utilities.h"

class Segmentation{

private:


public:

  Segmentation(){}
  ~Segmentation(){}


 static bool trunkSegmentation(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_aligned,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_without_trunk,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_trunk,bool setGUI=false,
                               Eigen::Matrix4f align_cloud=Eigen::Matrix4f::Identity(),
                               int K=50,double distanceWeight=0.1,
                               int maxIterations=500,double distanceThreshold=0.5,
                               double distanceWeight_cylinder=0.5,int maxIterations_cylinder=10000,
                               double distanceThreshold_cylinder=8, double minRadius=0, double maxRadius=60);

 static bool crownSegmentation(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_crown,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_better_Segmented,bool setGUI,int octreeResolution=124, double eps=60,int minPtsAux=3,int minPts=3);

 static bool extractTree(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
                         const std::string& output_path,
                         pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_aligned,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr& trunk_cloud,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr& tree_segmented,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr& crown_segmented);
                         
 static bool DBScan(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_segmented,pcl::PointXYZ& minTrunkHeight);

 static void MultipleViewportsVTK(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud1,
                                         pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud2);

};
