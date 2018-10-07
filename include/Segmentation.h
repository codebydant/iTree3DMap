

#include "Utilities.h"

class Segmentation{

private:


public:

  Segmentation(){}
  ~Segmentation(){}


 static bool trunkSegmentation(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_without_trunk,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_trunk,bool setGUI=false,int K=50,double distanceWeight=0.1,
                               int maxIterations=500,double distanceThreshold=0.5,
                               double distanceWeight_cylinder=0.5,int maxIterations_cylinder=10000,
                               double distanceThreshold_cylinder=8, double minRadius=0, double maxRadius=60);

 static bool crownSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_without_trunk,
                                      pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_crown);

 static bool extractTree(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
                         const std::string& output_path,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr& trunk_cloud,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr& tree_segmented,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr& crown_segmented);



};
