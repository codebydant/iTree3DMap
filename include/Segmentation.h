

#include "Utilities.h"

class Segmentation{

private:


public:

  Segmentation(){}
  ~Segmentation(){}


 static bool trunkSegmentation(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_without_trunk,
                                      pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_trunk);

 static bool crownSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_without_trunk,
                                      pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_crown);

 static bool extractTree(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
                         const std::string& output_path,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr& trunk_cloud,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr& tree_segmented,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr& crown_segmented);



};
