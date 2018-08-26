#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>

#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include "Utilities.h"

class Segmentation{

private:


public:

  Segmentation(){}
  ~Segmentation(){}


 static void trunkSegmentation(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_without_trunk,
                                      pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_trunk);

 static void crownSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_without_trunk,
                                      pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_crown);

 static void extractTree(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
                         const std::string& output_path,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr& trunk_cloud,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr& tree_segmented,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr& crown_segmented);



};
