#include "Segmentation.h"
 
class Dendrometry{

  private:

  public:

    Dendrometry(){}

    ~Dendrometry(){}

    static void estimate(const pcl::PointCloud<pcl::PointXYZ>::Ptr& trunk_cloud,
                         const pcl::PointCloud<pcl::PointXYZ>::Ptr& crown_cloud,
                         const std::string output_dir,
                         pcl::PointXYZ& minDBH,pcl::PointXYZ& maxDBH,pcl::PointXYZ& minTH,pcl::PointXYZ& maxTH,
                         pcl::PointXYZ& minCH,pcl::PointXYZ& maxCH,pcl::PointXYZ& minDBH5,pcl::PointXYZ& maxDBH5);

};

