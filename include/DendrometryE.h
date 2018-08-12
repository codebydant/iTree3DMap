#include "Segmentation.h"
 
class Dendrometry{

  private:

  public:

    Dendrometry(){}

    ~Dendrometry(){}

    static void estimate(const pcl::PointCloud<pcl::PointXYZ>::Ptr& trunk_cloud,
                         const pcl::PointCloud<pcl::PointXYZ>::Ptr& crown_cloud,
                         pcl::PointXYZ minDBH,pcl::PointXYZ maxDBH);

};

