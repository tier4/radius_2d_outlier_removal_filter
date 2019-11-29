#ifndef CYLINDER_SEARCH_FILTER_H_
#define CYLINDER_SEARCH_FILTER_H_

#include <ros/ros.h>
#include <ros/names.h>
#include <pcl_ros/pcl_nodelet.h>
#include <pcl/point_types.h>
#include <pcl/common/impl/common.hpp>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/extract_indices.h>

namespace cylinder_search_filter
{
class CylinderSearchFilter : public pcl_ros::PCLNodelet
{
public:
  CylinderSearchFilter();

protected:
  ros::Subscriber input_subscriber_;
  ros::Publisher output_publisher_;

private:
  virtual void onInit();
  void filter(const sensor_msgs::PointCloud2 input);

  pcl::RadiusOutlierRemoval<pcl::PCLPointCloud2> radius_outlier_removal_;
  pcl::search::Search<pcl::PointXY>::Ptr kd_tree_;
  pcl::ExtractIndices<pcl::PCLPointCloud2> extract_indices_;
  int min_neighbors_ = 0;
  double radius_search_ = 0.0;
};
}  // namespace cylinder_search_filter

#endif  // CYLINDER_SEARCH_FILTER_H_