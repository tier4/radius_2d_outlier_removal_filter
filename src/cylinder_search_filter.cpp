#include "cylinder_search_filter/cylinder_search_filter.h"
#include <pluginlib/class_list_macros.h>
#include <pcl_conversions/pcl_conversions.h>

namespace cylinder_search_filter
{
CylinderSearchFilter::CylinderSearchFilter()
{
  kd_tree_ = boost::make_shared<pcl::search::KdTree<pcl::PointXY> >(false);
}

void CylinderSearchFilter::onInit(void)
{
  PCLNodelet::onInit();

  // Define topics
  input_subscriber_ = pnh_->subscribe("input", 1, &CylinderSearchFilter::filter, this);
  output_publisher_ = pnh_->advertise<sensor_msgs::PointCloud2>("output", 1);

  // Get parameters
  min_neighbors_ = pnh_->param<int>("min_neighbors", 0);
  radius_search_ = pnh_->param<double>("radius_search", 0.0);

  // Set parameters to PCL
  radius_outlier_removal_.setMinNeighborsInRadius(min_neighbors_);
  radius_outlier_removal_.setRadiusSearch(radius_search_);
}

void CylinderSearchFilter::filter(const sensor_msgs::PointCloud2 input)
{
  int numInputPoints = input.height * input.width;
  std::vector<int> nn_indices(numInputPoints);
  std::vector<float> nn_dists(numInputPoints);

  // Convert to PointCloud
  pcl::PointCloud<pcl::PointXY>::Ptr cloud(new pcl::PointCloud<pcl::PointXY>);
  cloud->header = pcl_conversions::toPCL(input.header);
  cloud->height = 1;
  cloud->width = numInputPoints;
  cloud->is_dense = (input.is_dense == 1);
  cloud->points.resize(numInputPoints);
  for (int i = 0; i < numInputPoints; i++)
  {
    pcl::PointXY& point = cloud->points[i];
    memcpy(&point.x, &input.data[i * input.point_step + input.fields[0].offset], sizeof(float));
    memcpy(&point.y, &input.data[i * input.point_step + input.fields[1].offset], sizeof(float));
  }

  // Provide output
  sensor_msgs::PointCloud2 output;
  output.header = input.header;
  output.fields = input.fields;
  output.is_bigendian = input.is_bigendian;
  output.point_step = input.point_step;
  output.height = 1;
  output.is_dense = input.is_dense;
  output.data.resize(input.width * input.point_step);

  // Filtering
  int numPassed = 0;
  kd_tree_->setInputCloud(cloud);
  for (int i = 0; i < numInputPoints; i++)
  {
    int k = kd_tree_->radiusSearch(i, radius_search_, nn_indices, nn_dists);
    if (k >= min_neighbors_)
    {
      memcpy(&output.data[numPassed * output.point_step], &input.data[i * output.point_step], output.point_step);
      numPassed++;
    }
  }

  // Set info for output
  output.width = numPassed;
  output.row_step = output.point_step * output.width;
  output.data.resize(output.width * output.point_step);

  // Publish
  ROS_DEBUG("[CylinderSearchFilter::filter] Input size: %d, output size: %d", input.height * input.width,
            output.height * output.width);
  output_publisher_.publish(output);
}

}  // namespace cylinder_search_filter

typedef cylinder_search_filter::CylinderSearchFilter CylinderSearchFilter;
PLUGINLIB_EXPORT_CLASS(cylinder_search_filter::CylinderSearchFilter, nodelet::Nodelet);