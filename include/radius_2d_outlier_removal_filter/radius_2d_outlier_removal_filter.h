/*
 * Copyright 2019-2020 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef RADIUS_2D_OUTLIER_REMOVAL_FILTER_H_
#define RADIUS_2D_OUTLIER_REMOVAL_FILTER_H_

#include <ros/ros.h>
#include <ros/names.h>
#include <pcl_ros/pcl_nodelet.h>
#include <pcl/point_types.h>
#include <pcl/common/impl/common.hpp>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/extract_indices.h>

namespace radius_2d_outlier_removal_filter
{
class Radius2dOutlierRemovalFilter : public pcl_ros::PCLNodelet
{
public:
  Radius2dOutlierRemovalFilter();

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
}  // namespace radius_2d_outlier_removal_filter

#endif  // RADIUS_2D_OUTLIER_REMOVAL_FILTER_H_