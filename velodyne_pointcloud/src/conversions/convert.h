/* -*- mode: C++ -*- */
/*
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2011 Jesse Vera
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file

    This class converts raw Velodyne 3D LIDAR packets to PointCloud2.

*/

#ifndef _VELODYNE_POINTCLOUD_CONVERT_H_
#define _VELODYNE_POINTCLOUD_CONVERT_H_ 1

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <velodyne_pointcloud/rawdata.h>

#include <dynamic_reconfigure/server.h>
#include <velodyne_pointcloud/CloudNodeConfig.h>

#include <velodyne_msgs/VelodyneDeskewInfo.h>
#include <velodyne_msgs/VelodyneSweepInfo.h>

#include "diagnostics_utils/instrumentation.h"

namespace velodyne_pointcloud {
class Convert
{
 public:
  Convert(ros::NodeHandle node, ros::NodeHandle private_nh);
  ~Convert()
  {
  }

 private:
  void callback(velodyne_pointcloud::CloudNodeConfig& config, uint32_t level);
  velodyne_msgs::VelodyneSweepInfo create_sweep_entry(ros::Time stamp, float angle);
  void processScan(const velodyne_msgs::VelodyneScan::ConstPtr& scanMsg);

  /// Pointer to dynamic reconfigure service srv_
  boost::shared_ptr<dynamic_reconfigure::Server<velodyne_pointcloud::CloudNodeConfig> > srv_;

  boost::shared_ptr<velodyne_rawdata::RawData> data_;
  diagnostics_utils::SubscriberWrapper<velodyne_msgs::VelodyneScan> velodyne_scan_;
  diagnostics_utils::PublisherWrapper<velodyne_rawdata::VPointCloud> pointcloud_publisher_;
  diagnostics_utils::PublisherWrapper<velodyne_msgs::VelodyneDeskewInfo> deskew_info_publisher_;
  diagnostics_utils::TraceFrame trace_frame_ = diagnostics_utils::TraceFrame::INVALID;

  // make the pointcloud container a member variable to append different slices
  velodyne_rawdata::VPointCloud accumulated_cloud_;
  velodyne_msgs::VelodyneDeskewInfo deskew_info_;
  float prev_azimuth_;
  ros::Time prev_stamp_;
  ros::Time start_stamp_;
  /// configuration parameters
  typedef struct
  {
    int npackets; ///< number of packets to combine
  } Config;
  Config config_;
};

} // namespace velodyne_pointcloud

#endif // _VELODYNE_POINTCLOUD_CONVERT_H_
