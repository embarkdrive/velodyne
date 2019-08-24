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
  void processScanBUGGED(const velodyne_msgs::VelodyneScan::ConstPtr& scanMsg);

  /// Pointer to dynamic reconfigure service srv_
  boost::shared_ptr<dynamic_reconfigure::Server<velodyne_pointcloud::CloudNodeConfig> > srv_;

  boost::shared_ptr<velodyne_rawdata::RawData> data_;
  ros::Subscriber velodyne_scan_;
  ros::Publisher pointcloud_publisher_;
  ros::Publisher deskew_info_publisher_;

  // make the pointcloud container a member variable to append different slices
  velodyne_rawdata::VPointCloud accumulated_cloud_;
  velodyne_msgs::VelodyneDeskewInfo deskew_info_;
  float prev_azimuth_;
  ros::Time prev_stamp_;

  // Let's check the sum of the packates, should be very unlikely to get the same number twice...
  std::set<u_int64_t> packet_checksums_;
  size_t duplicates_ = 0;
  std::set<std::tuple<float,float,float>> pnts_set_; // expensive -- just bug-hunting!!!

  std::vector<size_t> packet_sizes_for_scan_;

  std::vector<float> azimuth_for_scan_;

  /// configuration parameters
  typedef struct
  {
    int npackets; ///< number of packets to combine
  } Config;
  Config config_;
};

} // namespace velodyne_pointcloud

#endif // _VELODYNE_POINTCLOUD_CONVERT_H_
