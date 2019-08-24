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

#include "convert.h"

#include <pcl_conversions/pcl_conversions.h>

namespace velodyne_pointcloud {
/** @brief Constructor. */
Convert::Convert(ros::NodeHandle node, ros::NodeHandle private_nh)
  : data_(new velodyne_rawdata::RawData()), prev_azimuth_(0.0), prev_stamp_(ros::Time())
{
  data_->setup(private_nh);


  // advertise output point cloud (before subscribing to input data)
  pointcloud_publisher_ = node.advertise<sensor_msgs::PointCloud2>("velodyne_points", 10);

  // advertise output deskew info
  deskew_info_publisher_ =
      node.advertise<velodyne_msgs::VelodyneDeskewInfo>("velodyne_deskew_info", 10);

  srv_ = boost::make_shared<dynamic_reconfigure::Server<velodyne_pointcloud::CloudNodeConfig> >(
      private_nh);
  dynamic_reconfigure::Server<velodyne_pointcloud::CloudNodeConfig>::CallbackType f;
  f = boost::bind(&Convert::callback, this, _1, _2);
  srv_->setCallback(f);

  prev_azimuth_ = 0.0f;
  accumulated_cloud_.height = 1;
  accumulated_cloud_.width = 0;
  accumulated_cloud_.points.clear();

  // Add an extra entry for angle 0, for initial sweep
  deskew_info_.sweep_info.push_back(create_sweep_entry(prev_stamp_, 0.0));

  // subscribe to VelodyneScan packets
  velodyne_scan_ = node.subscribe("velodyne_packets", 10, &Convert::processScan, (Convert*)this,
                                  ros::TransportHints().tcpNoDelay(true));
}

void Convert::callback(velodyne_pointcloud::CloudNodeConfig& config, uint32_t level)
{
  ROS_INFO("Reconfigure Request");
  data_->setParameters(config.min_range, config.max_range, config.view_direction,
                       config.view_width);
}

velodyne_msgs::VelodyneSweepInfo Convert::create_sweep_entry(ros::Time stamp, float angle)
{
  velodyne_msgs::VelodyneSweepInfo sweep_info;
  sweep_info.stamp = stamp;
  sweep_info.start_angle = angle;
  return sweep_info;
}

void Convert::processScan(const velodyne_msgs::VelodyneScan::ConstPtr& scanMsg)
{
  for (size_t i = 0; i < scanMsg->packets.size(); ++i) {
    const velodyne_rawdata::raw_packet_t* raw =
        (const velodyne_rawdata::raw_packet_t*)&scanMsg->packets[i].data[0];

    // azimuth corresponds to the starting sweep angle for the current packet
    float azimuth = float(raw->blocks[0].rotation) / 100.0;

    // azimuth value will wrap around after a full 360 degree sweep
    if (azimuth < prev_azimuth_) {
      ROS_INFO("Azimuth = %.2f (prev = %.2f), publish", azimuth, prev_azimuth_);

      // Publish data for the full sweep
      accumulated_cloud_.header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
      accumulated_cloud_.header.frame_id = scanMsg->header.frame_id;
      assert(accumulated_cloud_.width == accumulated_cloud_.points.size());
      accumulated_cloud_.width = accumulated_cloud_.points.size();

      size_t actual_cnt =
          std::accumulate(packet_sizes_for_scan_.begin(), packet_sizes_for_scan_.end(), 0);
      ROS_INFO("Full sweep sz: %lu, expected cnt: %lu, from: %lu PKT",
               accumulated_cloud_.points.size(), actual_cnt, packet_sizes_for_scan_.size());
      pointcloud_publisher_.publish(accumulated_cloud_);

      deskew_info_.header.stamp = scanMsg->header.stamp;
      deskew_info_.header.frame_id = scanMsg->header.frame_id;
      deskew_info_publisher_.publish(deskew_info_);

      std::cout << "az for scan: [";
      for (float az : azimuth_for_scan_) {
        std::cout << az << ", ";
      }
      std::cout << "]" << std::endl;

      // Clear data we are accumulating
      accumulated_cloud_.points.clear();
      accumulated_cloud_.width = 0;
      deskew_info_.sweep_info.clear();
      packet_sizes_for_scan_.clear();
      azimuth_for_scan_.clear();

      // Add an extra entry for angle 0, for next sweep
      deskew_info_.sweep_info.push_back(create_sweep_entry(prev_stamp_, 0.0));
    }

    azimuth_for_scan_.push_back(azimuth);

    size_t valid_points_for_packet;
    data_->unpack(scanMsg->packets[i], accumulated_cloud_, &valid_points_for_packet);
    packet_sizes_for_scan_.push_back(valid_points_for_packet);

    deskew_info_.sweep_info.push_back(create_sweep_entry(scanMsg->packets[i].stamp, azimuth));
    prev_azimuth_ = azimuth;
    prev_stamp_ = scanMsg->packets[i].stamp;
  }
}

/** @brief Callback for raw scan messages. */
void Convert::processScanBUGGED(const velodyne_msgs::VelodyneScan::ConstPtr& scanMsg)
{
  if (pointcloud_publisher_.getNumSubscribers() == 0) // no one listening?
    return;                                           // avoid much work

  // allocate a point cloud with same time and frame ID as raw data
  velodyne_rawdata::VPointCloud::Ptr partial_scan_pointcloud(new velodyne_rawdata::VPointCloud());

  // outMsg's header is a pcl::PCLHeader, convert it before stamp assignment
  partial_scan_pointcloud->header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
  partial_scan_pointcloud->header.frame_id = scanMsg->header.frame_id;
  partial_scan_pointcloud->height = 1;

  ROS_INFO("Processing %lu packets", scanMsg->packets.size());

  // process each packet provided by the driver
  for (size_t i = 0; i < scanMsg->packets.size(); ++i) {
    const velodyne_rawdata::raw_packet_t* raw =
        (const velodyne_rawdata::raw_packet_t*)&scanMsg->packets[i].data[0];

    u_int64_t pkt_checksum = 0;
    for (int j = 0; j < velodyne_rawdata::PACKET_SIZE - 6;
         ++j) { // don't include timestamp and factory
      pkt_checksum += reinterpret_cast<const u_int8_t*>(raw)[j] * j;
    }
    if (packet_checksums_.count(pkt_checksum) == 0) {
      packet_checksums_.insert(pkt_checksum);
    } else {
      ROS_WARN("Identical packet checksum!");
    }

    // azimuth corresponds to the starting sweep angle for the current packet
    float azimuth = float(raw->blocks[0].rotation) / 100.0;

    // azimuth value will wrap around after a full 360 degree sweep
    // once we save all packets for the last full 360 degrees, publish them
    if (azimuth < prev_azimuth_) {
      accumulated_cloud_.header.stamp = partial_scan_pointcloud->header.stamp;
      accumulated_cloud_.header.frame_id = partial_scan_pointcloud->header.frame_id;
      accumulated_cloud_.height = partial_scan_pointcloud->height;
      accumulated_cloud_.width = accumulated_cloud_.points.size();

      size_t actual_cnt =
          std::accumulate(packet_sizes_for_scan_.begin(), packet_sizes_for_scan_.end(), 0);
      ROS_INFO("Full sweep sz: %lu, expected cnt: %lu", accumulated_cloud_.points.size(),
               actual_cnt);
      pointcloud_publisher_.publish(accumulated_cloud_);

      for (size_t j = 0; j < accumulated_cloud_.points.size(); ++j) {
        const velodyne_rawdata::VPoint& pnt = accumulated_cloud_.points[j];
        auto pnt_tuple = std::make_tuple(pnt.x, pnt.y, pnt.z);
        if (pnts_set_.count(pnt_tuple) == 0) {
          pnts_set_.insert(pnt_tuple);
        } else {
          duplicates_++;
        }
      }
      ROS_INFO("Num duplicates: %lu", duplicates_);
      std::cout << "Valid counts for scan: ";
      for (size_t n_valid : packet_sizes_for_scan_) {
        std::cout << n_valid << ",";
      }
      std::cout << std::endl;

      deskew_info_.header.stamp = scanMsg->header.stamp;
      deskew_info_.header.frame_id = scanMsg->header.frame_id;
      deskew_info_publisher_.publish(deskew_info_);

      accumulated_cloud_.points.clear();
      accumulated_cloud_.width = 0;
      deskew_info_.sweep_info.clear();

      // Add an extra entry for angle 0, for next sweep
      deskew_info_.sweep_info.push_back(create_sweep_entry(prev_stamp_, 0.0));

      packet_sizes_for_scan_.clear();
      packet_checksums_.clear();
      duplicates_ = 0;
      pnts_set_.clear();
    }

    size_t valid_points_for_packet;
    data_->unpack(scanMsg->packets[i], *partial_scan_pointcloud, &valid_points_for_packet);
    packet_sizes_for_scan_.push_back(valid_points_for_packet);

    // Accumulate the pt cloud
    accumulated_cloud_.points.insert(accumulated_cloud_.points.end(),
                                     partial_scan_pointcloud->points.begin(),
                                     partial_scan_pointcloud->points.end());

    // Get the sweep info
    deskew_info_.sweep_info.push_back(create_sweep_entry(scanMsg->packets[i].stamp, azimuth));

    prev_azimuth_ = azimuth;
    prev_stamp_ = scanMsg->packets[i].stamp;
  }
}

} // namespace velodyne_pointcloud
