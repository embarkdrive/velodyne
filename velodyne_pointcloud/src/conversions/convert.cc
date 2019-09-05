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

#include <geometry_msgs/TransformStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace {

double getYawFromMsg(const geometry_msgs::TransformStamped& transform_msg)
{
  tf2::Quaternion quat_tf;
  tf2::convert(transform_msg.transform.rotation, quat_tf);
  tf2::Matrix3x3 m_tf(quat_tf);
  double roll, pitch, yaw;
  m_tf.getRPY(roll, pitch, yaw);
  return yaw;
}

} // namespace

namespace velodyne_pointcloud {
/** @brief Constructor. */
Convert::Convert(ros::NodeHandle node, ros::NodeHandle private_nh)
  : data_(new velodyne_rawdata::RawData()), prev_azimuth_(-1.0), prev_stamp_(ros::Time())
{
  // Read configuration for our sensor
  data_->setup(private_nh);

  setupVisibilityAngles(node, private_nh);

  accumulated_cloud_.width = 0;
  accumulated_cloud_.height = 1;

  cropped_accumulated_cloud_.width = 0;
  cropped_accumulated_cloud_.height = 1;

  // advertise output point cloud (before subscribing to input data)
  pointcloud_publisher_ = node.advertise<sensor_msgs::PointCloud2>("velodyne_points", 10);
  cropped_pointcloud_publisher_ =
      node.advertise<sensor_msgs::PointCloud2>("cropped_velodyne_points", 10);

  // advertise output deskew info
  deskew_info_publisher_ =
      node.advertise<velodyne_msgs::VelodyneDeskewInfo>("velodyne_deskew_info", 10);

  srv_ = boost::make_shared<dynamic_reconfigure::Server<velodyne_pointcloud::CloudNodeConfig> >(
      private_nh);
  dynamic_reconfigure::Server<velodyne_pointcloud::CloudNodeConfig>::CallbackType f;
  f = boost::bind(&Convert::callback, this, _1, _2);
  srv_->setCallback(f);

  synced_with_visibility_ = false;

  last_sample_added_ = false;
  start_full_sweep_ = start_cropped_sweep_ = std::chrono::steady_clock::now();

  // Add an extra entry for angle 0, for initial sweep
  deskew_info_.sweep_info.push_back(create_sweep_entry(prev_stamp_, 0.0));

  // subscribe to VelodyneScan packets
  velodyne_scan_ = node.subscribe("velodyne_packets", 10, &Convert::processScan, (Convert*)this,
                                  ros::TransportHints().tcpNoDelay(true));
}

void Convert::setupVisibilityAngles(ros::NodeHandle& node, ros::NodeHandle& private_nh)
{
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);

  // Relative to x-axis of vehicle frame, counter clock-wise
  // The "front" angle is the start angle for the left sensor, if it spins counter-cw
  float visibility_angle_left_front =
      node.param("/config/vehicle/vehicle/visiblity_angle_left_front", -1.08350f);
  float visibility_angle_left_rear =
      node.param("/config/vehicle/vehicle/visiblity_angle_left_rear", 3.20565f);

  // The "rear" angle is the start angle for the right sensor, if it spins counter-cw
  float visibility_angle_right_rear =
      node.param("/config/vehicle/vehicle/visiblity_angle_right_rear", -3.20565f);
  float visibility_angle_right_front =
      node.param("/config/vehicle/vehicle/visiblity_angle_right_front", 1.08350f);


  // Look up yaw of velodyne relative to vehicle frame
  std::string frame_id;
  if (!private_nh.param<std::string>("frame_id", frame_id, "")) {
    throw std::logic_error("Param frame_id not set");
  }

  ROS_INFO("Waiting for transfrom from sensor: %s to vehicle_frame", frame_id.c_str());

  std::string errstr;
  if (!tf_buffer.canTransform(frame_id, "vehicle_frame", ros::Time(0), ros::Duration(10),
                              &errstr)) {
    ROS_ERROR("Required transform %s -> vehicle_frame not found in 10 seconds",
              ros::this_node::getName().c_str());
    throw std::logic_error("Transform invalid");
  }
  geometry_msgs::TransformStamped veh_to_sensor =
      tf_buffer.lookupTransform(frame_id, "vehicle_frame", ros::Time(0));
  ROS_INFO("Transform to vehicle frame found");

  double yaw_sens_veh = -getYawFromMsg(veh_to_sensor);
  // are we the left or right sensor?
  if (-veh_to_sensor.transform.translation.y >= 0) {
    ROS_INFO("Using left sensor angles");
    // left
    visibility_angle_start_ = visibility_angle_left_front - yaw_sens_veh;
    visibility_angle_end_ = visibility_angle_left_rear - yaw_sens_veh;
  } else {
    ROS_INFO("Using right sensor angles");
    // right
    visibility_angle_start_ = visibility_angle_right_rear - yaw_sens_veh;
    visibility_angle_end_ = visibility_angle_right_front - yaw_sens_veh;
  }

  visibility_angle_start_ = 180.0 / M_PI * visibility_angle_start_;
  visibility_angle_end_ = 180.0 / M_PI * visibility_angle_end_;

  ROS_INFO("Counter clockwise angles sens frame: %.2f -> %.2f (yaw_sens: %.2f)",
           visibility_angle_start_, visibility_angle_end_, 180.0 / M_PI * yaw_sens_veh);

  // Convert angles to azimuth (degrees clockwise from X-axis)
  // TODO, this is really weird (at page 38 in VLP32C manual, 0deg is towards the y-axis, but this
  // works...)
  visibility_angle_start_ = -visibility_angle_start_;
  visibility_angle_end_ = -visibility_angle_end_;
  while (visibility_angle_start_ < 0) {
    visibility_angle_start_ += 360;
  }
  while (visibility_angle_end_ < 0) {
    visibility_angle_end_ += 360;
  }

  // Because the sensor spins with INCREASING azimuth, we need to swap start and end angles
  std::swap(visibility_angle_start_, visibility_angle_end_);

  ROS_INFO("Will process points between azimuth %.2f -> %.2f deg.", visibility_angle_start_,
           visibility_angle_end_);
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

void Convert::emitFullSweep(const velodyne_msgs::VelodyneScan::ConstPtr& scanMsg)
{
  const auto t_end = std::chrono::steady_clock::now();
  ROS_INFO("Full sweep acquisition in = %.2f ms",
           std::chrono::duration_cast<std::chrono::nanoseconds>(t_end - start_full_sweep_).count() /
               1.0e6);

  accumulated_cloud_.header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
  accumulated_cloud_.header.frame_id = scanMsg->header.frame_id;
  assert(accumulated_cloud_.width == accumulated_cloud_.points.size());

  pointcloud_publisher_.publish(accumulated_cloud_);

  deskew_info_.header.stamp = scanMsg->header.stamp;
  deskew_info_.header.frame_id = scanMsg->header.frame_id;
  deskew_info_publisher_.publish(deskew_info_);

  // Clear data we are accumulating
  accumulated_cloud_.points.clear();
  accumulated_cloud_.width = 0;
  deskew_info_.sweep_info.clear();

  // Add an extra entry for angle 0, for next sweep
  deskew_info_.sweep_info.push_back(create_sweep_entry(prev_stamp_, 0.0));
}

void Convert::emitCroppedSweep(const velodyne_msgs::VelodyneScan::ConstPtr& scanMsg)
{
  const auto t_end = std::chrono::steady_clock::now();
  ROS_INFO(
      "Cropped sweep acquisition in = %.2f ms",
      std::chrono::duration_cast<std::chrono::nanoseconds>(t_end - start_cropped_sweep_).count() /
          1.0e6);

  cropped_accumulated_cloud_.header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
  cropped_accumulated_cloud_.header.frame_id = scanMsg->header.frame_id;
  assert(cropped_accumulated_cloud_.width == cropped_accumulated_cloud_.points.size());

  cropped_pointcloud_publisher_.publish(cropped_accumulated_cloud_);
  // ROS_DEBUG("Publish cropped pc with %lu points", cropped_accumulated_cloud_.points.size());

  cropped_accumulated_cloud_.points.clear();
  cropped_accumulated_cloud_.width = 0;
}

/** @brief Callback for raw scan messages. */
void Convert::processScan(const velodyne_msgs::VelodyneScan::ConstPtr& scanMsg)
{
  for (size_t i = 0; i < scanMsg->packets.size(); ++i) {
    const velodyne_rawdata::raw_packet_t* raw =
        (const velodyne_rawdata::raw_packet_t*)&scanMsg->packets[i].data[0];

    // azimuth corresponds to the starting sweep angle for the current packet
    // (0 deg is y-axis of velodyne, increasing CLOCK-WISE)
    float azimuth = float(raw->blocks[0].rotation) / 100.0;
    // ROS_DEBUG("azimuth = %.2f", azimuth);

    // wait until prev_azimuth -> azimuth containts start visibility angle
    if (synced_with_visibility_) {
      // azimuth value will wrap around after a full 360 degree sweep
      if (azimuth < prev_azimuth_) {
        emitFullSweep(scanMsg);
        start_full_sweep_ = std::chrono::steady_clock::now();
      }

      // Seen all the relevant parts of the sweep
      if (prev_azimuth_ <= visibility_angle_end_ && visibility_angle_end_ <= azimuth) {
        emitCroppedSweep(scanMsg);
      }

      size_t start_index = accumulated_cloud_.points.size();

      data_->unpackAndAdd(scanMsg->packets[i], accumulated_cloud_);
      deskew_info_.sweep_info.push_back(create_sweep_entry(scanMsg->packets[i].stamp, azimuth));

      if (visibility_angle_start_ > visibility_angle_end_) {
        // Sweep we want to capture goes across 360 deg.
        if (visibility_angle_start_ <= azimuth || azimuth <= visibility_angle_end_) {
          for (size_t j = start_index; j < accumulated_cloud_.points.size(); ++j) {
            cropped_accumulated_cloud_.points.push_back(accumulated_cloud_.points[j]);
            ++cropped_accumulated_cloud_.width;
          }
          if (!last_sample_added_) {
            start_cropped_sweep_ = std::chrono::steady_clock::now();
          }
          last_sample_added_ = true;
        } else {
          last_sample_added_ = false;
        }
      } else {
        // Sweep doesn't cross 360 deg.
        if (visibility_angle_start_ <= azimuth && azimuth <= visibility_angle_end_) {
          for (size_t j = start_index; j < accumulated_cloud_.points.size(); ++j) {
            cropped_accumulated_cloud_.points.push_back(accumulated_cloud_.points[j]);
            ++cropped_accumulated_cloud_.width;
          }
          if (!last_sample_added_) {
            start_cropped_sweep_ = std::chrono::steady_clock::now();
          }
          last_sample_added_ = true;
        } else {
          last_sample_added_ = false;
        }
      }
    } else {
      // Check that interval prev-curr overlaps start
      if (azimuth < prev_azimuth_) { // interval overlaps 360
        if (prev_azimuth_ >= 0 && prev_azimuth_ - 360 <= visibility_angle_start_ &&
            visibility_angle_start_ <= azimuth) {
          synced_with_visibility_ = true;
          ROS_DEBUG("Azimuth synced");
        }
      } else {
        if (prev_azimuth_ >= 0 && prev_azimuth_ <= visibility_angle_start_ &&
            visibility_angle_start_ <= azimuth) {
          synced_with_visibility_ = true;
          ROS_DEBUG("Azimuth synced");
        }
      }
    }

    prev_azimuth_ = azimuth;
    prev_stamp_ = scanMsg->packets[i].stamp;
  }
}

} // namespace velodyne_pointcloud
