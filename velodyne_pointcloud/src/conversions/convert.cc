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

namespace velodyne_pointcloud
{
  /** @brief Constructor. */
  Convert::Convert(ros::NodeHandle node, ros::NodeHandle private_nh):
    data_(new velodyne_rawdata::RawData())
  {
    data_->setup(private_nh);


    // advertise output point cloud (before subscribing to input data)
    output_ =
      node.advertise<sensor_msgs::PointCloud2>("velodyne_points", 10);
      
    srv_ = boost::make_shared <dynamic_reconfigure::Server<velodyne_pointcloud::
      CloudNodeConfig> > (private_nh);
    dynamic_reconfigure::Server<velodyne_pointcloud::CloudNodeConfig>::
      CallbackType f;
    f = boost::bind (&Convert::callback, this, _1, _2);
    srv_->setCallback (f);

    // subscribe to VelodyneScan packets
    velodyne_scan_ =
      node.subscribe("velodyne_packets", 10,
                     &Convert::processScan, (Convert *) this,
                     ros::TransportHints().tcpNoDelay(true));

    section_angle_ = 0.0;
  }
  
  void Convert::callback(velodyne_pointcloud::CloudNodeConfig &config,
                uint32_t level)
  {
  ROS_INFO("Reconfigure Request");
  data_->setParameters(config.min_range, config.max_range, config.view_direction,
                       config.view_width);
  }

  /** @brief Callback for raw scan messages. */
  void Convert::processScan(const velodyne_msgs::VelodyneScan::ConstPtr &scanMsg)
  {
    if (output_.getNumSubscribers() == 0)         // no one listening?
      return;                                     // avoid much work

    // allocate a point cloud with same time and frame ID as raw data
    velodyne_rawdata::VPointCloud::Ptr outMsg(new velodyne_rawdata::VPointCloud());

    // outMsg's header is a pcl::PCLHeader, convert it before stamp assignment
    outMsg->header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
    outMsg->header.frame_id = scanMsg->header.frame_id;
    outMsg->height = 1;
    
    //Debug 
    //std::cout << "Processing scan!\n"; 
    
    // process each packet provided by the driver
    for (size_t i = 0; i < scanMsg->packets.size(); ++i)
    {
      section_angle_ += data_->unpack(scanMsg->packets[i], *outMsg);
    }

    //Accumulate the pt cloud
    accumulated_cloud_.points.insert(accumulated_cloud_.points.end(), outMsg->points.begin(), outMsg->points.end());
    accumulated_cloud_.width = accumulated_cloud_.points.size();
    //std::cout << "accumulated angle: " << section_angle_ << std::endl;
    
    // publish the accumulated cloud message
    ROS_DEBUG_STREAM("Publishing " << outMsg->height * outMsg->width
                     << " Velodyne points, time: " << outMsg->header.stamp);

    if(section_angle_/100.0 >= 360.0)
    {
      accumulated_cloud_.header.stamp = outMsg->header.stamp;
      accumulated_cloud_.header.frame_id = outMsg->header.frame_id;
      accumulated_cloud_.height = outMsg->height;
      output_.publish(accumulated_cloud_);
      section_angle_ = 0.0;
      accumulated_cloud_.points.clear();
      accumulated_cloud_.width = 0;
    }             
      
    
  }

} // namespace velodyne_pointcloud
