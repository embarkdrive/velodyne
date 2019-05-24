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
#include <algorithm>
#include <pcl_conversions/pcl_conversions.h>

namespace velodyne_pointcloud
{
  /** @brief Constructor. */
  Convert::Convert(ros::NodeHandle node, ros::NodeHandle private_nh):
    data_(new velodyne_rawdata::RawData()),
    odom_spinner_(&node, "", 30)
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
    
    // subscribe to /odom
    odom_sub_ =
        odom_spinner_.get_nh()->subscribe("/odom", 5, &Convert::processOdom, (Convert *) this);
    odom_spinner_.start();
                     
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
  
  /** @brief Callback for odometry messages. */
  void Convert::processOdom(const nav_msgs::Odometry::ConstPtr &odomMsg)
  {
      ROS_INFO_STREAM(" Odom_rcvd: " << odomMsg->header.stamp << ", Now :" << ros::Time::now());
    // Save last N odom messages sorted by time
    if(odom_sorted_.empty()){
        odom_sorted_.push_back(*odomMsg);
    } else {
        std::vector<nav_msgs::Odometry>::iterator low = std::lower_bound(odom_sorted_.begin(), odom_sorted_.end(), *odomMsg, timecomparison());
        odom_sorted_.insert(low, *odomMsg);
        if(odom_sorted_.size() > SIZE_OF_ODOM_LIST){
            odom_sorted_.erase(odom_sorted_.begin());
        }
    }
    //debugPrintOdom();
  }
  
  void Convert::debugPrintOdom(){
      std::vector<nav_msgs::Odometry>::iterator it;
      for(it = odom_sorted_.begin(); it!= odom_sorted_.end(); ++it){
          ROS_INFO_STREAM("Time: " << it->header.stamp <<
           ", px: " << it->pose.pose.position.x << 
           ", py: " << it->pose.pose.position.y << 
           ", pz: " << it->pose.pose.position.z << 
           ", qx: " << it->pose.pose.orientation.x << 
           ", qy: " << it->pose.pose.orientation.y << 
           ", qy: " << it->pose.pose.orientation.z <<
           ", qw: " << it->pose.pose.orientation.w);
      }
  }
  
  std::vector<nav_msgs::Odometry>::iterator Convert::getClosestOdom(const ros::Time& packet_time){
      nav_msgs::Odometry temp_odom;
      temp_odom.header.stamp = packet_time;
      if(odom_sorted_.empty()){
          return odom_sorted_.end();
      }
      
      std::vector<nav_msgs::Odometry>::iterator it = std::lower_bound(odom_sorted_.begin(), odom_sorted_.end(), temp_odom, timecomparison());
      if(it != odom_sorted_.end()){
          std::vector<nav_msgs::Odometry>::iterator larger_time_it = it;
          std::vector<nav_msgs::Odometry>::iterator smaller_time_it = it-1;
          if((larger_time_it->header.stamp - packet_time) <= (packet_time - smaller_time_it->header.stamp)){
              return larger_time_it;
          } else{
              return smaller_time_it;
          }
      } else {
         return it;
      }
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
    int current_num_points = accumulated_cloud_.points.size();
    for (size_t i = 0; i < scanMsg->packets.size(); ++i)
    {
      section_angle_ += data_->unpack(scanMsg->packets[i], *outMsg);
      int points_received = accumulated_cloud_.points.size() + outMsg->points.size()-current_num_points;
      std::pair<ros::Time, int> timestamp_num_points(scanMsg->packets[i].stamp, points_received);
      time_stamps_.push_back(timestamp_num_points);
      current_num_points = accumulated_cloud_.points.size() + outMsg->points.size();
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
      deskewPoints(scanMsg->header.stamp);
      output_.publish(accumulated_cloud_);
      section_angle_ = 0.0;
      accumulated_cloud_.points.clear();
      accumulated_cloud_.width = 0;
      time_stamps_.clear();
    }             
  }
  
  void Convert::deskewPoints(ros::Time pointcloud_timestamp)
  {
      for(std::vector< std::pair<ros::Time, int> >::iterator it = time_stamps_.begin(); it != time_stamps_.end(); ++it){
          //ROS_INFO_STREAM(" packet_time = " << it->first << ", points in packet = " << it->second);
          /*if(!odom_sorted_.empty()){
              std::vector<nav_msgs::Odometry>::iterator nit = getClosestOdom(it->first);
              if(nit == odom_sorted_.end()){
                  --nit;
              }
              ROS_INFO_STREAM("Packet timestamp = " << pointcloud_timestamp <<
               " Odom_closest: " << nit->header.stamp <<
               ", px: " << nit->pose.pose.position.x << 
               ", py: " << nit->pose.pose.position.y << 
               ", pz: " << nit->pose.pose.position.z << 
               ", qx: " << nit->pose.pose.orientation.x << 
               ", qy: " << nit->pose.pose.orientation.y << 
               ", qy: " << nit->pose.pose.orientation.z <<
               ", qw: " << nit->pose.pose.orientation.w);
          }*/
      }
      /*if(!odom_sorted_.empty()){
          std::vector<nav_msgs::Odometry>::iterator it = getClosestOdom(pointcloud_timestamp);
          if(it == odom_sorted_.end()){
              --it;
          }
          ROS_INFO_STREAM("PtCloud timestamp = " << pointcloud_timestamp <<
           " Odom_closest: " << it->header.stamp <<
           ", px: " << it->pose.pose.position.x << 
           ", py: " << it->pose.pose.position.y << 
           ", pz: " << it->pose.pose.position.z << 
           ", qx: " << it->pose.pose.orientation.x << 
           ", qy: " << it->pose.pose.orientation.y << 
           ", qy: " << it->pose.pose.orientation.z <<
           ", qw: " << it->pose.pose.orientation.w);
      }*/
      
      //debugPrintOdom();
  }

} // namespace velodyne_pointcloud
