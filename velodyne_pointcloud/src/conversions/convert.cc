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
    deskew_(true),
    odom_spinner_(&node, ""),
    tf_available_(false)
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
    //ROS_DEBUG_STREAM(" Odom_rcvd: " << odomMsg->header.stamp << ", Now :" << ros::Time::now());
    boost::lock_guard<boost::mutex> lock(mutex_);
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
  
  std::vector<nav_msgs::Odometry>::iterator Convert::getClosestOdom(const ros::Time& packet_time, bool past_only = false){
      nav_msgs::Odometry temp_odom;
      temp_odom.header.stamp = packet_time;
      if(odom_sorted_.empty()){     // If no odometry msgs, return end
          return odom_sorted_.end();
      }
      
      std::vector<nav_msgs::Odometry>::iterator it = std::lower_bound(odom_sorted_.begin(), odom_sorted_.end(), temp_odom, timecomparison());
      if(past_only){  // If only past msgs are to be used,
          if(it == odom_sorted_.begin()){ // If next highest msg is the oldest in the queue
              return odom_sorted_.end();  // return end, indicating no closest in the past
          }
          return it-1; // Return the immediate past msg
      } else { // If both past and future msgs are to be used, find the closest
          if(it == odom_sorted_.begin()){
              return it;
          } else if (it == odom_sorted_.end()){
              return it-1;
          } else {  
              std::vector<nav_msgs::Odometry>::iterator larger_time_it = it;
              std::vector<nav_msgs::Odometry>::iterator smaller_time_it = it-1;
              if((larger_time_it->header.stamp - packet_time) <= (packet_time - smaller_time_it->header.stamp)){
                  return larger_time_it;
              } else{
                  return smaller_time_it;
              }
          }
      }
  }

  /** @brief Callback for raw scan messages. */
  void Convert::processScan(const velodyne_msgs::VelodyneScan::ConstPtr &scanMsg)
  {
    if (output_.getNumSubscribers() == 0)         // no one listening?
      return;                                     // avoid much work
      
    velodyne_rawdata::VPointCloud scan;
    
    // process each packet provided by the driver and save them
    for (size_t i = 0; i < scanMsg->packets.size(); ++i)
    {
        // scan's header is a pcl::PCLHeader, convert it before stamp assignment
        scan.header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
        scan.header.frame_id = scanMsg->header.frame_id;
        scan.height = 1;
        
      section_angle_ += data_->unpack(scanMsg->packets[i], scan);
      time_stamps_.push_back(scanMsg->packets[i].stamp);
      scans_.push_back(scan);
      //std::cout << "accumulated angle: " << section_angle_ << std::endl;
    }
    
    // Once we saved all packets for the last full 360 degrees, deskew them and publish
    if(section_angle_/100.0 >= 60.0)
    {
      accumulated_cloud_.header.stamp = scan.header.stamp;
      accumulated_cloud_.header.frame_id = scan.header.frame_id;
      accumulated_cloud_.height = scan.height;
      deskewPoints(scanMsg->header.stamp);
      accumulated_cloud_.width = accumulated_cloud_.points.size();
      output_.publish(accumulated_cloud_);
      //ROS_INFO("Size = %d", accumulated_cloud_.width);
      //ROS_INFO("Published");
      // Reset the buffers after publish
      section_angle_ = 0.0;
      accumulated_cloud_.points.clear();
      accumulated_cloud_.width = 0;
      time_stamps_.clear();
      scans_.clear();
    } 
  }
  
  void Convert::deskewPoints(ros::Time pointcloud_timestamp)
  {
      velodyne_rawdata::VPointCloud deskewed_cloud;
      tf::Transform packet_tf, pointcloud_tf, diff_tf;
      
      if(!tf_available_){
          try {
              tf_listener_.waitForTransform("/vehicle_frame", accumulated_cloud_.header.frame_id, ros::Time::now(), ros::Duration(0));
              tf_listener_.lookupTransform("/vehicle_frame", accumulated_cloud_.header.frame_id, ros::Time::now(), vehicle_to_velodyne_transform_);
          } catch (tf::TransformException ex){
              deskew_ = false;
              tf_available_ = false;
              ROS_WARN_THROTTLE(60, "vehicle to velodyne transform unavailable %s", ex.what());
          }
          if(deskew_){
              tf_available_ = true;
          }
      }
      
      // Get the odometry closest to the last packet timestamp (overall pointcloud timestamp) in the past
      std::vector<nav_msgs::Odometry>::iterator prev_odom_it = getClosestOdom(pointcloud_timestamp, true);
      
      if (odom_sorted_.size() <= 2 || prev_odom_it == odom_sorted_.end())
      {
          deskew_ = false;
          ROS_INFO_STREAM(" No deskew: " << pointcloud_timestamp << ", Now :" << ros::Time::now());
          debugPrintOdom();
      }
      if (deskew_){
          double time_diff = (pointcloud_timestamp - prev_odom_it->header.stamp).toSec();
          ROS_INFO("Time diff = %f", time_diff);
          nav_msgs::Odometry odom_at_timestamp;
          odom_at_timestamp.header.stamp = pointcloud_timestamp;
          odom_at_timestamp.pose.pose.position.x = prev_odom_it->pose.pose.position.x + prev_odom_it->twist.twist.linear.x * time_diff;
          odom_at_timestamp.pose.pose.position.y = prev_odom_it->pose.pose.position.y + prev_odom_it->twist.twist.linear.y * time_diff;
          odom_at_timestamp.pose.pose.position.z = prev_odom_it->pose.pose.position.z;

          double yaw = tf::getYaw(prev_odom_it->pose.pose.orientation);
          double updated_yaw = yaw + prev_odom_it->twist.twist.angular.z * time_diff;
          ROS_DEBUG("Yaw = %f, Updated Yaw = %f", yaw, updated_yaw);
          
          odom_at_timestamp.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0,0.0,updated_yaw);
          
          odom_at_timestamp.twist.twist.linear.x = prev_odom_it->twist.twist.linear.x;
          odom_at_timestamp.twist.twist.linear.y = prev_odom_it->twist.twist.linear.y;
          odom_at_timestamp.twist.twist.linear.z = prev_odom_it->twist.twist.linear.z;
          odom_at_timestamp.twist.twist.angular.x = prev_odom_it->twist.twist.angular.x;
          odom_at_timestamp.twist.twist.angular.y = prev_odom_it->twist.twist.angular.y;
          odom_at_timestamp.twist.twist.angular.z = prev_odom_it->twist.twist.angular.z;
          
          ROS_DEBUG_STREAM("Pointcloud timestamp = " << pointcloud_timestamp <<
           " Odom_closest: " << prev_odom_it->header.stamp <<
           ", px: " << odom_at_timestamp.pose.pose.position.x << 
           ", py: " << odom_at_timestamp.pose.pose.position.y << 
           ", pz: " << odom_at_timestamp.pose.pose.position.z << 
           ", qx: " << odom_at_timestamp.pose.pose.orientation.x << 
           ", qy: " << odom_at_timestamp.pose.pose.orientation.y << 
           ", qy: " << odom_at_timestamp.pose.pose.orientation.z <<
           ", qw: " << odom_at_timestamp.pose.pose.orientation.w);
           
           tf::Transform odom_tf;
           tf::poseMsgToTF(odom_at_timestamp.pose.pose, odom_tf);
           pointcloud_tf.mult(odom_tf, vehicle_to_velodyne_transform_);
           
           /*geometry_msgs::Pose pointcloud_pose;
           tf::poseTFToMsg(pointcloud_tf, pointcloud_pose);
           ROS_INFO_STREAM(
           "pointcloud pose = " <<
           "px: " << pointcloud_pose.position.x << 
           ", py: " << pointcloud_pose.position.y << 
           ", pz: " << pointcloud_pose.position.z << 
           ", qx: " << pointcloud_pose.orientation.x << 
           ", qy: " << pointcloud_pose.orientation.y << 
           ", qz: " << pointcloud_pose.orientation.z <<
           ", qw: " << pointcloud_pose.orientation.w);*/
      }
       for(std::vector<ros::Time>::iterator it = time_stamps_.begin(); it != time_stamps_.end(); ++it){
           int index = std::distance(time_stamps_.begin(), it);
           if(deskew_){
               std::vector<nav_msgs::Odometry>::iterator nit = getClosestOdom(*it);
                ROS_DEBUG_STREAM("Packet timestamp = " << *it <<
                 " Odom_closest: " << nit->header.stamp <<
                 ", px: " << nit->pose.pose.position.x << 
                 ", py: " << nit->pose.pose.position.y << 
                 ", pz: " << nit->pose.pose.position.z << 
                 ", qx: " << nit->pose.pose.orientation.x << 
                 ", qy: " << nit->pose.pose.orientation.y << 
                 ", qy: " << nit->pose.pose.orientation.z <<
                 ", qw: " << nit->pose.pose.orientation.w);
                tf::Transform odom_tf;
                tf::poseMsgToTF(nit->pose.pose, odom_tf);
                packet_tf.mult(odom_tf, vehicle_to_velodyne_transform_);
                
                /*geometry_msgs::Pose packet_pose;
                tf::poseTFToMsg(packet_tf, packet_pose);
                ROS_INFO_STREAM(
                "packet pose = " <<
                "px: " << packet_pose.position.x << 
                ", py: " << packet_pose.position.y << 
                ", pz: " << packet_pose.position.z << 
                ", qx: " << packet_pose.orientation.x << 
                ", qy: " << packet_pose.orientation.y << 
                ", qz: " << packet_pose.orientation.z <<
                ", qw: " << packet_pose.orientation.w);*/
                
                diff_tf = pointcloud_tf.inverseTimes(packet_tf);
                geometry_msgs::Pose diff_pose;
                tf::poseTFToMsg(diff_tf, diff_pose);
                /*ROS_INFO_STREAM(
                "diff pose = " <<
                "px: " << diff_pose.position.x << 
                ", py: " << diff_pose.position.y << 
                ", pz: " << diff_pose.position.z << 
                ", qx: " << diff_pose.orientation.x << 
                ", qy: " << diff_pose.orientation.y << 
                ", qz: " << diff_pose.orientation.z <<
                ", qw: " << diff_pose.orientation.w);*/
                
                pcl_ros::transformPointCloud(scans_[index], deskewed_cloud, diff_tf);
                /*ROS_INFO_STREAM(
                "  px: " << scans_[index].points[0].x << 
                ", py: " << scans_[index].points[0].y << 
                ", pz: " << scans_[index].points[0].z);
                ROS_INFO_STREAM(
                "  px: " << deskewed_cloud.points[0].x << 
                ", py: " << deskewed_cloud.points[0].y << 
                ", pz: " << deskewed_cloud.points[0].z);*/
           } else {
               deskewed_cloud = scans_[index];
           }
           //Accumulate the pt cloud
           accumulated_cloud_.points.insert(accumulated_cloud_.points.end(), deskewed_cloud.points.begin(), deskewed_cloud.points.end());
       }
  }
} // namespace velodyne_pointcloud
