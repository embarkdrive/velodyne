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
#include <nav_msgs/Odometry.h>
#include <velodyne_pointcloud/rawdata.h>

#include <dynamic_reconfigure/server.h>
#include <velodyne_pointcloud/CloudNodeConfig.h>

namespace velodyne_pointcloud
{
class timecomparison
{
  bool reverse;
public:
  timecomparison(const bool& revparam=false)
    {reverse=revparam;}
  bool operator() (const nav_msgs::Odometry& lhs, const nav_msgs::Odometry& rhs) const
  {
    ros::Duration diff = lhs.header.stamp - rhs.header.stamp;
    double diff_in_secs = diff.toSec();
    if (reverse) return (diff_in_secs > 0);
    else return (diff_in_secs < 0);
  }
};
    
  class Convert
  {
     static const int SIZE_OF_ODOM_LIST = 50;
  public:

    Convert(ros::NodeHandle node, ros::NodeHandle private_nh);
    ~Convert() {}

  private:
    
    void callback(velodyne_pointcloud::CloudNodeConfig &config,
                uint32_t level);
    void processScan(const velodyne_msgs::VelodyneScan::ConstPtr &scanMsg);
    void processOdom(const nav_msgs::Odometry::ConstPtr &odomMsg);
    std::vector<nav_msgs::Odometry>::iterator getClosestOdom(const ros::Time& packet_time);
    
    void deskewPoints(ros::Time pointcloud_timestamp);
    void debugPrintOdom();
    ///Pointer to dynamic reconfigure service srv_
    boost::shared_ptr<dynamic_reconfigure::Server<velodyne_pointcloud::
      CloudNodeConfig> > srv_;
    
    boost::shared_ptr<velodyne_rawdata::RawData> data_;
    ros::Subscriber velodyne_scan_;
    ros::Subscriber odom_sub_;
    ros::Publisher output_;

    //make the pointcloud container a member variable to append different slices
    velodyne_rawdata::VPointCloud accumulated_cloud_;
    float section_angle_;
    
    std::vector<nav_msgs::Odometry> odom_sorted_;
    std::vector< std::pair<ros::Time, int> > time_stamps_;
    
    /// configuration parameters
    typedef struct {
      int npackets;                    ///< number of packets to combine
    } Config;
    Config config_;
  };

} // namespace velodyne_pointcloud

#endif // _VELODYNE_POINTCLOUD_CONVERT_H_
