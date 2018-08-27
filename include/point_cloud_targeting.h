


#ifndef __PCD_TARGETING_H__
#define __PCD_TARGETING_H__

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <mkp_pcd/targeting.h>
#include <algorithm> // pt.indices.assign



namespace mkp_pcd_targeting
{
  class pcd_targeting
  {
  private:
  ros::NodeHandle n_;
  ros::ServiceServer pcd_targeing_server;
  ros::Subscriber pointcloud_subscriber_m; // subscribe the joint topics of iiwa

  pcl::PointCloud<pcl::PointXYZ> pointcloud_in_XYZ;
  pcl::PointXYZ point_buffer_usedInAtMethod;

  void pointcloudsubscriberCallback(const sensor_msgs::PointCloud2ConstPtr &cloud);
  bool pointcloudtargetingService(mkp_pcd::targeting::Request& req, mkp_pcd::targeting::Response& res);
  unsigned int queue_size_;

  public:

    pcd_targeting();
    void spin();
    ~pcd_targeting();

    typedef boost::function<bool (mkp_pcd::targeting::Request& req, mkp_pcd::targeting::Response& res)>
          targeting_server_t;

    typedef boost::function<void (const sensor_msgs::PointCloud2ConstPtr &cloud)>
          point_cloud_subscribe_t;

  };


}


#endif
