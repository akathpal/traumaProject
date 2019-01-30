


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
#include <mkp_pcd/trigger.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <algorithm> // pt.indices.assign
#include <kdl/frames.hpp>



namespace mkp_pcd_targeting
{
  class pcd_targeting
  {
  private:
  ros::NodeHandle n_;
  ros::ServiceServer pcd_targeing_server;
  ros::Subscriber pointcloud_subscriber_m; // subscribe the joint topics of iiwa
  ros::Publisher bbPoint;
  ros::Publisher FastScanpub1;
  ros::Publisher FastScanpub2;
  ros::Publisher FastScanpub4;
  ros::Publisher FastScanpub5;
  ros::Subscriber bbsub__;
  ros::Subscriber Height_;
  ros::Subscriber Width_;
  ros::ServiceServer FindfastScan;
  pcl::PointCloud<pcl::PointXYZRGB> pointcloud_in_XYZ;
  pcl::PointCloud<pcl::PointXYZRGB> pointcloud_out_XYZ;
  pcl::PointXYZRGB point_buffer_usedInAtMethod;




  void pointcloudsubscriberCallback(const sensor_msgs::PointCloud2ConstPtr &cloud);
  bool pointcloudtargetingService(mkp_pcd::targeting::Request& req, mkp_pcd::targeting::Response& res);
  void bbdetection(const geometry_msgs::TwistConstPtr  &twis);
  void phantom_h_(const std_msgs::Float64ConstPtr &ultra_h);
  void phantom_w_(const std_msgs::Float64ConstPtr &ultra_w);
  bool FindthefourFastScan(mkp_pcd::trigger::Request& req, mkp_pcd::trigger::Response& res);
  void pub_fk();

  unsigned int queue_size_;
  //-----------------------------------------
  geometry_msgs::Twist ultra_bb;
  double Phantom_height_;
  double Phantom_width_;
  tf::Pose transform_tf;
  tf::TransformListener listener;
  int c,r;
  double height_;
  double width_;
  geometry_msgs::Twist F1_,F2_,F5_,F4_;

  //-----------------------------------------
  public:

    pcd_targeting();
    void spin();
    ~pcd_targeting();

    typedef boost::function<bool (mkp_pcd::targeting::Request& req, mkp_pcd::targeting::Response& res)>
          targeting_server_t;

    typedef boost::function<void (const sensor_msgs::PointCloud2ConstPtr &cloud)>
          point_cloud_subscribe_t;

    typedef boost::function<void (const geometry_msgs::TwistConstPtr  &twis)>
                bb_t;

    typedef boost::function<void (const std_msgs::Float64ConstPtr &ultra_h)>
                ultra_h;

    typedef boost::function<void (const std_msgs::Float64ConstPtr &ultra_w)>
                ultra_w;

    typedef boost::function<bool (mkp_pcd::trigger::Request& req, mkp_pcd::trigger::Response& res)>
                Fast__Scan__;



  };


}


#endif
