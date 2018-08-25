#include "point_cloud_targeting.h"
#include <ros/ros.h>

namespace mkp_pcd_targeting
{

void pcd_targeting::pointcloudsubscriberCallback(const sensor_msgs::PointCloud2ConstPtr &cloud)
{
   pcl::fromROSMsg(*cloud, pointcloud_in_XYZ);
}

bool pcd_targeting::pointcloudtargetingService(mkp_pcd::targeting::Request& req, mkp_pcd::targeting::Response& res)
{
  point_buffer_usedInAtMethod = pointcloud_in_XYZ.at(req.img_x , req.img_y);
  res.pcd_x = point_buffer_usedInAtMethod.x;
  res.pcd_y = point_buffer_usedInAtMethod.y;
  res.pcd_z = point_buffer_usedInAtMethod.z;
  return true;
}



pcd_targeting::pcd_targeting(): queue_size_(1000)
{

  targeting_server_t point_cloud_targeting_service =
        boost::bind(&pcd_targeting::pointcloudtargetingService, this, _1, _2);
  pcd_targeing_server = n_.advertiseService("mkp_pcd/pcd_targeting", point_cloud_targeting_service);

  point_cloud_subscribe_t point_cloud_subscriber_callback_m =
        boost::bind(&pcd_targeting::pointcloudsubscriberCallback, this, _1);
  pointcloud_subscriber_m = n_.subscribe("TBD", queue_size_, point_cloud_subscriber_callback_m);


}


void pcd_targeting::spin()
{
  ros::spin();
}
pcd_targeting::~pcd_targeting()
{

}










}
