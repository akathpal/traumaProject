#include "point_cloud_targeting.h"

namespace mkp_pcd_targeting
{

void pcd_targeing::pointcloudsubscriberCallback(const sensor_msgs::PointCloud2ConstPtr cloud)
{
   pcl::fromROSMsg(*cloud, pointcloud_in_XYZ);
}

bool pointcloudtargetingService(mkp_pcd::targeting::Request& req, mkp_pcd::targeting::Response& res)
{
  point_buffer_usedInAtMethod = pointcloud_in_XYZ.at(req.img_x , req.img_y);
  res.pcd_x = point_buffer_usedInAtMethod.x;
  res.pcd_y = point_buffer_usedInAtMethod.y;
  res.pcd_z = point_buffer_usedInAtMethod.z;
  return true;
}



pcd_targeing::pcd_targeing():queue_size_(1000)
{
  point_cloud_subscribe_t point_cloud_subscriber_callback =
        boost::bind(&pcd_targeing::pointcloudsubscriberCallback, this, _1);
  point_cloud_subscribe = n_.subscribe("....", queue_size_, point_cloud_subscriber_callback);

  targeting_server_t point_cloud_targeting_service =
        boost::bind(&pcd_targeing::pointcloudtargetingService, this, _1, _2);
  targeting_server = n_.advertiseService("mkp_pcd/pcd_targeting", point_cloud_targeting_service);

}


void pcd_targeting::spin()
{
  ros::spin();
}
pcd_targeting::~pcd_targeting()
{

}










}
