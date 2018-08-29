#include <ros/ros.h>
#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <ros/package.h>
#include "mkp_pcd/trigger.h"

typedef pcl::PointXYZRGBA PointColor;
typedef pcl::PointCloud<PointColor> PointCloudColor;
PointCloudColor pointcloud_in_color ,pointcloud_out_color;


void color_point_callback(const sensor_msgs::PointCloud2ConstPtr &input)
{
	pcl::fromROSMsg(*input, pointcloud_in_color);
  std::stringstream ss;
	ss << ros::package::getPath("mkp_pcd") << "/resultView_in_color" << ".pcd";
  pcl::io::savePCDFile (ss.str (), pointcloud_in_color, true);

}

bool point_cloud_save(mkp_pcd::trigger::Request& req, mkp_pcd::trigger::Response& res)
{
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "color_point_cloud_save");
  ros::NodeHandle n_;
  ros::Subscriber pointcloudsubscribe = n_.subscribe("mkp_pcd/registration_result",1000,
                                                       color_point_callback);
  ros::ServiceServer save_ = n_.advertiseService(
           "mkp_pcd/save_registration_result_as_color", point_cloud_save);
	ros::Rate loop_rate(10);
  ros::spin();

}
