#include <ros/ros.h>
#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <ros/package.h>
#include <pcl/filters/voxel_grid.h>
#include <boost/make_shared.hpp>
#include "mkp_pcd/trigger.h"

typedef pcl::PointXYZRGBA PointColor;
typedef pcl::PointCloud<PointColor> PointCloudColor;
PointCloudColor pointcloud_in_color, pointcloud_out_color;

void color_point_callback(const sensor_msgs::PointCloud2ConstPtr &input)
{
	pcl::fromROSMsg(*input, pointcloud_in_color);
	// Create the filtering object
  PointCloudColor::Ptr cloud;

	cloud = pointcloud_in_color.makeShared();
	pcl::VoxelGrid<PointColor> sor;
	sor.setInputCloud (cloud);
	sor.setLeafSize (0.01f, 0.01f, 0.01f);
	sor.filter (*cloud);
  std::stringstream ss;
	ss << ros::package::getPath("mkp_pcd") << "/resultView_in_color_new" << ".pcd";
  pcl::io::savePCDFile (ss.str (), *cloud, true);

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
