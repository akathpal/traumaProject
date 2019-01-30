#include <iostream>
#include <vector>
#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include "ros/ros.h"
#include <queue>
#include <vector>
#include <fstream>
#include <sstream> // need to add this to avoid error of loading file when compiled
#include <iostream>
#include <algorithm> // pt.indices.assign
#include <ros/package.h>
#include <time.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <string>
#include <std_msgs/Float64.h>
#include "mkp_pcd/FindWidthOrHeight.h"
#include <algorithm>  // std::min_element, std::max_element

pcl::PointCloud <pcl::PointXYZRGB> pointcloud_in;
double _PhantomWidth = 0;

void pcl_callback(const sensor_msgs::PointCloud2ConstPtr cloud) {
	//pcl::fromROSMsg(*cloud, pointcloud_out);
	pcl::fromROSMsg(*cloud, pointcloud_in);
}

bool segmentAndFindWidth(mkp_pcd::FindWidthOrHeight::Request& req, mkp_pcd::FindWidthOrHeight::Response& res)
{
  pcl::search::Search <pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> > (new pcl::search::KdTree<pcl::PointXYZRGB>);

  pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud <pcl::PointXYZRGB>);
  /*
  if ( pcl::io::loadPCDFile <pcl::PointXYZRGB> ("resultView_in_color_new.pcd", *cloud) == -1 )
  {
    std::cout << "Cloud reading failed." << std::endl;
    return (-1);
  }
  */
  cloud = pointcloud_in.makeShared();
  pcl::IndicesPtr indices (new std::vector <int>);
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.2,1.0);
  pass.filter (*indices);

  pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
  reg.setInputCloud (cloud);
  reg.setIndices (indices);
  reg.setSearchMethod (tree);
  reg.setDistanceThreshold (0.05);
  reg.setPointColorThreshold (6);
  reg.setRegionColorThreshold (5);
  reg.setMinClusterSize (600);

  std::vector <pcl::PointIndices> clusters;
  reg.extract (clusters);

  std::cout << "Number of clusters is equal to " << clusters.size () << std::endl; // expect to be 5
	for(int i=0;i<clusters.size (); i++)
	{
	std::cout << "First cluster has " << clusters[i].indices.size () << " points." << endl;
  }
  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();

/*
	pcl::visualization::CloudViewer viewer ("Cluster viewer");
  viewer.showCloud (colored_cloud);

  while (!viewer.wasStopped ())
  {
    boost::this_thread::sleep (boost::posix_time::microseconds (100));
  }
*/
	std::vector<double> y_components;
	// consider the first cluster will be the one we need.

	// find the largest data sets in the
	int max_region = 0;
	int points_in_max_region_ = 0;
	for(int i=0; i< clusters.size ();i++)
	{
		if(clusters[i].indices.size() > points_in_max_region_)
		{
			 points_in_max_region_ = clusters[i].indices.size() ;
			 max_region = i;
		}


	}
	ROS_INFO_STREAM("max_region= " << max_region);
	ROS_INFO_STREAM("points_in_max_region_= " << points_in_max_region_);

	for(int i=0; i< clusters[max_region].indices.size ();i++)
	{
		y_components.push_back(cloud->points[clusters[max_region].indices[i]].y);
	}

	double y_max = *std::max_element(y_components.begin(),y_components.end());
	double y_min = *std::min_element(y_components.begin(),y_components.end());
	res.distance = y_max-y_min;
	_PhantomWidth = (y_max-y_min)*1.15;
	return true;


}


int main(int argc,char**argv){
  ros::init(argc, argv, "segmentAndFindWidth");
  ros::NodeHandle n_;
  ros::Subscriber sub_pcl_save = n_.subscribe("mkp_pcd/pointCloudToRobotFrame",
          1000, pcl_callback);
	ros::Publisher pub = n_.advertise<std_msgs::Float64> ("mkp_pcd/PhantomWidth", 1);
  ros::ServiceServer FindPhantomWidth = n_.advertiseService(
           "mkp_pcd/PhantomWidth", segmentAndFindWidth);
/*  ros::ServiceServer FindPhantomHeight = n_.advertiseService(
           "mkp_pcd/FindPhantomHeight", segmentAndFindHeight);;
      */
  ros::Rate loop_rate(10);
  //ros::spin();
	while(ros::ok())
  {
		std_msgs::Float64 print_;
		print_.data= _PhantomWidth;
    pub.publish (print_);
    ros::spinOnce();
    loop_rate.sleep();
  }

}
