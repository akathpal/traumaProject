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
#include <pcl/search/kdtree.h>
#include <std_msgs/Float64.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <boost/make_shared.hpp>
#include "mkp_pcd/trigger.h"

typedef pcl::PointXYZRGB PointColor;
typedef pcl::PointCloud<PointColor> PointCloudColor;
PointCloudColor pointcloud_in_color, pointcloud_out_color;
double _PhantomHeight = 0;

void segmentAndFindHeight(pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloudIn)
{
  pcl::search::Search <pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> > (new pcl::search::KdTree<pcl::PointXYZRGB>);
  //pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud <pcl::PointXYZRGB>);
/*
	if ( pcl::io::loadPCDFile <pcl::PointXYZRGB> ("resultView_in_color_new.pcd", *cloud) == -1 )
  {
    std::cout << "Cloud reading failed." << std::endl;
    return (-1);
  }
	*/
  pcl::IndicesPtr indices (new std::vector <int>);
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud (cloudIn);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.10,1.0);
  pass.filter (*indices);

  pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
  reg.setInputCloud (cloudIn);
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
  std::vector<double> z_components;
	// consider the first cluster will be the one we need.
  for(int i=0; i< clusters[max_region].indices.size ();i++)
	{
		z_components.push_back(colored_cloud->points[clusters[max_region].indices[i]].z);
	}

	double z_max = *std::max_element(z_components.begin(),z_components.end());
	double z_min = *std::min_element(z_components.begin(),z_components.end());
	_PhantomHeight = z_max-z_min;
}


void color_point_callback(const sensor_msgs::PointCloud2ConstPtr &input)
{
	pcl::fromROSMsg(*input, pointcloud_in_color);
	// Create the filtering object
  PointCloudColor::Ptr cloud;

	cloud = pointcloud_in_color.makeShared();
	/*pcl::VoxelGrid<PointColor> sor;
	sor.setInputCloud (cloud);
	sor.setLeafSize (0.01f, 0.01f, 0.01f);
	sor.filter (*cloud);
  */
  std::stringstream ss;
	ss << ros::package::getPath("mkp_pcd") << "/resultView_in_color_new" << ".pcd";
  pcl::io::savePCDFile (ss.str (), *cloud, true);
  segmentAndFindHeight(cloud);
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
  ros::Publisher pub = n_.advertise<std_msgs::Float64> ("mkp_pcd/PhantomHeight", 1);
	ros::Rate loop_rate(10);
  //ros::spin();
	while(ros::ok())
  {
    std_msgs::Float64 print_;
    print_.data= _PhantomHeight;
    pub.publish (print_);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
