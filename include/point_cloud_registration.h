#ifndef POINT_CLOUD_REGISTRATION_H_
#define POINT_CLOUD_REGISTRATION_H_
#include "pointmatcher/PointMatcher.h"
#include <cassert>
#include <iostream>
#include <vector>
#include "boost/filesystem.hpp"
#include <sensor_msgs/PointCloud2.h>
#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include "mkp_pcd/registration_clear.h"
#include "mkp_pcd/registration_merge_and_output.h"
#include "mkp_pcd/registration_process_registration.h"
#include "mkp_pcd/registration_take_view.h"
#include "pointmatcher_ros/point_cloud.h"

typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;

namespace mkp_pcd_registration
{
class pcd_registration
{
private:
  ros::NodeHandle n_;
  ros::ServiceServer pcd_registration_take_view;
  ros::ServiceServer pcd_registration_process_registration;
  ros::ServiceServer pcd_registration_merge_and_output;
  ros::ServiceServer pcd_registration_clear_all;
  ros::Subscriber pointcloud_subscriber_m;
  ros::Publisher pub_merge_point_;
  unsigned int queue_size_;
  PM::TransformationParameters global;
  PM::ICP icp;
  DP buffer_cloud;



  bool takeView(mkp_pcd::registration_take_view::Request& req, mkp_pcd::registration_take_view::Response& res);
  std::vector<DP> Dps_saved_from_input; // take_view

  bool LoadRequest(mkp_pcd::registration_process_registration::Request& req, mkp_pcd::registration_process_registration::Response& res);
  std::vector<int> index_to_be_selected_to_registration;

  void PCDprocessRegistration(int type); // inside of the LoadRequest once it sort the order index
  std::vector<DP> Dps_saved_from_process_registration;
  int reg_index;


  bool MergePoint(mkp_pcd::registration_merge_and_output::Request& req, mkp_pcd::registration_merge_and_output::Response& res);
  std::vector<DP> pcd_reg_merged_storing;

  bool Clear(mkp_pcd::registration_clear::Request& req, mkp_pcd::registration_clear::Response& res);

  void pointcloudsubscriberCallback(const sensor_msgs::PointCloud2ConstPtr &cloud);


public:
     pcd_registration();
     void spin();
     ~pcd_registration();

     typedef boost::function<bool (mkp_pcd::registration_take_view::Request& req, mkp_pcd::registration_take_view::Response& res)>
               registration_take_view_t;

     typedef boost::function<bool (mkp_pcd::registration_process_registration::Request& req, mkp_pcd::registration_process_registration::Response& res)>
               registration_process_registration_t;

     typedef boost::function<bool (mkp_pcd::registration_merge_and_output::Request& req, mkp_pcd::registration_merge_and_output::Response& res)>
               registration_merge_and_output_t;

     typedef boost::function<bool (mkp_pcd::registration_clear::Request& req, mkp_pcd::registration_clear::Response& res)>
               registration_clear_t;

     typedef boost::function<void (const sensor_msgs::PointCloud2ConstPtr &cloud)>
               point_cloud_subscribe_t;


};

}


#endif
