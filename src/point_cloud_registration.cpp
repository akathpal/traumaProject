#include "point_cloud_registration.h"
#include <ros/ros.h>

namespace mkp_pcd_registration
{
  pcd_registration::pcd_registration(): queue_size_(1000)
  {
    // Create the default ICP algorithm
    	icp.setDefault();

      registration_take_view_t pcd_registration_take_view_service =
            boost::bind(&pcd_registration::takeView, this, _1, _2);
      pcd_registration_take_view = n_.advertiseService("mkp_pcd/pcd_registration_take_view",
                                    pcd_registration_take_view_service);

      registration_process_registration_t LoadRequest_service =
            boost::bind(&pcd_registration::LoadRequest, this, _1, _2);
      pcd_registration_process_registration = n_.advertiseService("mkp_pcd/pcd_registration_process_registration",
                                                    LoadRequest_service);

      registration_merge_and_output_t registration_merge_and_output_service =
            boost::bind(&pcd_registration::MergePoint, this, _1, _2);
      pcd_registration_merge_and_output = n_.advertiseService("mkp_pcd/pcd_registration_merge_and_output",
                                                               registration_merge_and_output_service);

      registration_clear_t pcd_registration_clear_all_service =
            boost::bind(&pcd_registration::Clear, this, _1, _2);
      pcd_registration_clear_all = n_.advertiseService("mkp_pcd/pcd_registration_clear_all_service",
                                                                pcd_registration_clear_all_service);

      point_cloud_subscribe_t point_cloud_subscriber_callback_m =
            boost::bind(&pcd_registration::pointcloudsubscriberCallback, this, _1);
      pointcloud_subscriber_m = n_.subscribe("mkp_pcd/pointCloudToRobotFrame", queue_size_,
                                                                    point_cloud_subscriber_callback_m);
      pub_merge_point_=n_.advertise<sensor_msgs::PointCloud2> ("mkp_pcd/registration_result", 1000);
      reg_index = 0;
  }

//TODO 20180903 Michael: This can make a template for using different source, such as Dps_saved_from_input
  void pcd_registration::PCDprocessRegistration(int type)
  {
    global = Eigen::Matrix4f::Identity();
    std::cout << "index_to_be_selected_to_registration.size()= " << index_to_be_selected_to_registration.size() << std::endl;

    switch(type)
    {
      case 1:
      {
        Dps_saved_from_process_registration.push_back(Dps_saved_from_input[index_to_be_selected_to_registration[0]-1]);
        for(int i = 1;i<index_to_be_selected_to_registration.size();i++)
        {
          int be_tras_index = index_to_be_selected_to_registration[i];
          int ref_index = index_to_be_selected_to_registration[i-1];
        std::cout << "i= " << i << std::endl;
        // Compute the transformation that expresses data in ref
       // frame started from 1, so grabbing the point cloud need to be index_to_be_selected_to_registration[]-1;
        PM::TransformationParameters T = icp(Dps_saved_from_input[be_tras_index-1], Dps_saved_from_input[ref_index-1]);
        // Transform data to express it in ref
        DP data_out(Dps_saved_from_input[be_tras_index-1]);
        global= global*T;
        icp.transformations.apply(data_out, global);
        data_out.save(ros::package::getPath("mkp_pcd") + "/pointCloudFile_out" + std::to_string(reg_index) + ".pcd");
        reg_index=reg_index+1;
        Dps_saved_from_process_registration.push_back(data_out);
        }
        break;
      } // case 1
      case 2:
      {
        Dps_saved_from_process_registration.push_back(pcd_reg_merged_storing[index_to_be_selected_to_registration[0]-1]); // you need a base
        std::cout << "pcd_reg_merged_storing.size()= " << pcd_reg_merged_storing.size() << std::endl;
        for(int i = 1;i<index_to_be_selected_to_registration.size();i++)
        {
          int be_tras_index = index_to_be_selected_to_registration[i];
          int ref_index = index_to_be_selected_to_registration[i-1];
        std::cout << "i= " << i << std::endl;
        PM::TransformationParameters T = icp(pcd_reg_merged_storing[be_tras_index-1], pcd_reg_merged_storing[ref_index-1]);
        // Transform data to express it in ref
        DP data_out(pcd_reg_merged_storing[be_tras_index-1]);
        global= global*T;
        icp.transformations.apply(data_out, global);

        data_out.save(ros::package::getPath("mkp_pcd") + "/pointCloudFile_out" + std::to_string(i) + ".pcd");
        Dps_saved_from_process_registration.push_back(data_out);
        }

        // merge and output
        ROS_INFO("Entering MergePoint function");
        DP data_out_merge(Dps_saved_from_process_registration[0]);
        ROS_INFO("Pass");
        std::cout << "Dps_saved_from_process_registration.size()= " << Dps_saved_from_process_registration.size() << std::endl;
        for(int i=1;i < Dps_saved_from_process_registration.size();i++)
        {
          ROS_INFO("Pass");
          data_out_merge.concatenate(Dps_saved_from_process_registration[i]);
        }
        ROS_INFO("Pass_merge_loop");
        //transforms the point cloud to the ros message
        sensor_msgs::PointCloud2 buffer__ =
               PointMatcher_ros::pointMatcherCloudToRosMsg< float >	(data_out_merge, "world", ros::Time::now());
        //publish and spineonce.
        pub_merge_point_.publish(buffer__);
        ros::spinOnce();
        ROS_INFO("MergePoint function Done");

        break;
      }

    }// end switch

  }

  void pcd_registration::spin()
  {
    ros::spin();
  }
  pcd_registration::~pcd_registration()
  {

  }


bool pcd_registration::LoadRequest(mkp_pcd::registration_process_registration::Request& req, mkp_pcd::registration_process_registration::Response& res)
{
  // http://www.cplusplus.com/reference/vector/vector/assign/
  // Assigns new contents to the vector, replacing its current contents, and modifying its size accordingly.
  index_to_be_selected_to_registration.assign(req.indices.begin(),req.indices.end());
  switch(req.type)
  {
    case 1:
    {
    PCDprocessRegistration(req.type);
    break;
    }
    case 2:
    {
    PCDprocessRegistration(req.type);
    break;
    }
  }

 return true;
}


bool pcd_registration::takeView(mkp_pcd::registration_take_view::Request& req, mkp_pcd::registration_take_view::Response& res)
{
  Dps_saved_from_input.push_back(buffer_cloud);
  return true;
}

bool pcd_registration::MergePoint(mkp_pcd::registration_merge_and_output::Request& req, mkp_pcd::registration_merge_and_output::Response& res)
{
  ROS_INFO("Entering MergePoint function");
  DP data_out_merge(Dps_saved_from_process_registration[0]);
  for(int i=1;i < Dps_saved_from_process_registration.size();i++)
  {
    data_out_merge.concatenate(Dps_saved_from_process_registration[i]);
  }
  pcd_reg_merged_storing.push_back(data_out_merge);// 20180903 Michael store the result

  //transforms the point cloud to the ros message
  sensor_msgs::PointCloud2 buffer__ =
         PointMatcher_ros::pointMatcherCloudToRosMsg< float >	(data_out_merge, "world", ros::Time::now());
  //publish and spineonce.
  pub_merge_point_.publish(buffer__);
  ros::spinOnce();
  data_out_merge.save(ros::package::getPath("mkp_pcd") + "/pointCloudFile_out_merge.pcd");
  ROS_INFO("MergePoint function Done");
  Dps_saved_from_process_registration.clear();// 20180904 Michael

  return true;
}

bool pcd_registration::Clear(mkp_pcd::registration_clear::Request& req, mkp_pcd::registration_clear::Response& res)
{
  Dps_saved_from_input.clear();
  Dps_saved_from_process_registration.clear();
  pcd_reg_merged_storing.clear();
  reg_index = 0;
  return true;

}
void pcd_registration::pointcloudsubscriberCallback(const sensor_msgs::PointCloud2ConstPtr &cloud)
{
  buffer_cloud = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(*cloud);
}


}
