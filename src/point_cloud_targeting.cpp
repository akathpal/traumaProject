#include "point_cloud_targeting.h"
#include <ros/ros.h>

namespace mkp_pcd_targeting
{




void pcd_targeting::pointcloudsubscriberCallback(const sensor_msgs::PointCloud2ConstPtr &cloud)
{
   pcl::fromROSMsg(*cloud, pointcloud_in_XYZ);
/*   ROS_INFO_STREAM("height: " << pointcloud_in_XYZ.height );
   ROS_INFO_STREAM("width: "<< pointcloud_in_XYZ.width);
   ROS_INFO_STREAM("-------------------------------------------");*/
   pcl_ros::transformPointCloud(pointcloud_in_XYZ, pointcloud_out_XYZ, transform_tf);
   ROS_INFO_STREAM("height: " << pointcloud_out_XYZ.height );
   ROS_INFO_STREAM("width: "<< pointcloud_out_XYZ.width);
   ROS_INFO_STREAM("*******************************************");
}



bool pcd_targeting::FindthefourFastScan(mkp_pcd::trigger::Request& req, mkp_pcd::trigger::Response& res)
{
  ROS_INFO_STREAM("pointcloud_out_XYZ.width= " <<   pointcloud_out_XYZ.width);
  ROS_INFO_STREAM("pointcloud_out_XYZ.height= " <<  pointcloud_out_XYZ.height );
//   int c = 114;
//   int r = 132;

//   point_buffer_usedInAtMethod = pointcloud_out_XYZ.at(c , r);
  pcl::PointCloud<pcl::PointXYZRGB> average_points;
  int pointCloudWindowSize=5;
  for(int i=-pointCloudWindowSize;i<pointCloudWindowSize;i++)
  {
    for(int j=-pointCloudWindowSize;j<pointCloudWindowSize;j++)
    {
      //c= 396+i;
      //r= 266+j;
      point_buffer_usedInAtMethod = pointcloud_out_XYZ.at(c , r);
      if(!std::isnan(point_buffer_usedInAtMethod.x) &&
         !std::isnan(point_buffer_usedInAtMethod.y) &&
         !std::isnan(point_buffer_usedInAtMethod.z))
      {
        ROS_INFO("got some");
        average_points.push_back(point_buffer_usedInAtMethod);
      }
    }
  }
  int sizePoint=average_points.size();
  double avg_x=0, avg_y=0, avg_z=0;

  for(int k=0;k < sizePoint;k++)
  {
   avg_x = avg_x + average_points[k].x;
   avg_y = avg_y + average_points[k].y;
   avg_z = avg_z + average_points[k].z;
  }

  ultra_bb.linear.x = avg_x/sizePoint;
  ultra_bb.linear.y = avg_y/sizePoint;
  ultra_bb.linear.z = avg_z/sizePoint;

  KDL::Vector BB_phantom(ultra_bb.linear.x, ultra_bb.linear.y, ultra_bb.linear.z);
  ROS_INFO_STREAM("ultra_bb.linear.x= " << ultra_bb.linear.x );
  ROS_INFO_STREAM("ultra_bb.linear.y= " << ultra_bb.linear.y );
  ROS_INFO_STREAM("ultra_bb.linear.z= " << ultra_bb.linear.z );
  //TODO Make the average grabbing point cloud
/*   geometry_msgs::Twist return___;
  return___.linear.x = point_buffer_usedInAtMethod.x;
  return___.linear.y = point_buffer_usedInAtMethod.y;
  return___.linear.z = point_buffer_usedInAtMethod.z;

  ROS_INFO_STREAM("point_buffer_usedInAtMethod.x= " << point_buffer_usedInAtMethod.x );
  ROS_INFO_STREAM("point_buffer_usedInAtMethod.y= " << point_buffer_usedInAtMethod.y );
  ROS_INFO_STREAM("point_buffer_usedInAtMethod.z= " << point_buffer_usedInAtMethod.z );
  ultra_bb.linear.x = point_buffer_usedInAtMethod.x;
  ultra_bb.linear.y = point_buffer_usedInAtMethod.y;
  ultra_bb.linear.z = point_buffer_usedInAtMethod.z;



  KDL::Vector BB_phantom(ultra_bb.linear.x, ultra_bb.linear.y, ultra_bb.linear.z);
*/
  //KDL::Vector BB_phantom(1,1,1);


  KDL::Vector T1(-0.137, 0.01, 0.006);
  KDL::Vector T2(0.125, -0.03, 0.0016);
  KDL::Vector T4(0.107, 0.084, 0.158);
  KDL::Vector T5(0.115, 0.0775, -0.156);
  KDL::Vector BB_atlas(0.001, 0.0, 0.00015);

  KDL::Rotation Ro(1,0,0,0,0,1,0,-1,0);

  double atlas_l= 0.19, atlas_w = 0.29, atlas_h = 0.21;
  double phantom_l= 0.16, phantom_w = Phantom_width_, phantom_h = Phantom_height_;

  T1 = Ro * T1;
  ROS_INFO_STREAM("T1= " << T1.data[0] << " " << T1.data[1] << " " << T1.data[2] );
  T2 = Ro * T2;
  ROS_INFO_STREAM("T2= " << T2.data[0] << " " << T2.data[1] << " " << T2.data[2] );
  T4 = Ro * T4;
  ROS_INFO_STREAM("T4= " << T4.data[0] << " " << T4.data[1] << " " << T4.data[2] );
  T5 = Ro * T5;
  ROS_INFO_STREAM("T5= " << T5.data[0] << " " << T5.data[1] << " " << T5.data[2] );
  BB_atlas = Ro * BB_atlas;
  ROS_INFO_STREAM("BB_atlas= " << BB_atlas.data[0] << " " << BB_atlas.data[1] << " " << BB_atlas.data[2] );

  KDL::Vector T1_to_BB = T1 - BB_atlas;
  ROS_INFO_STREAM("T1_to_BB= " << T1_to_BB.data[0] << " " << T1_to_BB.data[1] << " " << T1_to_BB.data[2] );
  KDL::Vector T2_to_BB = T2 - BB_atlas;
  ROS_INFO_STREAM("T2_to_BB= " << T2_to_BB.data[0] << " " << T2_to_BB.data[1] << " " << T2_to_BB.data[2] );
  KDL::Vector T4_to_BB = T4 - BB_atlas;
  ROS_INFO_STREAM("T4_to_BB= " << T4_to_BB.data[0] << " " << T4_to_BB.data[1] << " " << T4_to_BB.data[2] );
  KDL::Vector T5_to_BB = T5 - BB_atlas;
  ROS_INFO_STREAM("T5_to_BB= " << T5_to_BB.data[0] << " " << T5_to_BB.data[1] << " " << T5_to_BB.data[2] );

  ROS_INFO_STREAM("(phantom_l / atlas_l)= " << (phantom_l / atlas_l) );
  ROS_INFO_STREAM("(phantom_w / atlas_w)= " << (phantom_w / atlas_w) );
  ROS_INFO_STREAM("(phantom_h / atlas_h)= " << (phantom_h / atlas_h) );
  T1_to_BB.data[0] = T1_to_BB.data[0] * (phantom_l / atlas_l);
  T1_to_BB.data[1] = T1_to_BB.data[1] * (phantom_w / atlas_w);
  T1_to_BB.data[2] = T1_to_BB.data[2] * (phantom_h / atlas_h);
  ROS_INFO_STREAM("T1_to_BB.data[0]= " << T1_to_BB.data[0] );
  ROS_INFO_STREAM("T1_to_BB.data[1]= " << T1_to_BB.data[1] );
  ROS_INFO_STREAM("T1_to_BB.data[2]= " << T1_to_BB.data[2] );

  T2_to_BB.data[0] = T2_to_BB.data[0] * (phantom_l / atlas_l);
  T2_to_BB.data[1] = T2_to_BB.data[1] * (phantom_w / atlas_w);
  T2_to_BB.data[2] = T2_to_BB.data[2] * (phantom_h / atlas_h);

  T4_to_BB.data[0] = T4_to_BB.data[0] * (phantom_l / atlas_l);
  T4_to_BB.data[1] = T4_to_BB.data[1] * (phantom_w / atlas_w);
  T4_to_BB.data[2] = T4_to_BB.data[2] * (phantom_h / atlas_h);

  T5_to_BB.data[0] = T5_to_BB.data[0] * (phantom_l / atlas_l);
  T5_to_BB.data[1] = T5_to_BB.data[1] * (phantom_w / atlas_w);
  T5_to_BB.data[2] = T5_to_BB.data[2] * (phantom_h / atlas_h);

  T1_to_BB = BB_phantom + T1_to_BB;
  ROS_INFO_STREAM("T1_to_BB.data[0]= " << T1_to_BB.data[0] );
  ROS_INFO_STREAM("T1_to_BB.data[1]= " << T1_to_BB.data[1] );
  ROS_INFO_STREAM("T1_to_BB.data[2]= " << T1_to_BB.data[2] );
  T2_to_BB = BB_phantom + T2_to_BB;
  T4_to_BB = BB_phantom + T4_to_BB;
  T5_to_BB = BB_phantom + T5_to_BB;



  F1_.linear.x = T1_to_BB.data[0];
  F1_.linear.y = T1_to_BB.data[1];
  F1_.linear.z = T1_to_BB.data[2];
  ROS_INFO_STREAM("F1_.linear.x= " << F1_.linear.x );
  ROS_INFO_STREAM("F1_.linear.y= " << F1_.linear.y );
  ROS_INFO_STREAM("F1_.linear.z= " << F1_.linear.z );
  F2_.linear.x = T2_to_BB.data[0];
  F2_.linear.y = T2_to_BB.data[1];
  F2_.linear.z = T2_to_BB.data[2];
  ROS_INFO_STREAM("F2_.linear.x= " << F2_.linear.x );
  ROS_INFO_STREAM("F2_.linear.y= " << F2_.linear.y );
  ROS_INFO_STREAM("F2_.linear.z= " << F2_.linear.z );
  F4_.linear.x = T4_to_BB.data[0];
  F4_.linear.y = T4_to_BB.data[1];
  F4_.linear.z = T4_to_BB.data[2];
  ROS_INFO_STREAM("F4_.linear.x= " << F4_.linear.x );
  ROS_INFO_STREAM("F4_.linear.y= " << F4_.linear.y );
  ROS_INFO_STREAM("F4_.linear.z= " << F4_.linear.z );
  F5_.linear.x = T5_to_BB.data[0];
  F5_.linear.y = T5_to_BB.data[1];
  F5_.linear.z = T5_to_BB.data[2];
  ROS_INFO_STREAM("F5_.linear.x= " << F5_.linear.x );
  ROS_INFO_STREAM("F5_.linear.y= " << F5_.linear.y );
  ROS_INFO_STREAM("F5_.linear.z= " << F5_.linear.z );
  return true;
}

void pcd_targeting::pub_fk()
{

  ros::spinOnce();
}




// get the twis, which is the bb's location from Anirudh and grab the point cloud location.
void pcd_targeting::bbdetection(const geometry_msgs::TwistConstPtr &twis)
{
    c = twis->linear.x;
    r = twis->linear.y;

   //------------------------------
   //bbPoint.publish(return___); // publish "mkp_pcd/bb_location" topic
   //FastScanpub.publish();

}

bool pcd_targeting::pointcloudtargetingService(mkp_pcd::targeting::Request& req, mkp_pcd::targeting::Response& res)
{

  point_buffer_usedInAtMethod = pointcloud_in_XYZ.at(req.img_x , req.img_y);
  res.pcd_x = point_buffer_usedInAtMethod.x;
  res.pcd_y = point_buffer_usedInAtMethod.y;
  res.pcd_z = point_buffer_usedInAtMethod.z;
  return true;
}

  void pcd_targeting::phantom_h_(const std_msgs::Float64ConstPtr &ultra_h)
  {
    Phantom_height_ = ultra_h->data;
  }

  void pcd_targeting::phantom_w_(const std_msgs::Float64ConstPtr &ultra_w)
  {
    Phantom_width_ = ultra_w->data;
  }


pcd_targeting::pcd_targeting(): queue_size_(1000)
{

  targeting_server_t point_cloud_targeting_service =
        boost::bind(&pcd_targeting::pointcloudtargetingService, this, _1, _2);
  pcd_targeing_server = n_.advertiseService("mkp_pcd/pcd_targeting", point_cloud_targeting_service);

  point_cloud_subscribe_t point_cloud_subscriber_callback_m =
        boost::bind(&pcd_targeting::pointcloudsubscriberCallback, this, _1);
  pointcloud_subscriber_m = n_.subscribe("camera/depth_registered/points", queue_size_, point_cloud_subscriber_callback_m);



//mkp_pcd/pointCloudToRobotFrame
//camera/depth_registered/points

  bb_t bbt__ = boost::bind(&pcd_targeting::bbdetection, this, _1);
  bbsub__= n_.subscribe("/xybb", queue_size_, bbt__);

  ultra_h hh = boost::bind(&pcd_targeting::phantom_h_, this, _1);
  Height_= n_.subscribe("/mkp_pcd/PhantomHeight", queue_size_, hh);

  ultra_w ww = boost::bind(&pcd_targeting::phantom_w_, this, _1);
  Width_= n_.subscribe("/mkp_pcd/PhantomWidth", queue_size_, ww);

  Fast__Scan__ find_scan= boost::bind(&pcd_targeting::FindthefourFastScan , this, _1, _2);
  FindfastScan = n_.advertiseService("mkp_pcd/pcd_find_fastscan", find_scan);

  bbPoint = n_.advertise<geometry_msgs::Twist> ("mkp_pcd/bb_location", 1000);

  FastScanpub1 = n_.advertise<geometry_msgs::Twist> ("FScan3", 1000);
  FastScanpub2 = n_.advertise<geometry_msgs::Twist> ("FScan4", 1000);
  FastScanpub4 = n_.advertise<geometry_msgs::Twist> ("FScan1", 1000);
  FastScanpub5 = n_.advertise<geometry_msgs::Twist> ("FScan2", 1000);

  ultra_bb.linear.x = 0.0;
  ultra_bb.linear.y = 0.0;
  ultra_bb.linear.z = 0.0;
  c = 0;
  r = 0;
  height_ = 0.0;
  width_ = 0.0;
}


void pcd_targeting::spin()
{
  //ros::spin();
  ros::Rate loop_rate(10);
  ros::Duration(5.0).sleep();
  while(ros::ok())
  {
    tf::StampedTransform transform_;
    try{
       listener.lookupTransform("world", "tf_iiwa_FK",
                                ros::Time(0), transform_);
       //ROS_INFO("pass101");
       transform_tf.setOrigin(transform_.getOrigin());
       //ROS_INFO("pass103");
       transform_tf.setRotation(transform_.getRotation());

     }
     catch (tf::TransformException ex){
       ROS_ERROR("%s",ex.what());

     }
     FastScanpub1.publish(F1_);
     FastScanpub2.publish(F2_);
     FastScanpub4.publish(F4_);
     FastScanpub5.publish(F5_);
     ros::spinOnce();
     loop_rate.sleep();

  }
}
pcd_targeting::~pcd_targeting()
{

}










}
