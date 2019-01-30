#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <vector>
#include <fstream>
#include <std_msgs/Bool.h>
#include <sstream> // need to add this to avoid error of loading file when compiled
#include <iostream>
#include <math.h>
#include <mkp_pcd/movj.h>


typedef boost::function<bool (mkp_pcd::movj::Request& req, mkp_pcd::movj::Response& res)>
          mov_j_server_t;
typedef boost::function<void (const sensor_msgs::JointState::ConstPtr& joint_iiwa)>
          joint_feed_back_sub_t;

typedef boost::function<void (const std_msgs::BoolConstPtr& shut_in)>
          shut_down__;

bool shut_shut_down = false;
double joint1 = 0, joint2 = 0, joint3 = 0, joint4 = 0, joint5 = 0, joint6 = 0, joint7 = 0;
double d01 = 0, d02 = 0, d03 = 0, d04 = 0, d05 = 0, d06 = 0, d07 = 0;
trajectory_msgs::JointTrajectory msg5;
ros::Publisher cmd_pub; // send joint comd to iiwa
double threshold_follower_error = 0.1;

void jointFeedbackSubCallback(const sensor_msgs::JointState::ConstPtr& joint_iiwa)
  {
    joint1 = joint_iiwa->position[0];
    joint2 = joint_iiwa->position[1];
    joint3 = joint_iiwa->position[2];
    joint4 = joint_iiwa->position[3];
    joint5 = joint_iiwa->position[4];
    joint6 = joint_iiwa->position[5];
    joint7 = joint_iiwa->position[6];
  }

// This callback only send a request of a single set of joint; It is called by
// INT.cpp
bool movJServerCallback(mkp_pcd::movj::Request& req ,mkp_pcd::movj::Response& res)
{
  msg5.points[0].positions.resize(7);
  msg5.points[0].positions[0] = req.movj.positions[0];
  msg5.points[0].positions[1] = req.movj.positions[1];
  msg5.points[0].positions[2] = req.movj.positions[2];
  msg5.points[0].positions[3] = req.movj.positions[3];
  msg5.points[0].positions[4] = req.movj.positions[4];
  msg5.points[0].positions[5] = req.movj.positions[5];
  msg5.points[0].positions[6] = req.movj.positions[6];
  ROS_INFO_STREAM("Going to A Scan Position");
  cmd_pub.publish(msg5);
  //ros::spinOnce();
  //loop_rate.sleep();
  //ros::spinOnce();
  bool reached = false;
  while(!reached)
  {

         d01 = fabs(joint1-req.movj.positions[0]);
         d02 = fabs(joint2-req.movj.positions[1]);
         d03 = fabs(joint3-req.movj.positions[2]);
         d04 = fabs(joint4-req.movj.positions[3]);
         d05 = fabs(joint5-req.movj.positions[4]);
         d06 = fabs(joint6-req.movj.positions[5]);
         d07 = fabs(joint7-req.movj.positions[6]);

         ROS_INFO("joint1: %f ",joint1);
         ROS_INFO("joint2: %f ",joint2);
         ROS_INFO("joint3: %f ",joint3);
         ROS_INFO("joint4: %f ",joint4);
         ROS_INFO("joint5: %f ",joint5);
         ROS_INFO("joint6: %f ",joint6);
         ROS_INFO("joint7: %f ",joint7);

         ROS_INFO("req.movj.positions[0]: %f ",req.movj.positions[0]);
         ROS_INFO("req.movj.positions[1]: %f ",req.movj.positions[1]);
         ROS_INFO("req.movj.positions[2]: %f ",req.movj.positions[2]);
         ROS_INFO("req.movj.positions[3]: %f ",req.movj.positions[3]);
         ROS_INFO("req.movj.positions[4]: %f ",req.movj.positions[4]);
         ROS_INFO("req.movj.positions[5]: %f ",req.movj.positions[5]);
         ROS_INFO("req.movj.positions[6]: %f ",req.movj.positions[6]);

         ROS_INFO("d01: %f ",d01);
         ROS_INFO("d02: %f ",d02);
         ROS_INFO("d03: %f ",d03);
         ROS_INFO("d04: %f ",d04);
         ROS_INFO("d05: %f ",d05);
         ROS_INFO("d06: %f ",d06);
         ROS_INFO("d07: %f ",d07);
         ROS_INFO("*******************************************************");

         if (d01 < threshold_follower_error && d02 < threshold_follower_error && d03 < threshold_follower_error
         						&& d04 < threshold_follower_error && d05 < threshold_follower_error && d06 < threshold_follower_error
         						&& d07 < threshold_follower_error) {
         					reached = true;
         				}
        	ros::spinOnce();
  }
res.error_code = 0;
return true;

}

void shut_down(const std_msgs::BoolConstPtr& shut_in)
{
  shut_shut_down = shut_in->data;
}




  int main(int argc,char**argv){
    ros::init(argc, argv, "movJServerCallback");
    ros::NodeHandle n_;
    ros::ServiceServer mov_j_server; // provide services for mov_j
    ros::Subscriber joint_feed_back_sub;
    ros::Subscriber shut_down_sub;
    unsigned int queue_size_ = 1000;
    cmd_pub = n_.advertise<trajectory_msgs::JointTrajectory>("/iiwa/PositionJointInterface_trajectory_controller/command", 1000);

    shut_down__ shut_down___ = boost::bind(&shut_down, _1);
    shut_down_sub = n_.subscribe("mkp_pcd/shut_down", queue_size_, shut_down___);

    mov_j_server_t mov_j_server_callback = boost::bind(&movJServerCallback,_1, _2);
    mov_j_server = n_.advertiseService("mkp_pcd/movj", mov_j_server_callback);

    joint_feed_back_sub_t joint_feed_back_sub_callback = boost::bind(&jointFeedbackSubCallback, _1);
    joint_feed_back_sub = n_.subscribe("/iiwa/joint_states", queue_size_, joint_feed_back_sub_callback);



    msg5.header.seq = 0;       // remove
    msg5.header.stamp.sec = 0;  // ros time::now() Bharat
    msg5.header.stamp.nsec = 0;  // remove
    msg5.header.frame_id = "";
    msg5.joint_names.push_back("iiwa_joint_1");
    msg5.joint_names.push_back("iiwa_joint_2");
    msg5.joint_names.push_back("iiwa_joint_3");
    msg5.joint_names.push_back("iiwa_joint_4");
    msg5.joint_names.push_back("iiwa_joint_5");
    msg5.joint_names.push_back("iiwa_joint_6");
    msg5.joint_names.push_back("iiwa_joint_7");
    msg5.points.resize(1);
    msg5.points[0].velocities.resize(7);
    msg5.points[0].effort.resize(7);
			 for (size_t j = 0; j < 7; ++j)
			 {
				 msg5.points[0].velocities[j]=0.0;
				 msg5.points[0].effort[j] = 0.0;
			 }
    msg5.points[0].time_from_start = ros::Duration(1.0);
    ros::Rate loop_rate(10);

    while(ros::ok())
    {
      if(shut_shut_down)
      {ros::shutdown();}

      ros::spinOnce();
      loop_rate.sleep();
    }

    return 0 ;
  }
