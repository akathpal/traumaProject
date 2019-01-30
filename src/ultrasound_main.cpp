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
#include <std_msgs/Bool.h>
#include <string>
#include "mkp_pcd/movj.h"
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include "mkp_pcd/registration_merge_and_output.h"
#include "mkp_pcd/registration_process_registration.h"
#include "mkp_pcd/registration_take_view.h"
#include "mkp_pcd/registration_clear.h"
#include "mkp_pcd/FindWidthOrHeight.h"
#include "mkp_pcd/trigger.h"

bool go = false;


void trigger(const std_msgs::Float64ConstPtr& In) // need to add "const"
{
  if(In->data == 777.0)
  {
    go = true;
  }
}
/*
std_msgs::Float64 jjj_msg;
void jjj_callback(const std_msgs::Float64ConstPtr& msg)
{
 jjj_msg.data=msg->data;
}
*/


struct cmd_data
{
int cmd_type;
double para1;
double para2;
double para3;
double para4;
double para5;
double para6;
double para7;
};

std::vector<cmd_data> J_points;
std::vector<int> point_reg;

class ultrasound
{
private:
  ros::NodeHandle n_;
  ros::ServiceClient Take_view_;
  ros::ServiceClient Merge_view_;
  ros::ServiceClient Clear_view_;
  ros::ServiceClient PCD_Registration_;
  ros::ServiceClient send_to_MOT_;
  ros::ServiceClient find_width_;
  ros::ServiceClient FindFastScan_;
  ros::Subscriber trigger_;
  ros::Publisher shut_down;


public:
  ultrasound();

  void shut_down_all()
  {
    std_msgs::Bool shut_down_;
    shut_down_.data = true;
    shut_down.publish(shut_down_);
    ros::spinOnce();
  }


  double findWidth()
  {
    mkp_pcd::FindWidthOrHeight fuckthisshit;
    fuckthisshit.request.trigger.data = true;
    if(find_width_.call(fuckthisshit))
    {ROS_INFO_STREAM("****************width_ : " << fuckthisshit.response.distance <<std::endl);}
    else{ROS_ERROR("Failed to call service find_width_");}

  }




  void movej(int point_index)
  {
      mkp_pcd::movj movj_ob;
      movj_ob.request.movj.positions.resize(7);
      movj_ob.request.movj.positions[0] = J_points[point_index].para1;
      movj_ob.request.movj.positions[1] = J_points[point_index].para2;
      movj_ob.request.movj.positions[2] = J_points[point_index].para3;
      movj_ob.request.movj.positions[3] = J_points[point_index].para4;
      movj_ob.request.movj.positions[4] = J_points[point_index].para5;
      movj_ob.request.movj.positions[5] = J_points[point_index].para6;
      movj_ob.request.movj.positions[6] = J_points[point_index].para7;
      //ROS_INFO("Before services");
      if(send_to_MOT_.call(movj_ob))
      {ROS_INFO_STREAM("send_to_MOT_ "<< point_index << " : " << movj_ob.response.error_code <<std::endl);}
      else{ROS_ERROR("Failed to call service movj");}

  }

  void take_view()
  {
    mkp_pcd::registration_take_view pt;
    ROS_INFO(" start calling Take_view_ ");
    //ROS_INFO("Before services");
    if(Take_view_.call(pt))
    {
      ROS_INFO_STREAM("Take_view_ Called done");
    }
    else
    {
      ROS_ERROR("Failed to call service Take_view_");
    }
  }

  void delay(int sec)
  {
    ros::Duration(static_cast<double>(sec)).sleep();
  }

  void PCD_ProcessRegistration(std::vector<int> &input_vector, double type__)
  {
    mkp_pcd::registration_process_registration pt;
    pt.request.indices.assign(input_vector.begin(),input_vector.end());
    pt.request.type = static_cast<int>(type__);

    for(int i=0; i < pt.request.indices.size();i++)
    {
    ROS_INFO_STREAM(pt.request.indices[i]);
    }

    ROS_INFO(" start calling PCD_Registration_ ");
    //ROS_INFO("Before services");
    if(PCD_Registration_.call(pt))
    {
      ROS_INFO_STREAM("PCD_Registration_ Called done");
    }
    else
    {
      ROS_ERROR("Failed to call service PCD_Registration_");
    }

  }

  void PCD_Merge()
  {
    mkp_pcd::registration_merge_and_output pt;
    ROS_INFO(" start calling Merge ");
    //ROS_INFO("Before services");
    if(Merge_view_.call(pt))
    {
      ROS_INFO_STREAM("Merge Called done");
    }
    else
    {
      ROS_ERROR("Failed to call service Merge");
    }
  }

  void PCD_Clear()
  {
    mkp_pcd::registration_clear pt;
    ROS_INFO(" start calling Clear_view_ ");
    //ROS_INFO("Before services");
    if(Clear_view_.call(pt))
    {
      ROS_INFO_STREAM("Clear_view_ Called done");
    }
    else
    {
      ROS_ERROR("Failed to call service Clear_view_");
    }
  }

  void FindFastScan()
  {
    mkp_pcd::trigger tr;
    ROS_INFO(" start calling FindFastScan ");
    //ROS_INFO("Before services");
    if(FindFastScan_.call(tr))
    {
      ROS_INFO_STREAM("FindFastScan Called done");
    }
    else
    {
      ROS_ERROR("Failed to call service FindFastScan");
    }
  }



};

ultrasound::ultrasound()
{
  Take_view_= n_.serviceClient< mkp_pcd::registration_take_view > ("mkp_pcd/pcd_registration_take_view");
  Merge_view_ = n_.serviceClient< mkp_pcd::registration_merge_and_output > ("mkp_pcd/pcd_registration_merge_and_output");
  Clear_view_ = n_.serviceClient< mkp_pcd::registration_clear > ("mkp_pcd/pcd_registration_clear_all_service");
  PCD_Registration_= n_.serviceClient< mkp_pcd::registration_process_registration > ("mkp_pcd/pcd_registration_process_registration");
  // movj
  send_to_MOT_= n_.serviceClient< mkp_pcd::movj > ("mkp_pcd/movj");
  find_width_ = n_.serviceClient< mkp_pcd::FindWidthOrHeight> ("mkp_pcd/PhantomWidth");
  FindFastScan_ = n_.serviceClient< mkp_pcd::trigger> ("mkp_pcd/pcd_find_fastscan");
  trigger_= n_.subscribe("/jjj",1000, trigger);//TODO
  shut_down = n_.advertise<std_msgs::Bool>("mkp_pcd/shut_down", 1);
}




int main(int argc,char**argv){
  ros::init(argc, argv, "ultrasound_main");
  ultrasound ultra;
// read the point
  std::string package_n = ros::package::getPath("mkp_pcd");
  std::string file_n_J("/J_positions_file.txt");
  std::ifstream fileJ( package_n + file_n_J);
  std::vector<float> rJ;
  std::string word1_J;
  while (fileJ >> word1_J) { rJ.push_back(atof(word1_J.c_str()));}
  int size_all_J = rJ.size();


  cmd_data J_point;
  int number_of_points_J = size_all_J / 7;
  ROS_INFO("size_all_J= %d", size_all_J);
  ROS_INFO("number_of_points_J= %d", number_of_points_J);
  for (int i = 0; i < number_of_points_J; i++) {
    J_point.para1=rJ[7 * i];
    J_point.para2=rJ[7 * i + 1];
    J_point.para3=rJ[7 * i + 2];
    J_point.para4=rJ[7 * i + 3];
    J_point.para5=rJ[7 * i + 4];
    J_point.para6=rJ[7 * i + 5];
    J_point.para7=rJ[7 * i + 6];
    J_points.push_back(J_point);
  }
ros::Rate loop_rate(10);
int delay_take_view = 2;
int delay_move = 0;


//start

while(ros::ok())
{

  if(go == true)
  {
    ultra.movej(0);
    ultra.findWidth();
    ultra.PCD_Clear();
    for(int i = 1; i <=6 ; i++)
    {
      ROS_INFO_STREAM("going to point: " << i );
      ultra.movej(i-1);
      ultra.delay(delay_take_view);
      ultra.take_view();
      ultra.delay(delay_move);
    }

    for(int i = 6; i >= 1; i-- )
    {
     ROS_INFO_STREAM("going to point: " << i );
     ultra.movej(i-1);
    }

    for(int i = 7; i <= 11 ; i++)
    {
      ROS_INFO_STREAM("going to point: " << i );
      ultra.movej(i-1);
      ultra.delay(delay_take_view);
      ultra.take_view();
      ultra.delay(delay_move);
    }

    for(int i = 11; i >= 7; i-- )
    {
     ROS_INFO_STREAM("going to point: " << i );
     ultra.movej(i-1);
    }

    for(int i = 12; i <= 15 ; i++)
    {
      ROS_INFO_STREAM("going to point: " << i );
      ultra.movej(i-1);
      ultra.delay(delay_take_view);
      ultra.take_view();
      ultra.delay(delay_move);
    }

    for(int i = 15; i >= 12; i-- )
    {
      ROS_INFO_STREAM("going to point: " << i );
      ultra.movej(i-1);
    }


    for(int i = 16; i <= 20 ; i++)
    {
      ROS_INFO_STREAM("going to point: " << i );
      ultra.movej(i-1);
      ultra.delay(delay_take_view);
      ultra.take_view();
      ultra.delay(delay_move);
    }

    for(int i = 20; i >= 16; i-- )
    {
     ROS_INFO_STREAM("going to point: " << i );
     ultra.movej(i-1);
    }



   ultra.movej(0);

   point_reg.push_back(1);
   point_reg.push_back(2);
   point_reg.push_back(3);
   point_reg.push_back(4);
   point_reg.push_back(5);
   point_reg.push_back(6);
   //point_reg.push_back(7);
   //point_reg.push_back(8);
   ultra.PCD_ProcessRegistration(point_reg,1);
   point_reg.clear();

   point_reg.push_back(1);
   point_reg.push_back(7);
   point_reg.push_back(8);
   point_reg.push_back(9);
   point_reg.push_back(10);
   point_reg.push_back(11);
   //point_reg.push_back(12);
   //point_reg.push_back(13);
   //point_reg.push_back(14);
   //point_reg.push_back(15);
   ultra.PCD_ProcessRegistration(point_reg,1);
   point_reg.clear();
   ultra.PCD_Merge();
   ultra.FindFastScan();

/*
   point_reg.push_back(12);
   point_reg.push_back(13);
   point_reg.push_back(14);
   point_reg.push_back(15);
   //point_reg.push_back(20);
   //point_reg.push_back(21);
   //point_reg.push_back(22);
   ultra.PCD_ProcessRegistration(point_reg,1);
   point_reg.clear();

   point_reg.push_back(12);
   point_reg.push_back(16);
   point_reg.push_back(17);
   point_reg.push_back(18);
   point_reg.push_back(19);
   point_reg.push_back(20);
   ultra.PCD_ProcessRegistration(point_reg,1);
   point_reg.clear();
   ultra.PCD_Merge();


   point_reg.push_back(1);
   point_reg.push_back(2);
   ultra.PCD_ProcessRegistration(point_reg,2);
*/



    go = false;
    ultra.shut_down_all();
    //loop_rate.sleep();
    //ros::shutdown();

  } // end if(go == true)

  ros::spinOnce();
  loop_rate.sleep();
}


  return 0 ;
}
