#include "point_cloud_targeting.h"
#include "ros/ros.h"

int main(int argc,char**argv){
  ros::init(argc, argv, "point_cloud_targeting");
  mkp_pcd_targeting::pcd_targeting().spin();
  return 0 ;
}
