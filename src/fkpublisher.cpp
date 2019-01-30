#include "ros/ros.h"
#include "std_msgs/String.h"
#include <vector>
#include <sstream>
#include <geometry_msgs/Transform.h>
#include <sensor_msgs/JointState.h>
#include <ros/package.h>
#include <fstream>
#include <iostream>
#include <string>
#include <geometry_msgs/Twist.h>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolver.hpp>
#include <tf/transform_broadcaster.h>
#include <boost/make_shared.hpp>
#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;
pcl::PassThrough<PointT> pass;
PointCloud pointcloud_out, pointcloud_in;

sensor_msgs::JointState joints;
geometry_msgs::Twist fk_twist;
bool initialized = false;
int kinematics_status;
double rx,ry,rz,rw;
tf::Pose transform_tf;
PointCloud::Ptr msg, msg_before_trans;


void pcl_callback(const sensor_msgs::PointCloud2ConstPtr cloud) {
	//pcl::fromROSMsg(*cloud, pointcloud_out);
	pcl::fromROSMsg(*cloud, pointcloud_in);

  //msg = pointcloud_in.makeShared();
  PointCloud pointcloud_buffer;
  PointCloud::Ptr point_cloud__;
	PointCloud::Ptr cloud_o;
	point_cloud__= pointcloud_buffer.makeShared();

	//ROS_INFO("Width= %d", pointcloud_in.width);

	for(int j=0; j< 280;j++)
{
		for(int i=0;i<pointcloud_in.width;i++)
	{
		point_cloud__->push_back(pointcloud_in.at(i,j)); // TODO need to get rid of this
	}
}

  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.3, 5);
  pass.setInputCloud(point_cloud__);
  pass.filter(*point_cloud__);
  pcl_ros::transformPointCloud(*point_cloud__, pointcloud_out, transform_tf);

	msg = pointcloud_out.makeShared();

	pass.setFilterFieldName("x");
	pass.setFilterLimits(0, 1.0);
	pass.setInputCloud(msg);
	pass.filter(*msg);

	pass.setFilterFieldName("y");
	pass.setFilterLimits(-0.5, 0.5);
	pass.setInputCloud(msg);
	pass.filter(*msg);

	pass.setFilterFieldName("z");
	pass.setFilterLimits(-0.2, 0.4);
	pass.setInputCloud(msg);
	pass.filter(*msg);


	pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
	msg->header.frame_id = "world";
}



KDL::Chain LWR() {
  KDL::Chain chain;

  // base
  chain.addSegment(
  // KDL::Segment(KDL::Joint(KDL::Joint::None),KDL::Frame::DH_Craig1989(0, 0, 0.33989, 0)));
      KDL::Segment(KDL::Joint(KDL::Joint::None),
                   KDL::Frame::DH_Craig1989(0, 0, 0.34, 0)));
  // joint 1
  chain.addSegment(
      KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
                   KDL::Frame::DH_Craig1989(0, -M_PI_2, 0, 0)));
  // joint 2
  chain.addSegment(
  // KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH_Craig1989(0, M_PI_2, 0.40011, 0)));
      KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
                   KDL::Frame::DH_Craig1989(0, M_PI_2, 0.4, 0)));
  // joint 3
  chain.addSegment(
      KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
                   KDL::Frame::DH_Craig1989(0, M_PI_2, 0, 0)));
  // joint 4
  chain.addSegment(
      KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
                   KDL::Frame::DH_Craig1989(0, -M_PI_2, 0.4000, 0)));
  // joint 5
  chain.addSegment(
      KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
                   KDL::Frame::DH_Craig1989(0, -M_PI_2, 0, 0)));
  // joint 6
  chain.addSegment(
      KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
                   KDL::Frame::DH_Craig1989(0, M_PI_2, 0, 0)));

  // joint 7 (with flange adapter)
  chain.addSegment(
      KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
                   KDL::Frame::DH_Craig1989(0, 0, 0.12597, 0)));
  return chain;

}
// TODO: do something
KDL::Chain chain = LWR();
KDL::ChainFkSolverPos_recursive fksolver = KDL::ChainFkSolverPos_recursive(
    chain);
KDL::Frame cartpos;
unsigned int nj = chain.getNrOfJoints();  // get the number of joints from the chain
KDL::JntArray jointpositions = KDL::JntArray(nj);  // define a joint array in KDL format for the joint positions

void get_joints(const sensor_msgs::JointState & data) {

  for (int i = 0; i < data.position.size(); i++) {
    // if this is not the first time the callback function is read, obtain the joint positions
    if (initialized) {
      joints.position[i] = data.position[i];
      // otherwise initilize them with 0.0 values
    } else {
      joints.position.push_back(0.0);
    }
  }
  initialized = true;

    for (int k = 0; k < nj; k++) {
      jointpositions(k) = joints.position[k];
    }
    kinematics_status = fksolver.JntToCart(jointpositions, cartpos);
//    fk_twist.linear.x=cartpos.p[0];
//    fk_twist.linear.y=cartpos.p[1];
//    fk_twist.linear.z=cartpos.p[2];
    cartpos.M.GetQuaternion(rx,ry,rz,rw);
    /*

     * // 2018.06.26
     *
     *     Don't move this:
     *
     *     static tf::TransformBroadcaster br;
     *
     *     to global, otherwise it will show you below:
     *
       [FATAL] [1530044283.875503310]: You must call ros::init() before creating the first NodeHandle
       Couldn't find an AF_INET address for []
       Couldn't find an AF_INET address for []
       [ERROR] [1530044283.877908410]: [registerPublisher] Failed to contact master at [:0].  Retrying...
       Couldn't find an AF_INET address for []
       Couldn't find an AF_INET address for []
     *
     *
     *
     */
    static tf::TransformBroadcaster br;
		tf::Transform transform;
    // FK
    transform.setOrigin( tf::Vector3(cartpos.p[0], cartpos.p[1], cartpos.p[2]) );
    tf::Quaternion q(rx,ry,rz,rw);
    transform.setRotation(q);
    // Camera_to_Flange
		tf::Transform transform_camera;
  	transform_camera.setOrigin(tf::Vector3(-0.1057, 0.0068, -0.0391));
  	tf::Quaternion q_camera;
  	q_camera.setRPY(-0.0923 ,0.0637,-1.5261);
  	transform_camera.setRotation(q_camera);

    br.sendTransform(tf::StampedTransform(transform * transform_camera, ros::Time::now(),
     "world", "tf_iiwa_FK"));

}

int main(int argc, char ** argv) {

  ros::init(argc, argv, "fkpublisher");
  ros::NodeHandle n_;
  //ros::Publisher fk_iiwa= n.advertise<geometry_msgs::Twist>("fk_chatter",1000);
  ros::Subscriber joints_sub = n_.subscribe("/iiwa/joint_states", 10,
                                            get_joints);
  ros::Subscriber sub_pcl_save = n_.subscribe("camera/depth_registered/points",
          1000, pcl_callback);

	ros::Publisher pub = n_.advertise<PointCloud> ("mkp_pcd/pointCloudToRobotFrame", 1);
  tf::TransformListener listener;
	ros::Rate loop_rate(10);


  while(ros::ok())
  {
  pub.publish (msg);

	tf::StampedTransform transform_;
	try{
		 listener.lookupTransform("world", "tf_iiwa_FK",
															ros::Time(0), transform_);
		 //ROS_INFO("pass101");
		 transform_tf.setOrigin(transform_.getOrigin());
		 //ROS_INFO("pass103");
		 transform_tf.setRotation(transform_.getRotation());
		 ROS_INFO("pass105");
	 }
	 catch (tf::TransformException ex){
		 ROS_ERROR("%s",ex.what());
		 ros::Duration(5.0).sleep();
	 }

	ros::spinOnce();
	loop_rate.sleep();
  }
  return 0;
}
