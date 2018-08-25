#include "point_cloud_registration.h"
#include <ros/ros.h>

namespace mkp_pcd_registration
{
  pcd_registration::pcd_registration()
  {
    // Create the default ICP algorithm
    	icp.setDefault();

  }
  void pcd_registration::PCDprocessRegistration()
  {
    int ss=0;
    std::cout << "Dps.size()= " << Dps.size() << std::endl;
    for(int i=1;i<Dps.size();i++)
    {
    std::cout << "i= " << i << std::endl;
    // Compute the transformation that expresses data in ref
  	PM::TransformationParameters T = icp(Dps[i], Dps[i-1]);
  	// Transform data to express it in ref
  	DP data_out(Dps[i]);
    global= global*T;
  	icp.transformations.apply(data_out, global);
  	data_out.save("pointCloudFile_out" + std::to_string(i) + ".pcd");
    }

  }
  void pcd_registration::LoadFiles()
  {
    const DP ref(DP::load("pointCloudFile_1.pcd"));
    Dps.push_back(ref);
  	DP data(DP::load("pointCloudFile_2.pcd"));
    Dps.push_back(data);
  	DP data1(DP::load("pointCloudFile_3.pcd"));
    Dps.push_back(data1);
    DP data2(DP::load("pointCloudFile_4.pcd"));
    Dps.push_back(data2);
  }

  void pcd_registration::spin()
  {
    ros::spin();
  }
  pcd_registration::~pcd_registration()
  {

  }


}
