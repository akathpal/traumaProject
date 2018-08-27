#include "ros/ros.h"
#include "point_cloud_registration.h"


using namespace std;
int main(int argc,char**argv){
  ros::init(argc, argv, "point_cloud_registration");
  mkp_pcd_registration::pcd_registration().spin();
  //REG.LoadFiles();
  //REG.PCDprocessRegistration();
  //REG.spin();
  return 0 ;
}

/*
int main(int argc,char**argv){
  ros::init(argc, argv, "point_cloud_registration");

	typedef PointMatcher<float> PM;
	typedef PM::DataPoints DP;
	PM::TransformationParameters global = Eigen::Matrix4f::Identity();
	// Load point clouds
  std::vector<DP> Dps;
	const DP ref(DP::load("pointCloudFile_1.pcd"));
  Dps.push_back(ref);
	DP data(DP::load("pointCloudFile_2.pcd"));
  Dps.push_back(data);
	DP data1(DP::load("pointCloudFile_3.pcd"));
  Dps.push_back(data1);
  DP data2(DP::load("pointCloudFile_4.pcd"));
  Dps.push_back(data2);
	// Create the default ICP algorithm
	PM::ICP icp;
	// See the implementation of setDefault() to create a custom ICP algorithm
	icp.setDefault();
  int ss=0;
  cout << "Dps.size()= " << Dps.size() << endl;
  for(int i=1;i<Dps.size();i++)
  {
  cout << "i= " << i << endl;

  // Compute the transformation that expresses data in ref
	PM::TransformationParameters T = icp(Dps[i], Dps[i-1]);

	// Transform data to express it in ref
	DP data_out(Dps[i]);
  global= global*T;
	icp.transformations.apply(data_out, global);
	// Safe files to see the results
	//ref.save("test_ref.vtk");
	//data.save("test_data_in.vtk");
	data_out.save("pointCloudFile_out" + to_string(i) + ".pcd");
  }

/
  return 0 ;
}

*/
/*
int main(int argc,char**argv){
  ros::init(argc, argv, "point_cloud_registration");

	typedef PointMatcher<float> PM;
	typedef PM::DataPoints DP;
	PM::TransformationParameters global = Eigen::Matrix4f::Identity();
	// Load point clouds
  std::vector<DP> Dps;
	const DP ref(DP::load("pointCloudFile_1.pcd"));
	const DP data(DP::load("pointCloudFile_2.pcd"));

	// Create the default ICP algorithm
	PM::ICP icp;

	// See the implementation of setDefault() to create a custom ICP algorithm
	icp.setDefault();

	// Compute the transformation that expresses data in ref
	PM::TransformationParameters T = icp(data, ref);

	// Transform data to express it in ref
	DP data_out(data);
	icp.transformations.apply(data_out, T);

	// Safe files to see the results
	//ref.save("test_ref.vtk");
	//data.save("test_data_in.vtk");
	data_out.save("pointCloudFile_out.pcd");
	cout << "Final transformation:" << endl << T << endl;

  return 0 ;
}
*/
