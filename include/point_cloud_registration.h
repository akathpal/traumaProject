#ifndef POINT_CLOUD_REGISTRATION_H_
#define POINT_CLOUD_REGISTRATION_H_
#include "pointmatcher/PointMatcher.h"
#include <cassert>
#include <iostream>
#include "boost/filesystem.hpp"

typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;

namespace mkp_pcd_registration
{
class pcd_registration
{
private:
     PM::TransformationParameters global = Eigen::Matrix4f::Identity();
     std::vector<DP> Dps;
     PM::ICP icp;

public:
     pcd_registration();
     void spin();
     ~pcd_registration();
     void PCDprocessRegistration();
     void LoadFiles();


};

}


#endif
