#include "utility/tracking_health.h"
#include <limits>
#include <stdexcept>
#include <iostream>
void check(bool b){if(!b)throw std::runtime_error("tracking health test failed");}
int main(){Eigen::Vector3d p=Eigen::Vector3d::Zero(),v=p,b=p;Eigen::Matrix3d R=Eigen::Matrix3d::Identity();int weak=0;
 check(!trackingStateFailed(p,p,R,v,b,b,20,true,weak));
 check(!trackingStateFailed(p,p,R,v,b,b,0,true,weak));
 check(!trackingStateFailed(p,p,R,v,b,b,0,true,weak));
 check(trackingStateFailed(p,p,R,v,b,b,0,true,weak));
 check(!trackingStateFailed(p,p,R,v,b,b,20,true,weak));check(weak==0);
 auto nan=p;nan.x()=std::numeric_limits<double>::quiet_NaN();
 check(trackingStateFailed(nan,p,R,v,b,b,20,true,weak));
 check(trackingStateFailed(Eigen::Vector3d(3,0,0),p,R,v,b,b,20,true,weak));
 check(trackingStateFailed(p,p,R,v,Eigen::Vector3d(3,0,0),b,20,true,weak));
 std::cout<<"PASS: consecutive track loss, recovery, nonfinite state, pose jump and bias rejection\n";}
