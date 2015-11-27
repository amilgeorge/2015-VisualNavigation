// This source code is intended for use in the teaching course "Vision-Based Navigation" at Technical University Munich only.
// Copyright 2015 Vladyslav Usenko, Joerg Stueckler, Technical University Munich

#include <ros/ros.h>
#include <string>
#include "../include/uav_controller.hpp"


Eigen::Vector3d getTokens(const std::string &s)
{
    std::stringstream ss(s+' ');
    std::string item;
    Eigen::Vector3d elems;
    char delim=',';
    int count=0 ;
    while(std::getline(ss, item, delim))
    {
        elems(count++)= ::atof(item.c_str());
    }
    return elems;
}

int main(int argc, char** argv) {

	ros::init(argc, argv, "ex3_solution");
	ros::NodeHandle nh;

	std::string kp_string ;

	nh.param<std::string>("kp", kp_string,"default_value");

	std::string kd_string ;
	nh.param<std::string>("kd", kd_string,"default_value");

	std::string ki_string ;
	nh.param<std::string>("ki", ki_string,"default_value");

	Eigen::Vector3d kp_arr=getTokens(kp_string) ;
	Eigen::Vector3d kd_arr=getTokens(kd_string) ;
	Eigen::Vector3d ki_arr=getTokens(ki_string) ;

	std::cout << "kp_str::\n" << kp_string<< std::endl;


//	std::cout << "kp_arr::\n" << kp_arr << std::endl;
//	std::cout << "kd_arr::\n" << kd_arr<< std::endl;
//	std::cout << "ki_arr::\n" << ki_arr<< std::endl;
//
//
	UAVController<double> u(nh);

	//UAVController<double> u(nh);

	Sophus::SE3d desired_pose;
	desired_pose.translation() << 0,0,1;
	desired_pose.setQuaternion(Eigen::Quaterniond::Identity());

    u.setDesiredPose(desired_pose);

	ros::spin();

	return 0;
}

