#include <ros/ros.h>
#include <iostream>
#include <string>
#include <cwru_ariac/time_estimator_service_msg.h>
using namespace std;


int main(int argc, char **argv) {
	ros::init(argc, argv, "time_estimator_client");
	ros::NodeHandle nh;
 	ros::ServiceClient client = nh.serviceClient<cwru_ariac::time_estimator_service_msg>("time_estimator");
	cwru_ariac::time_estimator_service_msg srv;

	srv.request.start_track = 2;
	srv.request.start_arm = 2;
	srv.request.start_flag = 0;

	srv.request.finish_track = 3;
	srv.request.finish_arm = 0;
	srv.request.finish_flag = 0;


	
	if(client.call(srv)){
		ROS_INFO("Estimated time is: %f", srv.response.time_estimate.toSec());
	}

}



