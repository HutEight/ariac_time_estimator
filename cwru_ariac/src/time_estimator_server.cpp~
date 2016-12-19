#include <ros/ros.h>
#include <cwru_ariac/time_estimator_service_msg.h>
#include <iostream>
#include <string>
using namespace std;


ros::Duration timeEstimate(int start[], int finish[]){//each argument should be of length 2
    ros::Duration trackComponent = ros::Duration(0);
    ros::Duration armComponent = ros::Duration(0);

    if (start[2] == 1 || finish[2] == 1){//a pick or place
	switch(start[2]){
	    case 0://bin further from track
		return ros::Duration(0.13);
		break;
	    case 1://bin nearer to track
		return ros::Duration(0.16);
		break;
	    case 3://conveyor belt
		return ros::Duration(0.44);
		break;
	    case 4:
		return ros::Duration(0.52);
		break;
	}
    }
    
    int s = start[0];
    int f = finish[0];
    if (s == 0 || f == 0 || s == 5 || f == 5){//motion starts or ends at an AGV
        switch(abs(s-f)) {
            case 0:
                trackComponent = ros::Duration(0);
                break;
            case 1:
                trackComponent = ros::Duration(0.42);
                break;
            case 2:
                trackComponent = ros::Duration(0.64);
                break;
            case 3:
                trackComponent = ros::Duration(0.88);
                break;
            case 4:
                trackComponent = ros::Duration(1.12);
                break;
            case 5:
                trackComponent = ros::Duration(1.485);
                break;
	}
    }else{//motion starts and ends aligned with a bin
        switch(abs(s-f)){
            case 0:
                trackComponent = ros::Duration(0);
                break;
            case 1:
                trackComponent = ros::Duration(0.3);
                break;
            case 2:
                trackComponent = ros::Duration(0.525);
                break;
            case 3:
                trackComponent = ros::Duration(0.78);
                break;
	}
    }
    
    s = start[1];
    f = finish[1];
    
    if (s == f){
        armComponent = ros::Duration(0);
    }else if((s == 0 && f == 1) || (f == 0 && s == 1)){//far bin to near bin
        armComponent = ros::Duration(1.125);
    }else if((s == 0 && f == 2) || (f == 0 && s == 2)){//far bin to home
        armComponent = ros::Duration(1.41);
    }else if((s == 1 && f == 2) || (f == 1 && s == 2)){//near bin to home
        armComponent = ros::Duration(0.985);
    }else if((s == 2 && f == 3) || (f == 2 && s == 3)){//far home to conveyor belt
        armComponent = ros::Duration(0.83);
    }
    
    return (trackComponent > armComponent ? trackComponent : armComponent);
}


bool execute_cb(cwru_ariac::time_estimator_service_msgRequest& request,
cwru_ariac::time_estimator_service_msgResponse &response)
{
	ROS_INFO("Execute_Cb activated..");
	
	int start_array[] = {request.start_track, request.start_arm, request.start_flag};
	int finish_array[] = {request.finish_track, request.finish_arm, request.finish_flag};
	//int start_array[] = {0,0,0};
	//int finish_array[] = {0,0,0};
	//start_array.push_back(request.start[0].data);		
	//start_array.push_back(request.start[1].data);
	//start_array.push_back(request.start[2].data);
	//finish_array.push_back(request.finish[0].data)
	//finish_array.push_back(request.finish[1].data)
	//finish_array.push_back(request.finish[2].data)

	response.time_estimate = timeEstimate(start_array, finish_array);

	return true;
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "time_estimator_server");
	ros::NodeHandle nh;
	ros::ServiceServer service = nh.advertiseService("time_estimator", execute_cb);
	ROS_INFO("Ready to estimate motion time.");
	ros::spin();

	return 0;
}









