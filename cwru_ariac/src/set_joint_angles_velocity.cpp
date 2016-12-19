//
// Created by shipei on 10/25/16.
// Modified by HMS Queen Elizabeth on 18/12/16

#include <AriacBase.h>
#include <stdio.h>      /* printf */
#include <time.h>       /* time_t, struct tm, difftime, time, mktime */
#include <ctime>
#include <std_msgs/Float64.h>
#include <stdlib.h>

ros::Publisher joint_trajectory_publisher_2;
sensor_msgs::JointState current_joint_states;
bool called = false;

bool g_arrived = false;
vector<double> g_target_pose;

ros::Time request_time;
ros::Time completion_time;

/// Create a JointTrajectory with all positions set to zero, and command the arm.
ros::Time sendArmCommand(vector<double> joints) {//changed fom void type
    // Create a message to send.
    trajectory_msgs::JointTrajectory msg;
    msg.header.stamp = ros::Time::now();
    // Copy the joint names from the msg off the '/ariac/joint_states' topic.
    msg.joint_names = current_joint_states.name;
    // Create one point in the trajectory.
    msg.points.resize(1);
    // Resize the vector to the same length as the joint names.
    // Values are initialized to 0.
    msg.points[0].positions.resize(current_joint_states.name.size(), 0.0);
    for (int i = 0; i < joints.size(); ++i) {
        msg.points[0].positions[i] = joints[i];
    }
    // How long to take getting to the point (floating point seconds).

    //std::cout<<"Enter time from start value"<<endl;
    //std::cin >> tfs;

    msg.points[0].time_from_start = ros::Duration(0.011);

    //TODO: tune this to get a good balance of speed and stability assuggested by shipei

    // Setting only track's velocity
    //msg.points[0].velocities = {0,0.5,0,0,0,0,0};
    //msg.points[0].accelerations = {2,2,0.5,0.5,0.5,0.5,0.5};
    // Memo: with velocities and accelerations alone cannot move the robot at all

    ROS_INFO_STREAM("Sending command:\n" << msg);
    joint_trajectory_publisher_2.publish(msg);

    return ros::Time::now();
}

void joint_state_callback(const sensor_msgs::JointState::ConstPtr &joint_state_msg)
{
    // ROS_INFO_STREAM("Joint States:\n" << *joint_state_msg);
    current_joint_states = *joint_state_msg;
    called = true;
}

void arrivalCallback(const sensor_msgs::JointState::ConstPtr & joint_state_msg){
    current_joint_states = *joint_state_msg;
    double max_diff = 0.0;
    completion_time = ros::Time::now();
    for (int i = 0; i < 6; ++i){
	double new_diff = abs(g_target_pose[i] - current_joint_states.position[i]);
	max_diff = ((max_diff > new_diff)?max_diff:new_diff);
	//printf("%f, ",max_diff);
    }
    if(!g_arrived && (max_diff < 0.05)){//TODO: change this to test all joint angles
	completion_time = ros::Time::now();//update with actual completion time
	g_arrived = true;
	ros::Duration t_elapsed_secs = completion_time - request_time;
	printf("seconds required to complete motion: %f \n",t_elapsed_secs.toSec());
    }
    else if((completion_time - request_time).toSec() > 5){
	g_arrived = true;
    }
    //ros::Duration(0.005).sleep();//don't check too often
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "set_joint_angles_velocity");

    ros::NodeHandle nh;                                 // standard ros node handle
    ros::Subscriber joint_state_subscriber_2 = nh.subscribe(
            "/ariac/joint_states", 10,
            &joint_state_callback);
    joint_trajectory_publisher_2 = nh.advertise<trajectory_msgs::JointTrajectory>(
            "/ariac/arm/command", 10);
    int joint;
    double angle;
    vector<double> my_pose;

    int cmd_code=7;
    int mode_code=1;
    int bin_code=8;

    while(!called && ros::ok()) {
        ROS_INFO("Waiting for joint feedback...");
        ros::spinOnce();
        ros::Duration(0.2).sleep();
    }

    while (ros::ok()) {
        my_pose.resize(current_joint_states.name.size(), 0.0);
        for (int i = 0; i < my_pose.size(); ++i) {
            my_pose = current_joint_states.position;
	    printf("position 0.2 \n");
        }
	printf("position 0.25 \n");
        //ROS_INFO("my_pose before input %f",my_pose[joint]);
        ROS_WARN("Current joints: {\n    [0]%s:\t\t\t%f, \n    [1]%s:\t%f, \n    [2]%s:\t\t%f, \n    [3]%s:\t\t%f, \n"
                         "    [4]%s:\t\t\t%f, \n    [5]%s:\t\t\t%f, \n    [6]%s:\t\t\t%f}",
                 current_joint_states.name[0].c_str(), my_pose[0], current_joint_states.name[1].c_str(), my_pose[1],
                 current_joint_states.name[2].c_str(), my_pose[2], current_joint_states.name[3].c_str(), my_pose[3],
                 current_joint_states.name[4].c_str(), my_pose[4], current_joint_states.name[5].c_str(), my_pose[5],
                 current_joint_states.name[6].c_str(), my_pose[6]);

        //ROS_INFO("Target Joint Number");
        //ROS_INFO("Press 7 to set all")


g_arrived = false;
std::cout<<"Press 1 to enter set absolute value mode"<<endl;
std::cout<<"Press 2 to enter set delta value mode"<<endl;
std::cout<<"Mode Code: ";
std::cin >> mode_code;

switch (mode_code) {
case 1:
	std::cout<<"Select the target Joint to set, press 7 to set all"<<endl;
        std::cout<<"Press 17 to home"<<endl;
        std::cout<<"Press 18 to select bins and move robot"<<endl;
        std::cout<<"Press 12 to face the bins"<<endl;
	std::cout<<"Press 13 to hover over the bins (close)"<<endl;
	std::cout<<"Press 14 to hover over bins (far)"<<endl;
	std::cout<<"Press 15 to face the conveyor belt"<<endl;
	std::cout<<"Press 16 to hover over the conveyor belt"<<endl;
	std::cout<<"Press 19 to get prepose for bins (far)"<<endl<<"--------------------------------------"<<endl;
	std::cout<<"Press 21 to pick from conveyor belt"<<endl;
	std::cout<<"Press 22 to pick from bins (close)"<<endl;
	std::cout<<"Press 23 to pick from bins (far)"<<endl;
	std::cout<<"Press 24 to pick from AGV Right"<<endl;
	std::cout<<"Press 25 to get to prepose for AGV Right"<<endl;
        std::cout<<"Joint Code: ";
      	std::cin >> cmd_code;
        
        switch (cmd_code) {
        
	    case 0:
		std::cout << "Set Joint# 00 Elbow_joint (prep:-2.841614):";
        	std::cin >> angle;
		break;
	    case 1:
        	std::cout << "Set Joint# 01 Track:";
        	std::cin >> angle;
        	my_pose[1] = angle;
		break;
	    case 2:
         	std::cout << "Set Joint# 02 Shoulder_lift (prep: -1.581264):";
        	std::cin >> angle;
        	my_pose[2] = angle;
		break;
	    case 3:
        	std::cout << "Set Joint# 03 Shoulder_pan (prep: 1.560937):";
        	std::cin >> angle;
        	my_pose[3] = angle;
		break;
	    case 4:
        	std::cout << "Set Joint# 04 wrist_1 (prep: 4.379010):";
        	std::cin >> angle;
        	my_pose[4] = angle;
		break;
  	    case 5:
       		std::cout << "Set Joint# 05 wrist_2 (prep: -0.083689):";
       	 	std::cin >> angle;
        	my_pose[5] = angle;
		break;
	    case 6:
        	std::cout << "Set Joint# 06 wrist_3 (prep: 2.414974):";
        	std::cin >> angle;
        	my_pose[6] = angle;
		break;
	    case 7:
        	std::cout << "Set Joint# 00 Elbow_joint (prep:-2.841614):";
        	std::cin >> angle;
        	my_pose[0] = angle;
        	std::cout << "Set Joint# 01 Track:";
        	std::cin >> angle;
        	my_pose[1] = angle;
         	std::cout << "Set Joint# 02 Shoulder_lift (prep: -1.581264):";
        	std::cin >> angle;
        	my_pose[2] = angle;	
        	std::cout << "Set Joint# 03 Shoulder_pan (prep: 1.560937):";
        	std::cin >> angle;
        	my_pose[3] = angle;
        	std::cout << "Set Joint# 04 wrist_1 (prep: 4.379010):";
        	std::cin >> angle;
        	my_pose[4] = angle;
       		std::cout << "Set Joint# 05 wrist_2 (prep: -0.083689):";
       	 	std::cin >> angle;
        	my_pose[5] = angle;
        	std::cout << "Set Joint# 06 wrist_3 (prep: 2.414974):";
        	std::cin >> angle;
        	my_pose[6] = angle;
		break;

	    case 17:
		std::cout << "auto turnning to prepose..";
		my_pose[0] =-2.841614;
		my_pose[2] =-1.581264;
		my_pose[3] =1.560937;
		my_pose[4] =4.379010;
		my_pose[5] =-0.083689;
		//my_pose[6] =2.414974;
		break;
	    case 15:
		std::cout << "facing the conveyor belt..";
		my_pose[0] =-2.841614;
		my_pose[2] =-1.581264;
		my_pose[3] =3.128170;
		my_pose[4] =4.379010;
		my_pose[5] =-0.083689;
		break;
 	    case 14:
		std::cout << "reaching bins (far)";
		my_pose[0] =-0.571992;
		my_pose[2] =-2.913995;
		my_pose[3] =-0.012403;
		my_pose[4] =5.001631;
		my_pose[5] =1.667814;
		break;
 	    case 16:
		std::cout << "reaching conveyor belt..";
		my_pose[0] =-1.607046;
		my_pose[2] =-2.2818;
		my_pose[3] =3.081711;
		my_pose[4] =5.484607;
		my_pose[5] =1.510098;
		break;
  	    case 12:
		std::cout <<"facing the bins..";
		my_pose[0] =-2.841614;
		my_pose[2] =-1.581264;
		my_pose[3] =-0.024400;
		my_pose[4] =4.379010;
		my_pose[5] =-0.083689;
		//my_pose[6] =2.414974;
		break;
  	    case 19:
		std::cout <<"pre far bin pose..";
		my_pose[0] =-1.204143;
		my_pose[2] =-1.943020;
		my_pose[3] =-0.020627;
		my_pose[4] =4.292694;
		my_pose[5] =-0.106308;
		//my_pose[6] =2.414974;
		break;

	    case 13:
		std::cout <<"reaching bins (close)";
		my_pose[0] =-2.08;
		my_pose[2] =-2.47;
		my_pose[3] =-0.027;
		my_pose[4] =0.066393;
		my_pose[5] =1.683881;
		//my_pose[6] =1.549463;
		break;	

	    case 21:
		std::cout <<"picking from Conveyor Belt";
		my_pose[0] =-1.598208;
		my_pose[2] =-2.499164;
		my_pose[3] =3.588809;
		my_pose[4] =5.705032;
		my_pose[5] =1.533290;
		//my_pose[6] =1.549463;
		break;
	    case 22:
		std::cout <<"picking from bins (close)";
		my_pose[0] =-2.127689;
		my_pose[2] =-2.606857;
		my_pose[3] =-0.027287;
		my_pose[4] =0.113149;
		my_pose[5] =1.667603;
		break;
	    case 23:
		std::cout <<"picking from bins (far)";
		my_pose[0] =-0.532396;
		my_pose[2] =-3.076464;
		my_pose[3] =-0.015197;
		my_pose[4] =5.174618;
		my_pose[5] =1.657853;
		break;
	    case 24:
		std::cout <<"picking from AVG Right";
		my_pose[0] =-1.480375;
		my_pose[2] =-2.434312;
		my_pose[3] =1.531265;
		my_pose[4] =5.482519;
		my_pose[5] =1.552242;
		break;
	    case 25:
		std::cout <<"preparing for AVG Right";
		my_pose[0] =-1.340833;
		my_pose[2] =-2.072419;
		my_pose[3] =1.564120;
		my_pose[4] =4.166029;
		my_pose[5] =-0.075246;
		break;

	    case 18:
		std::cout << "Choose a bin number from 1 to 8:"<<endl;
		std::cout << "Or choose the AGV on the left by pressing 11"<<endl;
		std::cout << "AGV on the right by pressing 12"<<endl;
        	std::cin >> bin_code;
		switch (bin_code){
			case 8:
        		std::cout << "Moving to bin#8/4";
        		my_pose[1] = 1;
			break;
			case 4:
        		std::cout << "Moving to bin#8/4";
        		my_pose[1] = 1;
			break;
			case 7:
        		std::cout << "Moving to bin#7/3";
        		my_pose[1] = 0.235;
			break;
			case 3:
        		std::cout << "Moving to bin#7/3";
        		my_pose[1] = 0.235;
			break;
			case 6:
        		std::cout << "Moving to bin#6/2";
        		my_pose[1] = -0.53;
			break;
			case 2:
        		std::cout << "Moving to bin#6/2";
        		my_pose[1] = -0.53;
			break;
			case 5:
        		std::cout << "Moving to bin#5/1";
        		my_pose[1] = -1.295;
			break;
			case 1:
        		std::cout << "Moving to bin#5/1";
        		my_pose[1] = -1.295;
			break;
			case 11:
        		std::cout << "Moving to AGV LEFT";
        		my_pose[1] = 2.1;
			break;
			case 12:
        		std::cout << "Moving to bin#5/1";
        		my_pose[1] = -2.1;
			break;


		}

	        break;
		
		
	}
break;
case 2:
        std::cout << "Select joint number:";
        std::cin >> joint;
        std::cout << "Increase angles (can be negative): ";
        std::cin >> angle;
        my_pose[joint] += angle;
break;
}

        request_time = sendArmCommand(my_pose);//send commands and store the time at whcih they were sent
	g_target_pose = my_pose;//set the global variable for use by the arrival checker callback

	completion_time = ros::Time::now();

	ros::Subscriber arrivalListener = nh.subscribe("/ariac/joint_states", 1, &arrivalCallback);

        //ros::Duration(5).sleep();

        ros::spinOnce();
        called = false;
	
	ros::Time timeOutStart = ros::Time::now();	
	
        while(!called || !g_arrived) {
            ros::spinOnce();
            ros::Duration(0.02).sleep();
        }
    }
    return 0;
}

