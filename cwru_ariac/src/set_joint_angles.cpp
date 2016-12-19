//
// Created by shipei on 10/25/16.
// Modified by HMS Queen Elizabeth on 18/12/16

#include <AriacBase.h>

ros::Publisher joint_trajectory_publisher_2;
sensor_msgs::JointState current_joint_states;
bool called = false;

/// Create a JointTrajectory with all positions set to zero, and command the arm.
void sendArmCommand(vector<double> joints) {
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
    msg.points[0].time_from_start = ros::Duration(2);
    ROS_INFO_STREAM("Sending command:\n" << msg);
    joint_trajectory_publisher_2.publish(msg);
}


void joint_state_callback(const sensor_msgs::JointState::ConstPtr & joint_state_msg)
{
    // ROS_INFO_STREAM("Joint States:\n" << *joint_state_msg);
    current_joint_states = *joint_state_msg;
    called = true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "set_joint_angles");

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
        }
        ROS_INFO("my_pose before input %f",my_pose[joint]);
        ROS_WARN("Current joints: {\n    [0]%s:\t\t\t%f, \n    [1]%s:\t%f, \n    [2]%s:\t\t%f, \n    [3]%s:\t\t%f, \n"
                         "    [4]%s:\t\t\t%f, \n    [5]%s:\t\t\t%f, \n    [6]%s:\t\t\t%f}",
                 current_joint_states.name[0].c_str(), my_pose[0], current_joint_states.name[1].c_str(), my_pose[1],
                 current_joint_states.name[2].c_str(), my_pose[2], current_joint_states.name[3].c_str(), my_pose[3],
                 current_joint_states.name[4].c_str(), my_pose[4], current_joint_states.name[5].c_str(), my_pose[5],
                 current_joint_states.name[6].c_str(), my_pose[6]);

        //ROS_INFO("Target Joint Number");
        //ROS_INFO("Press 7 to set all");
std::cout<<"Press one to enter set absolute value mode"<<endl;
std::cout<<"Press two to enter set delta value mode"<<endl;
std::cout<<"Mode Code: ";
std::cin >> mode_code;


switch (mode_code) {
case 1:
	std::cout<<"Select the target Joint to set, press 7 to set all"<<endl;
        std::cout<<"Press 17 to set joints to preposition"<<endl;
        std::cout<<"Press 18 to move base in front of bins"<<endl;
        std::cout<<"Press 12 to face robot to bins from preposition orientation"<<endl;
	std::cout<<"Press 13 to stretch the arm"<<endl;

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
		std::cout << "auto setting to prepostion..";
		my_pose[0] =-2.841614;
		my_pose[2] =-1.581264;
		my_pose[3] =1.560937;
		my_pose[4] =4.379010;
		my_pose[5] =-0.083689;
		my_pose[6] =2.414974;
		break;

  	    case 12:
		std::cout <<"facing the bins..";
		my_pose[0] =-2.841614;
		my_pose[2] =-1.581264;
		my_pose[3] =-0.024400;
		my_pose[4] =4.379010;
		my_pose[5] =-0.083689;
		my_pose[6] =2.414974;
		break;

	    case 13:
		std::cout <<"stretching the arms over bins";
		my_pose[0] =-2.08;
		my_pose[2] =-2.47;
		my_pose[3] =-0.027;
		my_pose[4] =0.066393;
		my_pose[5] =1.683881;
		my_pose[6] =1.549463;
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



        //ROS_INFO("my_pose after input %f",my_pose[joint]);
        sendArmCommand(my_pose);
        //sendArmCommand(my_pose);
        //ros::Duration(0.2).sleep();                         // wait for finish
        ros::Duration(3).sleep();  
        ros::spinOnce();
        called = false;
        while(!called) {
            ros::spinOnce();
            ros::Duration(0.02).sleep();
        }
    }
    return 0;
}
