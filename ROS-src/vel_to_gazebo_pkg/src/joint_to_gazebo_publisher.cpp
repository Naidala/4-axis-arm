using namespace std;

#include <cmath>
#include <cstring>
#include <string>
#include <ros/ros.h>
#include <joint_pkg/Data_Joints_EE.h>
#include <tf/transform_broadcaster.h>
#include <iostream>
#include <std_msgs/Float64.h>

float pos_joint1;
float pos_joint2;
float pos_joint3;
float pos_jointEe;

/****Subscriber Callback Function****/
void coord_Callback(const joint_pkg::Data_Joints_EE & msg)
{
	pos_joint1 = msg.joint_state.position[0];
	pos_joint2 = msg.joint_state.position[1];
	pos_joint3 = msg.joint_state.position[2];
	pos_jointEe = msg.joint_state.position[3];
}



int main(int argc, char** argv) 
{

    ros::init(argc, argv, "desired_pos");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("joint_states", 1, coord_Callback);

    ros::Publisher joint1_pos = n.advertise<std_msgs::Float64>("/rrbot/joint1_position_controller/command", 1000);
    ros::Publisher joint2_pos = n.advertise<std_msgs::Float64>("/rrbot/joint2_position_controller/command", 1000);
    ros::Publisher joint3_pos = n.advertise<std_msgs::Float64>("/rrbot/joint3_position_controller/command", 1000);
    ros::Publisher jointEe_pub = n.advertise<std_msgs::Float64>("/rrbot/jointEe_position_controller/command", 1000);

    ros::Rate loop_rate(20);

    std_msgs::Float64 pos_command;

    while (ros::ok()) 
    {
	pos_command.data = pos_joint1;
	joint1_pos.publish(pos_command);
	pos_command.data = pos_joint2;
	joint2_pos.publish(pos_command);
	pos_command.data = pos_joint3;
	joint3_pos.publish(pos_command);
	pos_command.data = pos_jointEe;
	jointEe_pub.publish(pos_command);

        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}



