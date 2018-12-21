#include <iostream>
#include <math.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

using namespace std;

ros::Publisher cmd_vel_pub_p3dx1;

ros::Subscriber pose_sub_p3dx1;

geometry_msgs::Twist cmd_vel_msg_p3dx1;

float p3dx1_x   = 0;
float p3dx1_y   = 0;
float p3dx1_yaw = 0;

float vl_p3dx1 = 0;
float va_p3dx1 = 0;

float ux_p3dx1 = 0;
float uy_p3dx1 = 0;

float d = 0.2;

void get_pose_p3dx1 (const nav_msgs::Odometry::ConstPtr& msg)
{
        p3dx1_x = msg->pose.pose.position.x;
    	p3dx1_y = msg->pose.pose.position.y;

	double quatx = msg->pose.pose.orientation.x;
	double quaty = msg->pose.pose.orientation.y;
	double quatz = msg->pose.pose.orientation.z;
	double quatw = msg->pose.pose.orientation.w;

	tf::Quaternion q(quatx, quaty, quatz, quatw);
	tf::Matrix3x3 m(q);
    	double roll, pitch, yaw;
    	m.getRPY(roll, pitch, yaw);
	
	p3dx1_yaw = yaw;
}

void vel_lin_p3dx1 (float vl)
{
	cmd_vel_msg_p3dx1.linear.x = vl;
    	cmd_vel_pub_p3dx1.publish(cmd_vel_msg_p3dx1);
}

void vel_ang_p3dx1 (float va)
{
	cmd_vel_msg_p3dx1.angular.z = va;
	cmd_vel_pub_p3dx1.publish(cmd_vel_msg_p3dx1);
}

int main (int argc, char** argv)
{	
	ros::init(argc, argv, "p3dx1");
	ros::NodeHandle nh;

	cmd_vel_pub_p3dx1 = nh.advertise<geometry_msgs::Twist>("/p3dx1_cmd_vel",10);

	pose_sub_p3dx1 = nh.subscribe("/p3dx1_odom",10,get_pose_p3dx1);

	ros::spinOnce();
	ros::Rate loop_rate(10);

    	while(ros::ok())
	{
		ros::spinOnce();

		ux_p3dx1 = 0.05;
		uy_p3dx1 = 0.05;

		vl_p3dx1 = cos(p3dx1_yaw)*ux_p3dx1+sin(p3dx1_yaw)*uy_p3dx1;
		va_p3dx1 = (-sin(p3dx1_yaw)/d)*ux_p3dx1+(cos(p3dx1_yaw)/d)*uy_p3dx1;

		vel_lin_p3dx1(vl_p3dx1);
		vel_ang_p3dx1(va_p3dx1);;

		loop_rate.sleep();
	}
	
	return 0;
}
