#include <iostream>
#include <math.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Accel.h>

using namespace std;

ros::Publisher cmd_vel_pub_p3dx1;
ros::Publisher cmd_accel_pub_quad1;
ros::Publisher cmd_accel_pub_quad2;

ros::Subscriber pose_sub_p3dx1;
ros::Subscriber pose_sub_quad1;
ros::Subscriber pose_sub_quad2;

geometry_msgs::Twist cmd_vel_msg_p3dx1;
geometry_msgs::Accel cmd_accel_msg_quad1;
geometry_msgs::Accel cmd_accel_msg_quad2;

float p3dx1_x   = 0;
float p3dx1_y   = 0;
float p3dx1_z   = 0;
float p3dx1_yaw = 0;

float quad1_x  = 0;
float quad1_y  = 0;
float quad1_z  = 0;
float quad1_vx = 0;
float quad1_vy = 0;
float quad1_vz = 0;

float quad2_x  = 0;
float quad2_y  = 0;
float quad2_z  = 0;
float quad2_vx = 0;
float quad2_vy = 0;
float quad2_vz = 0;

float vl_p3dx1 = 0;
float va_p3dx1 = 0;

float ux_p3dx1 = 0;
float uy_p3dx1 = 0;

float ux_quad1 = 0;
float uy_quad1 = 0;
float uz_quad1 = 0;

float ux_quad2 = 0;
float uy_quad2 = 0;
float uz_quad2 = 0;

float d = 0.2;

float dt = 0.05;

void get_pose_p3dx1 (const nav_msgs::Odometry::ConstPtr& msg)
{
        p3dx1_x = msg->pose.pose.position.x;
    	p3dx1_y = msg->pose.pose.position.y;
	p3dx1_z = msg->pose.pose.position.z;

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

void get_pose_quad1 (const nav_msgs::Odometry::ConstPtr& msg)
{
	quad1_x  = msg->pose.pose.position.x;
	quad1_y  = msg->pose.pose.position.y;
	quad1_z  = msg->pose.pose.position.z;
	quad1_vx = msg->twist.twist.linear.x;
	quad1_vy = msg->twist.twist.linear.y;
	quad1_vz = msg->twist.twist.linear.z;
}

void get_pose_quad2 (const nav_msgs::Odometry::ConstPtr& msg)
{
	quad2_x  = msg->pose.pose.position.x;
	quad2_y  = msg->pose.pose.position.y;
	quad2_z  = msg->pose.pose.position.z;
	quad2_vx = msg->twist.twist.linear.x;
	quad2_vy = msg->twist.twist.linear.y;
	quad2_vz = msg->twist.twist.linear.z;
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

void accel_x_quad1 (float accx)
{
	cmd_accel_msg_quad1.linear.x = accx;
	cmd_accel_pub_quad1.publish(cmd_accel_msg_quad1);
}

void accel_y_quad1 (float accy)
{
	cmd_accel_msg_quad1.linear.y = accy;
	cmd_accel_pub_quad1.publish(cmd_accel_msg_quad1);
}

void accel_z_quad1 (float accz)
{
	cmd_accel_msg_quad1.linear.z = accz;
	cmd_accel_pub_quad1.publish(cmd_accel_msg_quad1);
}

void accel_x_quad2 (float accx)
{
	cmd_accel_msg_quad2.linear.x = accx;
	cmd_accel_pub_quad2.publish(cmd_accel_msg_quad2);
}

void accel_y_quad2 (float accy)
{
	cmd_accel_msg_quad2.linear.y = accy;
	cmd_accel_pub_quad2.publish(cmd_accel_msg_quad2);
}

void accel_z_quad2 (float accz)
{
	cmd_accel_msg_quad2.linear.z = accz;
	cmd_accel_pub_quad2.publish(cmd_accel_msg_quad2);
}

int main (int argc, char** argv)
{	
	ros::init(argc, argv, "consensus");
	ros::NodeHandle nh;

	cmd_vel_pub_p3dx1 = nh.advertise<geometry_msgs::Twist>("/p3dx1_cmd_vel",10);
	cmd_accel_pub_quad1 = nh.advertise<geometry_msgs::Twist>("/quad1_cmd_accel",10);
	cmd_accel_pub_quad2 = nh.advertise<geometry_msgs::Twist>("/quad2_cmd_accel",10);

	pose_sub_p3dx1 = nh.subscribe("/p3dx1_odom",10,get_pose_p3dx1);
	pose_sub_quad1 = nh.subscribe("/quad1_odom",10,get_pose_quad1);
	pose_sub_quad2 = nh.subscribe("/quad2_odom",10,get_pose_quad2);

	ros::spinOnce();
	ros::Rate loop_rate(10);

    	while(ros::ok())
	{
		ros::spinOnce();

		ux_p3dx1 = 0.05*(- p3dx1_x + quad1_x + 0.5 - 0.0 - p3dx1_x + quad2_x + 0.5 - 1.0);
		uy_p3dx1 = 0.05*(- p3dx1_y + quad1_y + 0.0 + 0.0 - p3dx1_y + quad2_y + 0.0 + 0.0);		

		ux_quad1 = 0.05*(- quad1_x + p3dx1_x + 0.0 - 0.5 - quad1_x + quad2_x + 0.0 - 1.0 - 2*quad1_vx);
		uy_quad1 = 0.05*(- quad1_y + p3dx1_y + 0.0 - 0.0 - quad1_y + quad2_y + 0.0 - 0.0 - 2*quad1_vy);
		uz_quad1 = 0.50*(- quad1_z + p3dx1_z + 0.5 - 0.0 - quad1_z + quad2_z + 0.5 - 0.5 - 2*quad1_vz);

		ux_quad2 = 0.05*(- quad2_x + p3dx1_x + 1.0 - 0.5 - quad2_x + quad1_x + 1.0 - 0.0 - 2*quad2_vx);
		uy_quad2 = 0.05*(- quad2_y + p3dx1_y + 0.0 - 0.0 - quad2_y + quad1_y + 0.0 + 0.0 - 2*quad2_vy);
		uz_quad2 = 0.50*(- quad2_z + p3dx1_z + 0.5 - 0.0 - quad2_z + quad1_z + 0.5 - 0.5 - 2*quad2_vz);

		vl_p3dx1 = cos(p3dx1_yaw)*ux_p3dx1+sin(p3dx1_yaw)*uy_p3dx1;
		va_p3dx1 = (-sin(p3dx1_yaw)/d)*ux_p3dx1+(cos(p3dx1_yaw)/d)*uy_p3dx1;

		vel_lin_p3dx1(vl_p3dx1);
		vel_ang_p3dx1(va_p3dx1);

		accel_x_quad1(ux_quad1);
		accel_y_quad1(uy_quad1);
		accel_z_quad1(uz_quad1);

		accel_x_quad2(ux_quad2);
		accel_y_quad2(uy_quad2);
		accel_z_quad2(uz_quad2);

		loop_rate.sleep();
	}
	
	return 0;
}
