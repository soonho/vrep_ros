#include <iostream>
#include <math.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Accel.h>
#include <asctec_hl_comm/mav_ctrl.h>
#include <asctec_hl_comm/mav_ctrl_motors.h>
#include <asctec_hl_comm/GpsCustom.h>
#include <std_msgs/Int8.h>
#include <sensor_msgs/NavSatFix.h>

using namespace std;

ros::Publisher cmd_vel_pub_p3dx1;
ros::Publisher cmd_accel_pub_quad1;

ros::Subscriber pose_sub_p3dx1;
ros::Subscriber gps_sub_p3dx1;
ros::Subscriber pose_sub_quad1;
ros::Subscriber gps_sub_quad1;

geometry_msgs::Twist cmd_vel_msg_p3dx1;
//geometry_msgs::Accel cmd_accel_msg_quad1;
asctec_hl_comm::mav_ctrl cmd_accel_msg_quad1;

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

float vl_p3dx1 = 0;
float va_p3dx1 = 0;

float ux_p3dx1 = 0;
float uy_p3dx1 = 0;

float ux_quad1 = 0;
float uy_quad1 = 0;
float uz_quad1 = 0;

float d = 0.2;

float dt = 0.05;

void motorInputCallback(const std_msgs::Int8::ConstPtr& msg){
    asctec_hl_comm::mav_ctrl_motors::Request req;
    asctec_hl_comm::mav_ctrl_motors::Response res;
    req.startMotors = msg->data;
    ros::service::call("fcu/motor_control", req, res);
    std::cout << "motores ativos: " << (int)res.motorsRunning << std::endl;
}

void get_gps_p3dx1 (const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    p3dx1_x = msg->latitude * (111139);
	p3dx1_y = msg->longitude * (111139);
	p3dx1_z = msg->altitude;
}

void get_pose_p3dx1 (const nav_msgs::Odometry::ConstPtr& msg)
{
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

void get_gps_quad1 (const sensor_msgs::NavSatFix::ConstPtr& msg)
{
	quad1_x  = msg->latitude * (111139);
	quad1_y  = msg->longitude * (111139);
	quad1_z  = msg->altitude;
}

void get_pose_quad1 (const asctec_hl_comm::GpsCustom::ConstPtr& msg)
{
	quad1_vx = msg->velocity_x;
	quad1_vy = msg->velocity_y;
	//quad1_vz = msg->twist.twist.linear.z; //precisa saber do carlos como fazer
}

void print()
{
    ROS_INFO("---START---");
    ROS_INFO("Entradas P3: %f %f %f", p3dx1_x, p3dx1_y, p3dx1_z);
    ROS_INFO("Entradas Pelican: %f %f %f", quad1_x, quad1_y, quad1_z);
    ROS_INFO("Saidas P3: %f %f", vl_p3dx1, va_p3dx1);
    ROS_INFO("Saidas Pelican: %f %f %f", ux_quad1, uy_quad1, uz_quad1);
    ROS_INFO("----END----");
}

int main (int argc, char** argv)
{	
	ros::init(argc, argv, "consensus");
	ros::NodeHandle nh;

	cmd_vel_pub_p3dx1 = nh.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel",10);
	//cmd_accel_pub_quad1 = nh.advertise<geometry_msgs::Twist>("/quad1_cmd_accel",10);
	cmd_accel_pub_quad1 = nh.advertise<asctec_hl_comm::mav_ctrl> ("fcu/control", 1);

	gps_sub_p3dx1 = nh.subscribe("/mavros/global_position/global",10,get_gps_p3dx1);
	pose_sub_p3dx1 = nh.subscribe("/mavros/global_position/local",10,get_pose_p3dx1);
	//pose_sub_p3dx1 = nh.subscribe("/RosAria/pose",10,get_pose_p3dx1);
	gps_sub_quad1 = nh.subscribe("/fcu/gps",10,get_gps_quad1);
	pose_sub_quad1 = nh.subscribe("/fcu/gps_custom",10,get_pose_quad1);

	ros::spinOnce();
	ros::Rate loop_rate(10);

    	while(ros::ok())
	{
		ros::spinOnce();

		ux_p3dx1 = 0.1*(- p3dx1_x + quad1_x + 0   - 0);
		uy_p3dx1 = 0.1*(- p3dx1_y + quad1_y + 0   - 0);

		ux_quad1 = 0.1*(- quad1_x + p3dx1_x + 0   - 0 - 2*quad1_vx);
		uy_quad1 = 0.1*(- quad1_y + p3dx1_y + 0   - 0 - 2*quad1_vy);
		uz_quad1 = 0.1*(- quad1_z + p3dx1_z + 0.5 - 0 - 2*quad1_vz);

		vl_p3dx1 = cos(p3dx1_yaw)*ux_p3dx1+sin(p3dx1_yaw)*uy_p3dx1;
		va_p3dx1 = (-sin(p3dx1_yaw)/d)*ux_p3dx1+(cos(p3dx1_yaw)/d)*uy_p3dx1;

		cmd_vel_msg_p3dx1.linear.x = vl_p3dx1;
	    cmd_vel_msg_p3dx1.angular.z = va_p3dx1;
	    cmd_vel_pub_p3dx1.publish(cmd_vel_msg_p3dx1);

        //comandos no pelican
        if (ux_quad1 > 0.2) {
            ux_quad1 = 0.2;
        } else if (ux_quad1 < -0.2) {
            ux_quad1 = -0.2;
        }
        if (uy_quad1 > 0.2) {
            uy_quad1 = 0.2;
        } else if (uy_quad1 < -0.2) {
            uy_quad1 = -0.2;
        }
        cmd_accel_msg_quad1.x = ux_quad1;
        cmd_accel_msg_quad1.y = uy_quad1;
        cmd_accel_msg_quad1.z = 0.45;
        
        //comandos usados na simulacao
        //cmd_accel_msg_quad1.x = ux_quad1;
        //cmd_accel_msg_quad1.y = uy_quad1;
        //cmd_accel_msg_quad1.z = uz_quad1;

        //cmd_accel_msg_quad1.yaw = s_yaw;
        cmd_accel_msg_quad1.type = asctec_hl_comm::mav_ctrl::acceleration;
        cmd_accel_msg_quad1.v_max_xy = -1; // use max velocity from config
        cmd_accel_msg_quad1.v_max_z = -1;
	    cmd_accel_pub_quad1.publish(cmd_accel_msg_quad1);
        
        print();
        
		loop_rate.sleep();
	}
	
	return 0;
}
