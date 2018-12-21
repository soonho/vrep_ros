#include <iostream>
#include <math.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Accel.h>
#include <asctec_hl_comm/mav_ctrl.h>
#include <asctec_hl_comm/mav_ctrl_motors.h>
#include <asctec_hl_comm/mav_imu.h>
#include <asctec_hl_comm/GpsCustom.h>
#include <std_msgs/Int8.h>
#include <sensor_msgs/NavSatFix.h>

using namespace std;

ros::Publisher cmd_accel_pub_quad1;

ros::Subscriber imu_sub_quad1;
ros::Subscriber pose_sub_quad1;
ros::Subscriber gps_sub_quad1;

asctec_hl_comm::mav_ctrl cmd_accel_msg_quad1;

float fixed_x = 0;
float fixed_y = 0;
float fixed_z = 0;

float quad1_x  = 0;
float quad1_y  = 0;
float quad1_z  = 0;
float quad1_vx = 0;
float quad1_vy = 0;
float quad1_vz = 0;

float ux_quad1 = 0;
float uy_quad1 = 0;
float uz_quad1 = 0;

float d = 0.2;

float dt = 0.05;

float max_ = 0.4;

void motorInputCallback(const std_msgs::Int8::ConstPtr& msg){
    asctec_hl_comm::mav_ctrl_motors::Request req;
    asctec_hl_comm::mav_ctrl_motors::Response res;
    req.startMotors = msg->data;
    ros::service::call("fcu/motor_control", req, res);
    std::cout << "motores ativos: " << (int)res.motorsRunning << std::endl;
}

void get_gps_quad1 (const asctec_hl_comm::GpsCustom::ConstPtr& msg)
{
	quad1_x  = msg->latitude;// * (111139);
	quad1_y  = msg->longitude;// * (111139);
	
	quad1_vx = msg->velocity_x;
	quad1_vy = msg->velocity_y;
	//quad1_vz = msg->twist.twist.linear.z; //precisa saber do carlos como fazer
}

void get_pose_quad1 (const asctec_hl_comm::mav_imu::ConstPtr& msg)
{
	quad1_z  = msg->height;
}

void print()
{
    ROS_INFO("---START---");
    ROS_INFO("Entradas Pelican: %f %f %f", quad1_x, quad1_y, quad1_z);
    ROS_INFO("Entradas Pelican: %f %f %f", fixed_x, fixed_y, fixed_z);
    ROS_INFO("Saidas Pelican: %f %f %f", ux_quad1, uy_quad1, uz_quad1);
    ROS_INFO("----END----");
}

int main (int argc, char** argv)
{	
	ros::init(argc, argv, "consensus");
	ros::NodeHandle nh;

	cmd_accel_pub_quad1 = nh.advertise<asctec_hl_comm::mav_ctrl> ("fcu/control", 1);
	
	gps_sub_quad1 = nh.subscribe("/fcu/gps_custom",10,get_gps_quad1);
	pose_sub_quad1 = nh.subscribe("/fcu/imu_custom",10,get_pose_quad1);

	ros::spinOnce();
	ros::Rate loop_rate(10);

    while(ros::ok())
	{
		ros::spinOnce();
		
		nh.getParam("/fixed/gps_ref_latitude", fixed_x);
        nh.getParam("/fixed/gps_ref_longitude", fixed_y);
		nh.getParam("/fixed/gps_ref_altitude", fixed_z);
		
		/*
		fixed_x = fixed_x * (111139);
		fixed_y = fixed_y * (111139);
		fixed_z = fixed_z * (111139);
		*/
		
		ux_quad1 = 0.1*(- quad1_x + fixed_x + 0 - 0 - 2*quad1_vx);
		uy_quad1 = 0.1*(- quad1_y + fixed_y + 0 - 0 - 2*quad1_vy);
		uz_quad1 = 0.1*(- quad1_z + fixed_z + 2 - 0 - 2*quad1_vz);

        //comandos no pelican
        if (ux_quad1 > max_) {
            ux_quad1 = max_;
        } else if (ux_quad1 < -max_) {
            ux_quad1 = -max_;
        }
        if (uy_quad1 > max_) {
            uy_quad1 = max_;
        } else if (uy_quad1 < -max_) {
            uy_quad1 = -max_;
        }
        cmd_accel_msg_quad1.x = ux_quad1;
        cmd_accel_msg_quad1.y = uy_quad1;
        cmd_accel_msg_quad1.z = 0.45;
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
