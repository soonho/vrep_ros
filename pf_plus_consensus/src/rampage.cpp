#include "ros/ros.h"
#include <cmath>
#include "geometry_msgs/Quaternion.h"
#include "nav_msgs/Odometry.h"
#include "potential_fields.cpp"

// publisher do p3_01
ros::Publisher pub_r1;

// listeners dos outros p3
ros::Subscriber sub_p1;
ros::Subscriber sub_p2;
ros::Subscriber sub_p3;
ros::Subscriber sub_q1;
ros::Subscriber sub_o1;
ros::Subscriber sub_o2;
ros::Subscriber sub_o3;

// campos potenciais objetivo e de aliados/obstaculos
PotentialField goal;
PotentialField pf_p1;
PotentialField pf_p2;
PotentialField pf_p3;
PotentialField pf_r1;
PotentialField pf_q1;
PotentialField pf_o1;
PotentialField pf_o2;
PotentialField pf_o3;

// objetos para publicacao
nav_msgs::Odometry retorno;
int sequencer = 0;

//parametros para ganhos
float gain_x    = 0.15;
float gain_y    = 0.15;
float gain_z    = 0.05;
float gain_yaw  = 0.3;
float gain_vx   = 0.24;
float gain_vy   = 0.24;
float gain_vz   = 0.15;
float gain_vyaw = 0.15;
float target_z = 1.0;

//inicializacao dos objetos
void initParams() 
{
    pf_o1.gain = 1.0;
    pf_o1.radius = 1.0;
    pf_o1.spread = 1.0;

    pf_o2.gain = 1.0;
    pf_o2.radius = 1.0;
    pf_o2.spread = 1.0;

    pf_o3.gain = 1.0;
    pf_o3.radius = 1.0;
    pf_o3.spread = 1.0;
}

void p1_Callback(const nav_msgs::Odometry::ConstPtr& msg) 
{
    pf_p1.x = (double) msg->pose.pose.position.x;
    pf_p1.y = (double) msg->pose.pose.position.y;
    pf_p1.z = (double) msg->pose.pose.position.z;
    pf_p1.vx = (double) msg->twist.twist.linear.x;
    pf_p1.vy = (double) msg->twist.twist.linear.y;
    pf_p1.vz = (double) msg->twist.twist.linear.z;
}

void p2_Callback(const nav_msgs::Odometry::ConstPtr& msg) 
{
    pf_p2.x = (double) msg->pose.pose.position.x;
    pf_p2.y = (double) msg->pose.pose.position.y;
    pf_p2.z = (double) msg->pose.pose.position.z;
    pf_p2.vx = (double) msg->twist.twist.linear.x;
    pf_p2.vy = (double) msg->twist.twist.linear.y;
    pf_p2.vz = (double) msg->twist.twist.linear.z;
}

void p3_Callback(const nav_msgs::Odometry::ConstPtr& msg) 
{
    pf_p3.x = (double) msg->pose.pose.position.x;
    pf_p3.y = (double) msg->pose.pose.position.y;
    pf_p3.z = (double) msg->pose.pose.position.z;
    pf_p3.vx = (double) msg->twist.twist.linear.x;
    pf_p3.vy = (double) msg->twist.twist.linear.y;
    pf_p3.vz = (double) msg->twist.twist.linear.z;
}

void q1_Callback(const nav_msgs::Odometry::ConstPtr& msg) 
{
    pf_q1.x = (double) msg->pose.pose.position.x;
    pf_q1.y = (double) msg->pose.pose.position.y;
    pf_q1.z = (double) msg->pose.pose.position.z;
    pf_q1.vx = (double) msg->twist.twist.linear.x;
    pf_q1.vy = (double) msg->twist.twist.linear.y;
    pf_q1.vz = (double) msg->twist.twist.linear.z;
}

void o1_Callback(const nav_msgs::Odometry::ConstPtr& msg) 
{
    pf_o1.x = (double) msg->pose.pose.position.x;
    pf_o1.y = (double) msg->pose.pose.position.y;
    pf_o1.z = (double) msg->pose.pose.position.z;
}

void o2_Callback(const nav_msgs::Odometry::ConstPtr& msg) 
{
    pf_o2.x = (double) msg->pose.pose.position.x;
    pf_o2.y = (double) msg->pose.pose.position.y;
    pf_o2.z = (double) msg->pose.pose.position.z;
}

void o3_Callback(const nav_msgs::Odometry::ConstPtr& msg) 
{
    pf_o3.x = (double) msg->pose.pose.position.x;
    pf_o3.y = (double) msg->pose.pose.position.y;
    pf_o3.z = (double) msg->pose.pose.position.z;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "first_blood");
    ros::NodeHandle n;

    // iniciando subscribers
    sub_p1 = n.subscribe("/odom_p1", 10, p1_Callback);
    sub_p2 = n.subscribe("/odom_p2", 10, p2_Callback);
    sub_p3 = n.subscribe("/odom_p3", 10, p3_Callback);
    sub_q1 = n.subscribe("/odom_q1", 10, q1_Callback);
    sub_o1 = n.subscribe("/obst_01", 10, o1_Callback);
    sub_o2 = n.subscribe("/obst_02", 10, o2_Callback);
    sub_o3 = n.subscribe("/obst_03", 10, o3_Callback);
    ROS_INFO("Control for Neo_The_Chosen_One: online. Rampage.");

    // inicializando advertisers
    pub_r1 = n.advertise<nav_msgs::Odometry>("/odom_r1", 10);

    ros::Rate loop_rate(20);

    while (ros::ok()) {
        ros::spinOnce();

        pf_r1.x  = (pf_p1.x - 0.5 + pf_p2.x + 0.5 + pf_p3.x + 0.5) / 3;
        pf_r1.y  = (pf_p1.y - 0.5 + pf_p2.y + 0.5 + pf_p3.y + 0.5) / 3;
        pf_r1.z  = 0;

        //coleta dos parametros do ROS
        ros::NodeHandle n;
        n.getParam("/pf/x", goal.x);
        n.getParam("/pf/y", goal.y);
        n.getParam("/pf/gain", goal.gain);
        n.getParam("/pf/radius", goal.radius);
        n.getParam("/pf/spread", goal.spread);

        pf_r1.add(pf_r1.attForce(goal, pf_r1));
        pf_r1.add(pf_r1.repForce(pf_o1, pf_r1));
        pf_r1.add(pf_r1.repForce(pf_o2, pf_r1));
        pf_r1.add(pf_r1.repForce(pf_o3, pf_r1));

        retorno.child_frame_id = "frame_car";
        retorno.header.seq = sequencer++;
        retorno.header.frame_id = "world";
        retorno.pose.pose.position.x = pf_r1.x;
        retorno.pose.pose.position.y = pf_r1.y;
        retorno.pose.pose.position.z = 0.0;

        pub_r1.publish(retorno);

        loop_rate.sleep();
    }

    return 0;
}
