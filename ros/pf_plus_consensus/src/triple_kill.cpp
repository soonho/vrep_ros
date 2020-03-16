#include "ros/ros.h"
#include <cmath>
#include "geometry_msgs/Quaternion.h"
#include "nav_msgs/Odometry.h"
#include "potential_fields.cpp"

// publisher do p3_01
ros::Publisher pub_p1;

// listeners dos outros p3
ros::Subscriber sub_p1;
ros::Subscriber sub_p2;
ros::Subscriber sub_p3;
ros::Subscriber sub_q1;
ros::Subscriber sub_r1;
ros::Subscriber sub_o1;
ros::Subscriber sub_o2;
ros::Subscriber sub_o3;

// campos potenciais objetivo e de aliados
PotentialField goal;
PotentialField pf_p1;
PotentialField pf_p2;
PotentialField pf_p3;
PotentialField pf_r1;
PotentialField pf_q1;
PotentialField pf_o1;
PotentialField pf_o2;
PotentialField pf_o3;
PotentialField pf_o4;
PotentialField pf_o5;

// objeto para publicacao
geometry_msgs::Quaternion retorno;

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

void initParams() 
{
    pf_p2.gain = 1.0;
    pf_p2.radius = 0.2;
    pf_p2.spread = 0.5;

    pf_p1.gain = 1.0;
    pf_p1.radius = 0.2;
    pf_p1.spread = 0.5;

    pf_q1.gain = 1.0;
    pf_q1.radius = 0.2;
    pf_q1.spread = 0.5;

    pf_o1.x = -3.07;
    pf_o1.y = -0.67;
    pf_o1.gain = 1.0;
    pf_o1.radius = 0.3;
    pf_o1.spread = 0.5;

    pf_o2.x = -1.95;
    pf_o2.y = 2.21;
    pf_o2.gain = 1.0;
    pf_o2.radius = 0.3;
    pf_o2.spread = 0.5;

    pf_o3.x = 0.04;
    pf_o3.y = -1.93;
    pf_o3.gain = 1.0;
    pf_o3.radius = 0.3;
    pf_o3.spread = 0.5;

    pf_o4.x = 0.57;
    pf_o4.y = 0.66;
    pf_o4.gain = 1.0;
    pf_o4.radius = 0.3;
    pf_o4.spread = 0.5;

    pf_o5.x = 3.09;
    pf_o5.y = -0.88;
    pf_o5.gain = 1.0;
    pf_o5.radius = 0.3;
    pf_o5.spread = 0.5;
}

//leis de controle: omniant1 + omniant2 + quad1 (ganhos dinamicos)
PotentialField consensus() {
    PotentialField temp;
    double ux = gain_x * (0.5 * ((pf_p1.x + 1.0) - (pf_p3.x + 0.0)) + 0.5 * ((pf_p2.x + 0.0) - (pf_p3.x + 0.0)) + 0.5 * ((pf_r1.x + 0.5) - (pf_p3.x + 0.0))) - gain_vx * (pf_p3.vx);
    double uy = gain_y * (0.5 * ((pf_p1.y - 0.5) - (pf_p3.y + 1.0)) + 0.5 * ((pf_p2.y - 1.0) - (pf_p3.y + 1.0)) + 0.5 * ((pf_r1.y - 0.5) - (pf_p3.y - 1.0))) - gain_vy * (pf_p3.vy);
    double uz = gain_z * (target_z   - pf_p3.z)                                                                  - gain_vz * (pf_p1.vz);
    //uyaw = gain_yaw * (0.5 * ((omniant1_yaw - 0.0) - (quad1_yaw - 0.0)) + 0.5 * ((omniant2_yaw - 0.0) - (quad1_yaw - 0.0))) - gain_vyaw * (quad1_vyaw);

    temp.x = ux;// ux * cos(quad1_yaw) + uy * sin(quad1_yaw);
    temp.y = uy;//-ux * sin(quad1_yaw) + uy * cos(quad1_yaw);
    temp.z = uz;
    return temp;
}

void robot_Callback(const nav_msgs::Odometry::ConstPtr& msg) 
{
    pf_p3.x = (double) msg->pose.pose.position.x;
    pf_p3.y = (double) msg->pose.pose.position.y;
    pf_p3.z = (double) msg->pose.pose.position.z;

    PotentialField temp = consensus();
    temp.add(pf_r1.repForce(pf_o1, pf_p3));
    temp.add(pf_r1.repForce(pf_o2, pf_p3));
    temp.add(pf_r1.repForce(pf_o3, pf_p3));
    temp.add(pf_r1.repForce(pf_o4, pf_p3));
    temp.add(pf_r1.repForce(pf_o5, pf_p3));

    retorno.x = temp.x;
    retorno.y = temp.y;
    retorno.z = 0.0;
    retorno.w = 0.0;

    pub_p1.publish(retorno);
}

void r1_Callback(const nav_msgs::Odometry::ConstPtr& msg) 
{
    pf_r1.x = (double) msg->pose.pose.position.x;
    pf_r1.y = (double) msg->pose.pose.position.y;
    pf_r1.z = (double) msg->pose.pose.position.z;
    pf_r1.vx = (double) msg->twist.twist.linear.x;
    pf_r1.vy = (double) msg->twist.twist.linear.y;
    pf_r1.vz = (double) msg->twist.twist.linear.z;
}

void p2_Callback(const nav_msgs::Odometry::ConstPtr& msg) 
{
    pf_p2.x = (double) msg->pose.pose.position.x;
    pf_p2.y = (double) msg->pose.pose.position.y;
}

void p1_Callback(const nav_msgs::Odometry::ConstPtr& msg) 
{
    pf_p1.x = (double) msg->pose.pose.position.x;
    pf_p1.y = (double) msg->pose.pose.position.y;
}

void q1_Callback(const nav_msgs::Odometry::ConstPtr& msg) 
{
    pf_q1.x = (double) msg->pose.pose.position.x;
    pf_q1.y = (double) msg->pose.pose.position.y;
    pf_q1.z = (double) msg->pose.pose.position.z;
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

    // iniciando parametros
    initParams();

    // iniciando subscribers
    sub_p1 = n.subscribe("/odom_p1", 10, p1_Callback);
    sub_p2 = n.subscribe("/odom_p2", 10, p2_Callback);
    sub_p3 = n.subscribe("/odom_p3", 10, robot_Callback);
    sub_q1 = n.subscribe("/odom_q1", 10, q1_Callback);
    sub_q1 = n.subscribe("/odom_r1", 10, r1_Callback);
    sub_o1 = n.subscribe("/obst_01", 10, o1_Callback);
    sub_o2 = n.subscribe("/obst_02", 10, o2_Callback);
    sub_o3 = n.subscribe("/obst_03", 10, o3_Callback);
    ROS_INFO("Control for robot_03: online. Triple kill.");

    // inicializando advertisers
    pub_p1 = n.advertise<geometry_msgs::Quaternion>("/speed_03", 10);

    ros::spin();

    return 0;
}
