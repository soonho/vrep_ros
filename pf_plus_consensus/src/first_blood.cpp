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

//inicializacao dos objetos
void initParams() 
{
    pf_p2.gain = 1.0;
    pf_p2.radius = 0.2;
    pf_p2.spread = 0.5;

    pf_p3.gain = 1.0;
    pf_p3.radius = 0.2;
    pf_p3.spread = 0.5;

    pf_q1.gain = 1.0;
    pf_q1.radius = 0.2;
    pf_q1.spread = 0.5;

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

//leis de controle: omniant1 + omniant2 + quad1 (ganhos dinamicos)
PotentialField consensus() {
    PotentialField temp;
    double ux = gain_x * (0.5 * ((pf_p2.x - 1.0) - (pf_p1.x + 1.0)) + 0.5 * ((pf_p3.x - 1.0) - (pf_p1.x + 1.0)) + 0.5 * ((pf_r1.x - 0.5) - (pf_p1.x + 1.0))) - gain_vx * (pf_p1.vx);
    double uy = gain_y * (0.5 * ((pf_p2.y - 0.5) - (pf_p1.y - 0.0)) + 0.5 * ((pf_p3.y + 0.5) - (pf_p1.y - 0.0)) + 0.5 * ((pf_r1.y - 0.0) - (pf_p1.y + 0.0))) - gain_vy * (pf_p1.vy);
    double uz = gain_z * (target_z   - pf_p1.z)                                                                  - gain_vz * (pf_p1.vz);
    //uyaw = gain_yaw * (0.5 * ((omniant1_yaw - 0.0) - (quad1_yaw - 0.0)) + 0.5 * ((omniant2_yaw - 0.0) - (quad1_yaw - 0.0))) - gain_vyaw * (quad1_vyaw);

    temp.x = ux;// ux * cos(quad1_yaw) + uy * sin(quad1_yaw);
    temp.y = uy;//-ux * sin(quad1_yaw) + uy * cos(quad1_yaw);
    temp.z = uz;
    return temp;
}

void robot_Callback(const nav_msgs::Odometry::ConstPtr& msg) 
{
    pf_p1.x  = (double) msg->pose.pose.position.x;
    pf_p1.y  = (double) msg->pose.pose.position.y;
    pf_p1.z  = (double) msg->pose.pose.position.z;
    pf_p1.vx = (double) msg->twist.twist.linear.x;
    pf_p1.vy = (double) msg->twist.twist.linear.y;
    pf_p1.vz = (double) msg->twist.twist.linear.z;

    //coleta dos parametros do ROS
    ros::NodeHandle n;
    n.getParam("/pf/x", goal.x);
    n.getParam("/pf/y", goal.y);
    n.getParam("/pf/gain", goal.gain);
    n.getParam("/pf/radius", goal.radius);
    n.getParam("/pf/spread", goal.spread);

    PotentialField temp = consensus();
    temp.add(temp.repForce(pf_o1, pf_p1));
    temp.add(temp.repForce(pf_o2, pf_p1));
    temp.add(temp.repForce(pf_o3, pf_p1));

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

    // iniciando parametros
    initParams();
    n.setParam("/pf/x", 0);
    n.setParam("/pf/y", 0);
    n.setParam("/pf/z", 0);
    n.setParam("/pf/gain", 1.0);
    n.setParam("/pf/radius", 1.0);
    n.setParam("/pf/spread", 1.0);

    // iniciando subscribers
    sub_p1 = n.subscribe("/odom_p1", 10, robot_Callback);
    sub_p2 = n.subscribe("/odom_p2", 10, p2_Callback);
    sub_p3 = n.subscribe("/odom_p3", 10, p3_Callback);
    sub_q1 = n.subscribe("/odom_q1", 10, q1_Callback);
    sub_q1 = n.subscribe("/odom_r1", 10, r1_Callback);
    sub_o1 = n.subscribe("/obst_01", 10, o1_Callback);
    sub_o2 = n.subscribe("/obst_02", 10, o2_Callback);
    sub_o3 = n.subscribe("/obst_03", 10, o3_Callback);
    ROS_INFO("Control for robot_01: online. First blood.");

    // inicializando advertisers
    pub_p1 = n.advertise<geometry_msgs::Quaternion>("/speed_01", 10);

    ros::spin();

    return 0;
}
