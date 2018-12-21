#include "ros/ros.h"
#include <cmath>
#include "geometry_msgs/Quaternion.h"
#include "nav_msgs/Odometry.h"
#include "potential_fields.cpp"

// representacao de campos potenciais
struct pfield {
    double x;
    double y;
    double z;
    double gain;
    double radius;
    double spread;
	double vx;
	double vy;
	double vz;
};

// publisher do p3_01
ros::Publisher pub_p1;

// listeners dos outros p3
ros::Subscriber sub_p1;
ros::Subscriber sub_p2;
ros::Subscriber sub_p3;
ros::Subscriber sub_r1;
ros::Subscriber sub_q1;

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

// objeto para publicacao
geometry_msgs::Quaternion retorno;

//parametros para ganhos
float gain_x    = 0.5;
float gain_y    = 0.5;
float gain_z    = 0.05;
float gain_yaw  = 0.3;
float gain_vx   = 1.0;
float gain_vy   = 1.0;
float gain_vz   = 0.15;
float gain_vyaw = 0.15;
float target_z = 1.0;

void initParams() 
{
    pf_p2.gain = 1.0;
    pf_p2.radius = 1.0;
    pf_p2.spread = 1.0;

    pf_p1.gain = 1.0;
    pf_p1.radius = 1.0;
    pf_p1.spread = 1.0;

    pf_p3.gain = 1.0;
    pf_p3.radius = 1.0;
    pf_p3.spread = 1.0;
}

//leis de controle: omniant1 + omniant2 + quad1 (ganhos dinamicos)
PotentialField consensus() {
    PotentialField temp;
    double ux = gain_x * (0.5 * ((pf_p2.x - 1.0) - (pf_q1.x + 1.0)) + 0.5 * ((pf_p3.x - 1.0) - (pf_q1.x + 1.0))) - gain_vx * (pf_q1.vx);
    double uy = gain_y * (0.5 * ((pf_p2.y - 0.5) - (pf_q1.y - 0.0)) + 0.5 * ((pf_p3.y + 0.5) - (pf_q1.y - 0.0))) - gain_vy * (pf_q1.vy);
    double uz = gain_z * (target_z   - pf_q1.z)                                                                  - gain_vz * (pf_q1.vz);
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
    temp.add(temp.repForce(pf_o1, pf_q1));
    temp.add(temp.repForce(pf_o2, pf_q1));
    temp.add(temp.repForce(pf_o3, pf_q1));

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

void p3_Callback(const nav_msgs::Odometry::ConstPtr& msg) 
{
    pf_p3.x = (double) msg->pose.pose.position.x;
    pf_p3.y = (double) msg->pose.pose.position.y;
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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "first_blood");
    ros::NodeHandle n;

    // iniciando parametros
    initParams();

    // iniciando subscribers
    sub_p1 = n.subscribe("/odom_p1", 10, p1_Callback);
    sub_p2 = n.subscribe("/odom_p2", 10, p2_Callback);
    sub_p3 = n.subscribe("/odom_p3", 10, p3_Callback);
    sub_q1 = n.subscribe("/odom_q1", 10, robot_Callback);
    sub_q1 = n.subscribe("/odom_r1", 10, r1_Callback);
    ROS_INFO("Control for quad_01: online. Quadra kill.");

    // inicializando advertisers
    pub_p1 = n.advertise<geometry_msgs::Quaternion>("/acc_01", 10);

    ros::spin();

    return 0;
}
