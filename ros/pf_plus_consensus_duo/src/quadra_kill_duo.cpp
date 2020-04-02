#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include "tf/LinearMath/Matrix3x3.h"
#include <cmath>
#include <math.h>
#include "nav_msgs/Odometry.h"
#include "potential_fields_duo.cpp"

using namespace std;

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

//parametros para limitadores e offsets
float max_accxy  = 15;
float max_accz   = 0.5;
float min_accz   = 0.465;
float max_uyaw   = 0.8;
float offset_z   = 0.465;
float offset_yaw = 0.5;

//variaveis para o calculo de saturacao
float rmax = 0, rx = 0, ry = 0, rz = 0, ryaw = 0, pf_p3_yaw = 0;

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

    goal.gain = 1.0;
    goal.radius = 0.2;
    goal.spread = 0.5;
}

//leis de controle: omniant1 + omniant2 + quad1 (ganhos dinamicos)
PotentialField consensus() {
    PotentialField temp;
    double ux = gain_x * (0.5 * ((pf_p1.x - 0.0) - (pf_q1.x + 0.0))) - gain_vx * (pf_q1.vx);
    double uy = gain_y * (0.5 * ((pf_p1.y - 0.0) - (pf_q1.y - 0.0))) - gain_vy * (pf_q1.vy);
    double uz = gain_z * (target_z   - pf_q1.z)                                                                  - gain_vz * (pf_q1.vz);
    //uyaw = gain_yaw * (0.5 * ((omniant1_yaw - 0.0) - (quad1_yaw - 0.0)) + 0.5 * ((omniant2_yaw - 0.0) - (quad1_yaw - 0.0))) - gain_vyaw * (quad1_vyaw);

    temp.x = ux;//ux * cos(pf_p3_yaw) + uy * sin(pf_p3_yaw);
    temp.y = uy;//-ux * sin(pf_p3_yaw) + uy * cos(pf_p3_yaw);
    temp.z = uz;
    return temp;
}

void robot_Callback(const nav_msgs::Odometry::ConstPtr& msg) 
{
    pf_q1.x = (double) msg->pose.pose.position.x;
    pf_q1.y = (double) msg->pose.pose.position.y;
    pf_q1.z = (double) msg->pose.pose.position.z;
    pf_p3_yaw = msg->pose.pose.orientation.w;
/*
    retorno.x = (double) msg->pose.pose.orientation.x;
    retorno.y = (double) msg->pose.pose.orientation.y;
    retorno.z = (double) msg->pose.pose.orientation.z;
    retorno.w = (double) msg->pose.pose.orientation.w;

    // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
    tf::Quaternion quat;
    tf::quaternionMsgToTF(retorno, quat);

    // the tf::Quaternion has a method to acess roll pitch and yaw
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    pf_p3_yaw = yaw;
*/
    PotentialField temp = consensus();
    //temp.add(temp.repForce(pf_o1, pf_q1));
    //temp.add(temp.repForce(pf_o2, pf_q1));
    //temp.add(temp.repForce(pf_o3, pf_q1));

    //PotentialField temp;
    //temp.add(temp.attForce(pf_r1, pf_q1));
    //temp.x = temp.x * cos(pf_p3_yaw) + temp.y * sin(pf_p3_yaw);
    //temp.y = -temp.x * sin(pf_p3_yaw) + temp.y * cos(pf_p3_yaw);
/*
    //saturacao
    rx = abs(temp.x / max_accxy);
    ry = abs(temp.y / max_accxy);

    rmax = max(rx, ry);

    if (rmax > 1) {
        rx = temp.x / rmax;
        ry = temp.y / rmax;
    }
*/
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
    ros::init(argc, argv, "first_blood_duo");
    ros::NodeHandle n;

    // iniciando parametros
    initParams();

    // iniciando subscribers
    sub_p1 = n.subscribe("/odom_p1", 10, p1_Callback);
    sub_p2 = n.subscribe("/odom_p2", 10, p2_Callback);
    sub_p3 = n.subscribe("/odom_p3", 10, p3_Callback);
    sub_q1 = n.subscribe("/odom_q1", 10, robot_Callback);
    sub_r1 = n.subscribe("/odom_r1", 10, r1_Callback);
    ROS_INFO("Control for quad_01: online. Quadra kill.");

    // inicializando advertisers
    pub_p1 = n.advertise<geometry_msgs::Quaternion>("/acc_01", 10);

    ros::spin();

    return 0;
}