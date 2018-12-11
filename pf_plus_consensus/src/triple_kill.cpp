#include "ros/ros.h"
#include <cmath>
#include <vector>
#include "geometry_msgs/Quaternion.h"
#include "nav_msgs/Odometry.h"
#include "pf_plus_consensus/PotentialField.h"

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
ros::Subscriber sub_q1;
ros::Subscriber sub_o1;
ros::Subscriber sub_o2;
ros::Subscriber sub_o3;

// campos potenciais objetivo e de aliados
pfield goal;
pfield pf_p1;
pfield pf_p2;
pfield pf_p3;
pfield pf_q1;
pfield pf_o1;
pfield pf_o2;
pfield pf_o3;

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

// calculo da forca exercida pelo ponto objetivo
pfield attForce(pfield target, pfield robot)
{
    pfield temp;
    double distance = std::pow(std::pow(target.x-robot.x, 2)+std::pow(target.y-robot.y, 2), 0.5);
    double psi = std::atan2(target.y-robot.y, target.x-robot.x);
    double deltaX, deltaY;
    if (distance < target.radius) {
        deltaX = 0;
        deltaY = 0;
    } else if (distance <= target.spread + target.radius) {
        deltaX = target.gain * (distance - target.radius) * std::cos(psi);
        deltaY = target.gain * (distance - target.radius) * std::sin(psi);
    } else {
        deltaX = target.gain * target.spread * std::cos(psi);
        deltaY = target.gain * target.spread * std::sin(psi);
    }
    temp.x = deltaX;
    temp.y = deltaY;
    temp.z = 0.0;
    return temp;
}

// calculo da forca exercida por um obstaculo
pfield repForce(pfield obs, pfield robot)
{
    pfield temp;
    double distance = std::pow(std::pow(robot.x-obs.x, 2)+std::pow(robot.y-obs.y, 2), 0.5);
    double psi = std::atan2(obs.y-robot.y, obs.x-robot.x);
    double deltaX, deltaY;
    if (distance > obs.spread + obs.radius) {
        deltaX = 0;
        deltaY = 0;
    } else if (distance >= obs.radius) {
        deltaX = -obs.gain * (obs.spread + obs.radius - distance) * std::cos(psi);
        deltaY = -obs.gain * (obs.spread + obs.radius - distance) * std::sin(psi);
    } else {
        deltaX = -copysign(1.0, std::cos(psi)) * 9999;
        deltaY = -copysign(1.0, std::sin(psi)) * 9999;
    }
    temp.x = deltaX;
    temp.y = deltaY;
    temp.z = 0.0;
    return temp;
}
//leis de controle: omniant1 + omniant2 + quad1 (ganhos dinamicos)
pfield consensus() {
    pfield temp;
    double ux = gain_x * (0.5 * ((pf_p1.x + 1.0) - (pf_p3.x + 0.0)) + 0.5 * ((pf_p2.x + 0.0) - (pf_p3.x + 0.0))) - gain_vx * (pf_p1.vx);
    double uy = gain_y * (0.5 * ((pf_p1.y - 0.5) - (pf_p3.y + 1.0)) + 0.5 * ((pf_p2.y - 1.0) - (pf_p3.y + 1.0))) - gain_vy * (pf_p1.vy);
    double uz = gain_z * (target_z   - pf_p1.z)                                                                  - gain_vz * (pf_p1.vz);
    //uyaw = gain_yaw * (0.5 * ((omniant1_yaw - 0.0) - (quad1_yaw - 0.0)) + 0.5 * ((omniant2_yaw - 0.0) - (quad1_yaw - 0.0))) - gain_vyaw * (quad1_vyaw);

    temp.x = ux;// ux * cos(quad1_yaw) + uy * sin(quad1_yaw);
    temp.y = uy;//-ux * sin(quad1_yaw) + uy * cos(quad1_yaw);
    temp.z = uz;
    return temp;
}

// funcao para somar dois campos potenciais
pfield addPfields(pfield a, pfield b) 
{
    pfield temp;
    temp.x = a.x + b.x;
    temp.y = a.y + b.y;
    temp.z = a.z + b.z;
    return temp;
}

void robot_Callback(const nav_msgs::Odometry::ConstPtr& msg) 
{
    pf_p3.x = (double) msg->pose.pose.position.x;
    pf_p3.y = (double) msg->pose.pose.position.y;
    pf_p3.z = (double) msg->pose.pose.position.z;

    //coleta dos parametros do ROS
    ros::NodeHandle n;
    if (!n.getParam("/pf/x", goal.x)) {
        goal.x = 0;
    }
    if (!n.getParam("/pf/y", goal.y)) {
        goal.y = 0;
    }
    if (!n.getParam("/pf/gain", goal.gain)) {
        goal.gain = 1.0;
    }
    if (!n.getParam("/pf/radius", goal.radius)) {
        goal.radius = 1.0;
    }
    if (!n.getParam("/pf/spread", goal.spread)) {
        goal.spread = 1.0;
    }

    pfield temp;
    //temp = addPfields(attForce(goal, pf_p3), repForce(pf_p2, pf_p3));
    //temp = addPfields(temp, repForce(pf_p1, pf_p3));
    //temp = addPfields(temp, repForce(pf_o1, pf_p3));
    //temp = addPfields(temp, repForce(pf_o2, pf_p3));
    //temp = addPfields(temp, repForce(pf_o3, pf_p3));
    //temp = addPfields(temp, consensus());
	temp = consensus();


    retorno.x = temp.x;
    retorno.y = temp.y;
    retorno.z = 0.0;
    retorno.w = 0.0;

    pub_p1.publish(retorno);
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
    n.setParam("/pf/x", 0);
    n.setParam("/pf/y", 0);
    n.setParam("/pf/z", 0);
    n.setParam("/pf/gain", 1.0);
    n.setParam("/pf/radius", 1.0);
    n.setParam("/pf/spread", 1.0);

    // iniciando subscribers
    sub_p1 = n.subscribe("/odom_p1", 10, p1_Callback);
    sub_p2 = n.subscribe("/odom_p2", 10, p2_Callback);
    sub_p3 = n.subscribe("/odom_p3", 10, robot_Callback);
    sub_q1 = n.subscribe("/odom_q1", 10, q1_Callback);
    sub_o1 = n.subscribe("/obst_01", 10, o1_Callback);
    sub_o2 = n.subscribe("/obst_02", 10, o2_Callback);
    sub_o3 = n.subscribe("/obst_03", 10, o3_Callback);
    ROS_INFO("Control for robot_03: online. Triple kill.");

    // inicializando advertisers
    pub_p1 = n.advertise<geometry_msgs::Quaternion>("/speed_03", 10);

    ros::spin();

    return 0;
}
