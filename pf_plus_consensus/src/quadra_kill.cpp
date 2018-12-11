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

// campos potenciais objetivo e de aliados
pfield goal;
pfield pf_p1;
pfield pf_p2;
pfield pf_p3;
pfield pf_q1;

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
    temp = addPfields(attForce(goal, pf_q1), repForce(pf_p2, pf_q1));
    temp = addPfields(temp, repForce(pf_p1, pf_q1));

    retorno.x = temp.x;
    retorno.y = temp.y;
    retorno.z = 0.0;
    retorno.w = 0.0;

    pub_p1.publish(retorno);
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
    n.setParam("/pf/x", 0);
    n.setParam("/pf/y", 0);
    n.setParam("/pf/z", 0);
    n.setParam("/pf/gain", 1.0);
    n.setParam("/pf/radius", 1.0);
    n.setParam("/pf/spread", 1.0);

    // iniciando subscribers
    sub_p1 = n.subscribe("/odom_p1", 10, p1_Callback);
    sub_p2 = n.subscribe("/odom_p2", 10, p2_Callback);
    sub_p3 = n.subscribe("/odom_p3", 10, p3_Callback);
    sub_q1 = n.subscribe("/odom_q1", 10, robot_Callback);
    ROS_INFO("Control for quad_01: online. Quadra kill.");

    // inicializando advertisers
    pub_p1 = n.advertise<geometry_msgs::Quaternion>("/acc_01", 10);

    ros::spin();

    return 0;
}
