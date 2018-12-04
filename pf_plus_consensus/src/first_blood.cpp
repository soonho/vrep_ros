#include "ros/ros.h"
#include <cmath>
#include <vector>
#include "geometry_msgs/Twist.h"
#include "pf_plus_consensus/PotentialField.h"

struct pfield {
    double x;
    double y;
    double gain;
    double radius;
    double spread;
};

// publisher do p3_01
ros::Publisher pub_p1;

// listeners dos outros p3
ros::Subscriber sub_p2;
ros::Subscriber sub_p3;

// campos potenciais objetivo e de aliados
pfield goal;
pfield pf_p1;
pfield pf_p2;
pfield pf_p3;

// objeto para publicacao
geometry_msgs::Twist retorno;

void init() 
{
    goal.x = 0.0;
    goal.y = 0.0;
    goal.gain = 1.0;
    goal.radius = 0.5;
    goal.spread = 0.5;
}

// calculo da forca exercida pelo ponto objetivo
void goalForce()
{
    double distance = std::pow(std::pow(goal.x-pf_p3_01.x, 2)+std::pow(goal.y-pf_p3_01.y, 2), 0.5);
    double psi = std::atan2(goal.y-pf_p3_01.y, goal.x-pf_p3_01.x);
    double deltaX, deltaY;
    if (distance < goal.radius) {
        deltaX = 0;
        deltaY = 0;
    } else if (distance <= goal.spread + goal.radius) {
        deltaX = goal.gain * (distance - goal.radius) * std::cos(psi);
        deltaY = goal.gain * (distance - goal.radius) * std::sin(psi);
    } else {
        deltaX = goal.gain * goal.spread * std::cos(psi);
        deltaY = goal.gain * goal.spread * std::sin(psi);
    }
    retorno.linear.x += deltaX;
    retorno.linear.y += deltaY;
    retorno.linear.z += 0.0;
}

// calculo da forca exercida por um obstaculo
void obstacleForce(pfield obs)
{
    double distance = std::pow(std::pow(robotX-obs.x, 2)+std::pow(robotY-obs.y, 2), 0.5);
    double psi = std::atan2(obs.y-robotY, obs.x-robotX);
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
    force.linear.x += deltaX;
    force.linear.y += deltaY;
    force.linear.z += 0.0;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "potential_wizard");
    ros::NodeHandle n;

    // ros::ServiceServer service_02 = n.advertiseService("add_obstacle", addObstacle);
    // ROS_INFO("Obstacle adding service: online.");

    ros::Subscriber robot_01_listener = n.subscribe("robot_call", 100, robot_Callback);
    ROS_INFO("Control for robot_01: online.\nMay the Force be with you.");

    // inicializando subscribers
    sub_p1 = n.subscribe<geometry_msgs::Twist>("speed_01", 10);
    sub_p2 = n.subscribe<geometry_msgs::Twist>("speed_02", 10);
    sub_p3 = n.subscribe<geometry_msgs::Twist>("speed_03", 10);

    ros::spin();

    return 0;
}
