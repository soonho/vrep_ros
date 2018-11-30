#include <ros/ros.h>
#include <cmath>
#include <vector>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <potential_field_robot/Force.h>
#include <potential_field_robot/ForceResponse.h>
#include <potential_field_robot/PotentialField.h>
#include <potential_field_robot/PotentialFieldRequest.h>

struct obstacle {
  double x;
  double y;
  double alpha;
  double beta;
  double radius;
  double spread;
};

potential_field_robot::PotentialFieldRequest goal;
geometry_msgs::Twist force;
std::vector<obstacle> obstaculos;
std::vector<ros::Publisher> publishers;
int obstacle_index = 0;

bool addObstacle(potential_field_robot::PotentialField::Request  &req,
          potential_field_robot::PotentialField::Response &res) 
{
  obstaculos.push_back(obstacle());
  
  obstaculos[obstacle_index].x = req.x;
  obstaculos[obstacle_index].y = req.y;
  obstaculos[obstacle_index].alpha = req.a;
  obstaculos[obstacle_index].beta = req.b;
  obstaculos[obstacle_index].radius = req.r;
  obstaculos[obstacle_index].spread = req.s;

  obstacle_index++;

  ROS_INFO("obstacle_%d: x=%f, y=%f, alpha=%f, beta=%f, radius=%f, spread=%f", obstacle_index, (double)req.x,(double) req.y, (double)req.a, (double) req.b, (double) req.r, (double) req.s);

  return true;
}

bool setGoal(potential_field_robot::PotentialField::Request  &req,
          potential_field_robot::PotentialField::Response &res) 
{
  goal.x = req.x;
  goal.y = req.y;
  goal.a = req.a;
  goal.b = req.b;
  goal.r = req.r;
  goal.s = req.s;

  ROS_INFO("objective: x=%f, y=%f, alpha=%f, beta=%f, radius=%f, spread=%f", (double)req.x,(double) req.y, (double)req.a, (double) req.b, (double) req.r, (double) req.s);

  return true;
}

void goalForce(double robotX, double robotY, double robotZ)
{
  double distance = std::pow(std::pow(goal.x-robotX, 2)+std::pow(goal.y-robotY, 2), 0.5);
  double psi = std::atan2(goal.y-robotY, goal.x-robotX);
  double deltaX, deltaY;
  if (distance < goal.r) {
     deltaX = 0;
     deltaY = 0;
  } else if (distance <= goal.s + goal.r) {
     deltaX = goal.a * (distance - goal.r) * std::cos(psi);
     deltaY = goal.a * (distance - goal.r) * std::sin(psi);
  } else {
     deltaX = goal.a * goal.s * std::cos(psi);
     deltaY = goal.a * goal.s * std::sin(psi);
  }
  force.linear.x += deltaX;
  force.linear.y += deltaY;
  force.linear.z += 0.0;

//  ROS_INFO("Robot: x=%f, y=%f, z=%f", (double) robotX, (double) robotY, (double) robotZ);
//  ROS_INFO("Goal force: [dX=%f, dY=%f]", (double) force.linear.x, (double) force.linear.y);
}

void obstacleForce(obstacle obs, double robotX, double robotY, double robotZ)
{
  double distance = std::pow(std::pow(robotX-obs.x, 2)+std::pow(robotY-obs.y, 2), 0.5);
  double psi = std::atan2(obs.y-robotY, obs.x-robotX);
  double deltaX, deltaY;
  if (distance > obs.spread + obs.radius) {
     deltaX = 0;
     deltaY = 0;
  } else if (distance >= obs.radius) {
     deltaX = -obs.beta * (obs.spread + obs.radius - distance) * std::cos(psi);
     deltaY = -obs.beta * (obs.spread + obs.radius - distance) * std::sin(psi);
  } else {
     deltaX = -copysign(1.0, std::cos(psi)) * 9999;
     deltaY = -copysign(1.0, std::sin(psi)) * 9999;
  }
  force.linear.x += deltaX;
  force.linear.y += deltaY;
  force.linear.z += 0.0;

//  ROS_INFO("request: x=%f, y=%f", (double) obs.x,(double) obs.y);
//  ROS_INFO("sending back response: [dX=%f, dY=%f]", (double) deltaX, (double) deltaY);
}

void robot_Callback(const geometry_msgs::Twist::ConstPtr& msg) {
  ROS_INFO("robot_%d: x=%f, y=%f, z=%f", (int) msg->angular.z, msg->linear.x, msg->linear.y, msg->linear.z);

  force.linear.x = 0.0;
  force.linear.y = 0.0;
  force.linear.z = 0.0;
  force.angular.x = 0.0;
  force.angular.y = 0.0;
  force.angular.z = msg->angular.z;

  ROS_INFO("request from robot_%d: x=%f, y=%f", (int) msg->angular.z, (double) msg->linear.x,(double) msg->linear.y);

  goalForce(msg->linear.x, msg->linear.y, msg->linear.z);

  for (int i = 0; i < obstacle_index; i++) {
    obstacleForce(obstaculos[i], msg->linear.x, msg->linear.y, msg->linear.z);
  }

  int pub = (int) msg->angular.z - 1;
  publishers[pub].publish(force);

  
  ROS_INFO("response to robot_%d: [dX=%f, dY=%f]", (int) force.angular.z, (double) force.linear.x, (double) force.linear.y);

  ros::spinOnce();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "potential_wizard");
  ros::NodeHandle n;

  ros::ServiceServer service_01 = n.advertiseService("set_goal", setGoal);
  ROS_INFO("Goal setting service: online. May the Force be with you.");

  ros::ServiceServer service_02 = n.advertiseService("add_obstacle", addObstacle);
  ROS_INFO("Obstacle adding service: online. May the Force be with you.");

  ros::Subscriber robot_01_listener = n.subscribe("robot_call", 100, robot_Callback);
  ROS_INFO("Control for robot_01: online. May the Force be with you.");

  publishers.push_back(ros::Publisher());
  publishers[0] = n.advertise<geometry_msgs::Twist>("speed_01", 100);
  publishers.push_back(ros::Publisher());
  publishers[1] = n.advertise<geometry_msgs::Twist>("speed_02", 100);
  publishers.push_back(ros::Publisher());
  publishers[2] = n.advertise<geometry_msgs::Twist>("speed_03", 100);

  ros::spin();

  return 0;
}
