#include "ros/ros.h"
#include <cmath>
#include "potential_field_robot/PotentialField.h"
#include "potential_field_robot/Force.h"

double X = 0.0;
double Y = 0.0;
double alpha = 1.0;
double beta = 1.0;
double radius = 1.0;
double spread = 2.0;

bool goal(potential_field_robot::PotentialField::Request  &req,
          potential_field_robot::PotentialField::Response &res) {
  X = req.x;
  Y = req.y;
  alpha = req.a;
  beta = req.b;
  radius = req.r;
  spread = req.s;
  ROS_INFO("request: x=%f, y=%f, alpha=%f, beta=%f, radius=%f, spread=%f", (double)req.x,(double) req.y, (double)req.a, (double) req.b, (double) req.r, (double) req.s);
}

bool force(potential_field_robot::Force::Request  &req,
           potential_field_robot::Force::Response &res)
{
  double distance = std::pow(std::pow(X-req.x, 2)+std::pow(Y-req.y, 2), 0.5);
  double psi = std::atan2(req.y-Y, req.x-X);
  double deltaX, deltaY;
  if (distance < radius) {
     deltaX = 0;
     deltaY = 0;
  } else if (distance <= spread + radius) {
     deltaX = alpha * (distance - radius) * std::cos(psi);
     deltaY = alpha * (distance - radius) * std::sin(psi);
  } else {
     deltaX = alpha * spread * std::cos(psi);
     deltaY = alpha * spread * std::sin(psi);
  }
  res.dX = deltaX;
  res.dY = deltaY;
  res.psi = std::atan2(deltaY, deltaX);
//  res.sum = req.a + req.b;
  ROS_INFO("request: x=%f, y=%f", (double)req.x,(double) req.y);
  ROS_INFO("sending back response: [dX=%f, dY=%f, psi=%f]", (double)res.dX, (double) res.dY, (double) res.psi);
  return true;
}

bool obstacle(potential_field_robot::Force::Request  &req,
           potential_field_robot::Force::Response &res)
{
  double distance = std::pow(std::pow(X-req.x, 2)+std::pow(Y-req.y, 2), 0.5);
  double psi = std::atan2(req.y-Y, req.x-X);
  double deltaX, deltaY;
  if (distance > spread + radius) {
     deltaX = 0;
     deltaY = 0;
  } else if (distance >= radius) {
     deltaX = -beta * (spread + radius - distance) * std::cos(psi);
     deltaY = -beta * (spread + radius - distance) * std::sin(psi);
  } else {
     deltaX = -copysign(1.0, std::cos(psi)) * INFINITY;
     deltaY = -copysign(1.0, std::sin(psi)) * INFINITY;
  }
  res.dX = deltaX;
  res.dY = deltaY;
  res.psi = std::atan2(deltaY, deltaX);
//  res.sum = req.a + req.b;
  ROS_INFO("request: x=%f, y=%f", (double)req.x,(double) req.y);
  ROS_INFO("sending back response: [dX=%f, dY=%f, psi=%f]", (double)res.dX, (double) res.dY, (double) res.psi);
  return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "field_server");
  ros::NodeHandle n;

  ros::ServiceServer service_1 = n.advertiseService("goal", goal);
  ros::ServiceServer service_2 = n.advertiseService("force", force);
  ros::ServiceServer service_3 = n.advertiseService("obstacle", obstacle);
  ROS_INFO("Serviços online. Use a Força.");
  ros::spin();

  return 0;
}
