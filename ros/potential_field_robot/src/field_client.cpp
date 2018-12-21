#include "ros/ros.h"
#include "potential_field_robot/PotentialField.h"
#include "potential_field_robot/Force.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "field_client");
  if (argc != 3)
  {
    ROS_INFO("usage: goal_client X Y");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<potential_field_robot::Force>("force");
  potential_field_robot::Force srv;
  srv.request.x = atoll(argv[1]);
  srv.request.y = atoll(argv[2]);
  if (client.call(srv))
  {
    ROS_INFO("Força: x=%f, y=%f, psi=%f", (double) srv.response.dX, (double) srv.response.dY, (double) srv.response.psi);
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
    return 1;
  }

  return 0;
}
