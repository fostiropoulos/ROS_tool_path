#include "ros/ros.h"
#include "tool_path_planning/PathPlanningTask.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "generate_path_client");
  if (argc != 2)
  {
    ROS_INFO("usage: rosrun tool_path_planning generate_path_client /path/to/stl_file");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<tool_path_planning::PathPlanningTask>("generate_path");
  tool_path_planning::PathPlanningTask srv;
  srv.request.filename = (argv[1]);
  if (client.call(srv))
  {
    ROS_INFO_STREAM("Tool " << srv.response.output);
  }
  else
  {
    ROS_ERROR("Failed to call service tool_path_planning");
    return 1;
  }

  return 0;
}
