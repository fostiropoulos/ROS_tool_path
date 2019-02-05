#include "ros/ros.h"

bool compute(tool_path_planning::PathPlanningTask::Request  &req,
         tool_path_planning::PathPlanningTask::Response &res)
{
  res.output = req.filename;
  ROS_INFO("request: x=%ld, y=%ld", req.filename);
  ROS_INFO("sending back response: [%ld]", res.output);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "generate_path");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("generate_path", add);
  ROS_INFO("Read to calculate path.");
  ros::spin();

  return 0;
}
