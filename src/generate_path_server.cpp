#include "ros/ros.h"
#include "tool_path_planning/PathPlanningTask.h"
bool compute(tool_path_planning::PathPlanningTask::Request  &req,
         tool_path_planning::PathPlanningTask::Response &res)
{
  res.output = req.filename;
  ROS_INFO_STREAM("request: " <<  req.filename);
  ROS_INFO_STREAM("sending back response: " << res.output);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "generate_path");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("generate_path", compute);
  ROS_INFO("Read to calculate path.");
  ros::spin();

  return 0;
}
