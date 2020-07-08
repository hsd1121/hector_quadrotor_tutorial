#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <hector_moveit_navigation/NavigationAction.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "nagvigation_client");

  actionlib::SimpleActionClient<hector_moveit_navigation::NavigationAction> ac("hector_navigator", true);

  ROS_INFO("Waiting for action server to start.");
  ac.waitForServer();

  ROS_INFO("Action server started, sending goal.");

  hector_moveit_navigation::NavigationGoal goal;

  goal.goal_pose.position.x = 0;
  goal.goal_pose.position.y = 0;
  goal.goal_pose.position.z = 1;

  goal.goal_pose.orientation.x = 0;
  goal.goal_pose.orientation.y = 0;
  goal.goal_pose.orientation.z = 0;
  goal.goal_pose.orientation.w = 1;

  ROS_INFO("Send takeoff goal");
  ac.sendGoal(goal);

  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  if(finished_before_timeout)
  {
  	actionlib::SimpleClientGoalState state = ac.getState();
  	ROS_INFO("Takeoff finished: %s", state.toString().c_str());
  }
  else
  	ROS_INFO("Takeoff did not finish before the time out.");

  goal.goal_pose.position.x = 2;
  goal.goal_pose.position.y = 2;
  goal.goal_pose.position.z = 2;

  ac.sendGoal(goal);

  finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  if(finished_before_timeout)
  {
  	actionlib::SimpleClientGoalState state = ac.getState();
  	ROS_INFO("Action finished: %s", state.toString().c_str());
  }
  else
  	ROS_INFO("Action did not finish before the time out.");

  return 0;
}