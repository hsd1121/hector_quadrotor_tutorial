#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <hector_uav_msgs/PoseAction.h>
#include <hector_uav_msgs/EnableMotors.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "quadrotor_navigation");

  ros::NodeHandle n;

  ROS_INFO("Enabling motors for hector_quadrotor");
  ros::ServiceClient client = n.serviceClient<hector_uav_msgs::EnableMotors>("enable_motors");

  hector_uav_msgs::EnableMotors srv;
  srv.request.enable = true;

  if(client.call(srv))
  {
    ROS_INFO("Successfully enabled motors?: %s", srv.response.success ? "true" : "false");
  }
  else
  {
    ROS_INFO("Failed to call service enable_motors");
    return 1;
  }

  actionlib::SimpleActionClient<hector_uav_msgs::PoseAction> ac("action/pose", true);

  ROS_INFO("Waiting for action server to start.");
  ac.waitForServer();

  ROS_INFO("Action server started, sending goal.");

  
  hector_uav_msgs::PoseGoal goal;
  goal.target_pose.header.frame_id = "world";
  goal.target_pose.pose.position.x = 0;
  goal.target_pose.pose.position.y = 0;
  goal.target_pose.pose.position.z = 1;

  goal.target_pose.pose.orientation.x = 0;
  goal.target_pose.pose.orientation.y = 0;
  goal.target_pose.pose.orientation.z = 0;
  goal.target_pose.pose.orientation.w = 1;

  ros::Time stamp = ros::Time::now();
  goal.target_pose.header.stamp.sec = stamp.sec;
  goal.target_pose.header.stamp.nsec = stamp.nsec;


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

  goal.target_pose.pose.position.x = 2;
  goal.target_pose.pose.position.y = 2;
  goal.target_pose.pose.position.z = 2;
  stamp = ros::Time::now();
  goal.target_pose.header.stamp.sec = stamp.sec;
  goal.target_pose.header.stamp.nsec = stamp.nsec;

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