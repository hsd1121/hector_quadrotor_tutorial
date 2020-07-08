#!/usr/bin/env python
import rospy
import actionlib
from geometry_msgs.msg import Pose

import hector_moveit_navigation.msg
import octomap_msgs.msg

def callback(data):
	rospy.loginfo("Got octomap marker array")

def navigator_client():
	rospy.init_node('navigator_client_py')
	rospy.loginfo("Initialized node")
	rospy.Subscriber("octomap_binary", octomap_msgs.msg.Octomap, callback)

	client = actionlib.SimpleActionClient('hector_navigator', hector_moveit_navigation.msg.NavigationAction)

	rospy.loginfo("Waiting for server")
	client.wait_for_server()

	goal = hector_moveit_navigation.msg.NavigationGoal();
	goal.goal_pose.position.x = 0;
	goal.goal_pose.position.y = 0;
	goal.goal_pose.position.z = 1;

	goal.goal_pose.orientation.x = 0;
	goal.goal_pose.orientation.y = 0;
	goal.goal_pose.orientation.z = 0;
	goal.goal_pose.orientation.w = 1;

	rate = rospy.Rate(2)
	while not rospy.is_shutdown():
		if goal.goal_pose.position.z == 5:
			goal.goal_pose.position.z = 1
		else:
			goal.goal_pose.position.z = goal.goal_pose.position.z + 1

		rospy.loginfo("Sending goal")
		client.send_goal(goal)

		rospy.loginfo("Waiting for result")
		client.wait_for_result()
		rate.sleep()
	

	

if __name__ == '__main__':
	try:
		navigator_client()
	except rospy.ROSInterruptException:
		pass