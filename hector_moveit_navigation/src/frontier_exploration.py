#!/usr/bin/env python
import rospy
import actionlib
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray

import hector_moveit_navigation.msg
import octomap_msgs.msg

frontiers = PoseArray()

# Get frontiers
def callback(data):
	global frontiers
	frontiers = data

def navigator_client():
	rospy.init_node('navigator_client_py')
	rospy.loginfo("Initialized node")
	rospy.Subscriber("frontiers", PoseArray, callback)

	client = actionlib.SimpleActionClient('hector_navigator', hector_moveit_navigation.msg.NavigationAction)

	rospy.loginfo("Waiting for server")
	client.wait_for_server()

	goal = hector_moveit_navigation.msg.NavigationGoal()
	rate = rospy.Rate(2)
	while not rospy.is_shutdown():
		global frontiers
		current_frontiers = frontiers

		for pose in current_frontiers.poses:
			goal.goal_pose = pose
			rospy.loginfo("Sending goal position (%f, %f, %f)", pose.position.x, pose.position.y, pose.position.z)
			client.send_goal(goal)

			rospy.loginfo("Waiting for result")
			client.wait_for_result()

		rate.sleep()
	

	

if __name__ == '__main__':
	try:
		navigator_client()
	except rospy.ROSInterruptException:
		pass