#!/usr/bin/env python
import rospy
import time
import random
from hector_uav_msgs.srv import EnableMotors
from geometry_msgs.msg import PoseStamped
import actionlib
import hector_uav_msgs.msg

def hector_pose_client():
     # First enable motor service
     rospy.wait_for_service("/enable_motors")
     enabler1 = rospy.ServiceProxy("/enable_motors", EnableMotors)
     resp1 = enabler1(True)

     rospy.loginfo("Creating Action Client.")
     client = actionlib.SimpleActionClient('/action/pose', hector_uav_msgs.msg.PoseAction)
     rospy.loginfo("Client created.")

     # This is where the program seems to hang even though I assumed hector would automatically run the action server
     client.wait_for_server()

     # Create a random goal
     g = hector_uav_msgs.msg.PoseGoal()
     g.target_pose.header.frame_id = 'world'
     g.target_pose.pose.position.x = 0
     g.target_pose.pose.position.y = 1
     g.target_pose.pose.position.z = 3

     rospy.loginfo("Sending goal")
     client.send_goal(g)
     rospy.loginfo("Waiting for result")
     client.wait_for_result()
     return(client.get_result())

if __name__ == "__main__":
     try:
         rospy.init_node('drone_explorer')
         result = hector_pose_client()
         rospy.loginfo("Client navigated")
     except rospy.ROSInterruptException:
         print("program interrupted before completion")