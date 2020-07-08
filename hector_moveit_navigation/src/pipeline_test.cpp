#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/PointCloud2.h>
#include <gtsp/Tour.h>

#include <hector_moveit_navigation/NavigationAction.h>

#include <pcl_conversions/pcl_conversions.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <vector>
#include <math.h>
#include <string>

std::vector<int> tour;
double moveit_distance = -1;
bool tour_ready = false;
bool length_ready = false;

void tourCallback(const gtsp::Tour::ConstPtr& msg)
{
	ROS_INFO("Got tour");
	tour = msg->tour;
	std::string output = "TOUR: ";
	for(int i = 0; i < tour.size(); i++)
	{
		output.append(std::to_string(tour[i]));
		output.append(" ");
	}
	ROS_INFO("%s", output.c_str());
	tour_ready = true;
}

void lengthCallback(const std_msgs::Float64::ConstPtr& msg)
{
	ROS_INFO("Got length");
	moveit_distance = msg->data;
	ROS_INFO("MoveIt Distance: %f", moveit_distance);
	length_ready = true;
}

int main (int argc, char **argv)
{
	ros::init(argc, argv, "pipeline_test");
	ros::NodeHandle nh;

	ROS_INFO("Begin");
	ros::Publisher point_cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("gtsp_point_cloud", 1);
	ros::Publisher goal_distance_publisher = nh.advertise<geometry_msgs::Point>("/compute_path/point", 1);

	ros::Subscriber tour_sub = nh.subscribe("gtsp_tour_list", 1, tourCallback);
	ros::Subscriber distance_sub = nh.subscribe("/compute_path/length", 1, lengthCallback);

	pcl::PointCloud<pcl::PointXYZ> cloud;
	pcl::PointXYZ point1(0,0,0);
	pcl::PointXYZ point2(0,0,1);
	pcl::PointXYZ point3(3,3,7);
	pcl::PointXYZ point4(3,4,7);
	cloud.push_back(point1);
	cloud.push_back(point2);
	cloud.push_back(point3);
	cloud.push_back(point4);

	pcl::PCLPointCloud2 pcl_pc;
	pcl::toPCLPointCloud2(cloud, pcl_pc);
	sensor_msgs::PointCloud2 output;
	pcl_conversions::fromPCL(pcl_pc, output);

	
	
	ros::Rate poll_rate(100);
	while(point_cloud_publisher.getNumSubscribers() == 0)
		poll_rate.sleep();

	ROS_INFO("Publish point cloud");
	if(ros::ok())
	{	
		point_cloud_publisher.publish(output);
		ros::spinOnce();
		
	}
	while(!tour_ready)
	{
		ros::spinOnce();
		poll_rate.sleep();
	}
	tour_ready = false;
	ROS_INFO("Tour ready");

	int index = -1;

	for(int i = 0; i < tour.size(); i++)
	{

		ROS_INFO("Tour #:%d, Index #:%d, Point: (%f, %f, %f)", i+1, tour[i], cloud.points[tour[i]-1].x, cloud.points[tour[i]-1].y, cloud.points[tour[i]-1].z);
		int x = cloud.points[tour[i]-1].x;
		int y = cloud.points[tour[i]-1].y;
		int z = cloud.points[tour[i]-1].z;
		if(x == 0 && y == 0 && z == 0)
			index = i;
	}

	int next_index = -1;
	if((index + 1) == tour.size())
		next_index = 0;
	else
		next_index = index + 1;

	double actual_distance = 0;
	actual_distance = (cloud.points[tour[index]-1].x - cloud.points[tour[next_index]-1].x)*(cloud.points[tour[index]-1].x - cloud.points[tour[next_index]-1].x) +
					  (cloud.points[tour[index]-1].y - cloud.points[tour[next_index]-1].y)*(cloud.points[tour[index]-1].y - cloud.points[tour[next_index]-1].y) +
					  (cloud.points[tour[index]-1].z - cloud.points[tour[next_index]-1].z)*(cloud.points[tour[index]-1].z - cloud.points[tour[next_index]-1].z);

	actual_distance = sqrt(actual_distance);
	ROS_INFO("Actual point distance: %f", actual_distance);

	geometry_msgs::Point goal_point;

	goal_point.x = cloud.points[tour[next_index]-1].x;
	goal_point.y = cloud.points[tour[next_index]-1].y;
	goal_point.z = cloud.points[tour[next_index]-1].z;

	while(goal_distance_publisher.getNumSubscribers() == 0)
		poll_rate.sleep();

	ROS_INFO("Sending goal point: (%f, %f, %f), waiting for Moveit distance", goal_point.x, goal_point.y, goal_point.z);
	if(ros::ok())
	{	
		goal_distance_publisher.publish(goal_point);
		ros::spinOnce();
		
	}
	while(!length_ready)
	{
		ros::spinOnce();
		poll_rate.sleep();
	}
	length_ready = false;

	ROS_INFO("Length ready");

	double min_dist = actual_distance * 0.75;
	double max_dist = actual_distance * 1.25;

	if(moveit_distance > min_dist && moveit_distance < max_dist)
	{
	/*
		ROS_INFO("Distance within threshold");
		
		actionlib::SimpleActionClient<hector_moveit_navigation::NavigationAction> ac("hector_navigator", true);

		ROS_INFO("Waiting for action server to start.");
		ac.waitForServer();

		ROS_INFO("Action server started, sending goal.");

		hector_moveit_navigation::NavigationGoal goal;

		goal.goal_pose.position.x = goal_point.x;
		goal.goal_pose.position.y = goal_point.y;
		goal.goal_pose.position.z = goal_point.z;

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
	*/	
	}
		

	else
		ROS_INFO("Distance OUTSIDE of threshold");

    /*
	

*/
	return 0;
}