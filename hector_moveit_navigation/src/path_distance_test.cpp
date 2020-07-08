#include <ros/ros.h>
#include <geometry_msgs/Point.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "nagvigation_client");

  ros::NodeHandle nh;

  ros::Publisher points_pub = nh.advertise<geometry_msgs::Point>("/compute_path/points", 1);

  ros::Rate loop_rate(10);

  geometry_msgs::Point goal;

  goal.x = 0;
  goal.y = 0;
  goal.z = 1;
  
  while(ros::ok())
  {
    points_pub.publish(goal);
    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}