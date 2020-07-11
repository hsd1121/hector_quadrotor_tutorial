#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <math.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <octomap_msgs/conversions.h>

#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/GetPlanningScene.h>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <hector_uav_msgs/EnableMotors.h>
#include <hector_moveit_actions/ExecuteDroneTrajectoryAction.h>
#include <hector_moveit_navigation/NavigationAction.h>

#include <octomap/OcTree.h>

#define _USE_MATH_DEFINES
#include <cmath>
#include <queue>
#define XMIN -25
#define XMAX 25
#define YMIN -25
#define YMAX 35
#define ZMIN 0.2
#define ZMAX 10

#define EPSILON 1e-4

typedef std::pair<double,geometry_msgs::Pose> DistancedPoint;

class Compare{
    public: 
        bool operator()(DistancedPoint& lhs, DistancedPoint & rhs)
        {
            return lhs.first < rhs.first;
        }
};
typedef std::priority_queue<DistancedPoint,std::vector<DistancedPoint>, Compare> DistancedPointPriorityQueue;

#include <iostream>
#include <chrono>

using namespace std;
using  ns = chrono::nanoseconds;
using get_time = chrono::steady_clock;

#include <omp.h>
#define PATCH_LIMIT 1
class Quadrotor{
    private:
        ros::NodeHandle nh_;
        std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
        actionlib::SimpleActionClient<hector_moveit_actions::ExecuteDroneTrajectoryAction> trajectory_client;
        actionlib::SimpleActionServer<hector_moveit_navigation::NavigationAction> as_;
        std::unique_ptr<robot_state::RobotState> start_state;
        std::unique_ptr<planning_scene::PlanningScene> planning_scene;
        std::string action_name_;

        hector_moveit_navigation::NavigationFeedback feedback_;
        hector_moveit_navigation::NavigationResult result_;

        const double takeoff_altitude = 1.0;
        int GRID;
        bool odom_received,trajectory_received;
        bool isPathValid;
        bool collision;

        geometry_msgs::Pose odometry_information;
        std::vector<geometry_msgs::Pose> trajectory;
        
        std::vector<geometry_msgs::Pose> frontiers;
        std::vector<geometry_msgs::Pose> explored;

        ros::Subscriber base_sub,plan_sub,goal_sub,distance_sub;
        ros::Publisher distance_pub;
        ros::Publisher frontier_pub;

        ros::ServiceClient motor_enable_service; 
        ros::ServiceClient planning_scene_service;

        moveit_msgs::RobotState plan_start_state;
        moveit_msgs::RobotTrajectory plan_trajectory;
    
        const std::string PLANNING_GROUP = "DroneBody";

        void poseCallback(const nav_msgs::Odometry::ConstPtr & msg);

        void planCallback(const moveit_msgs::DisplayTrajectory::ConstPtr& msg);

        void collisionCallback(const hector_moveit_actions::ExecuteDroneTrajectoryFeedbackConstPtr& feedback);

        void executeCB(const hector_moveit_navigation::NavigationGoalConstPtr &goal);

        void computePathLengthCB(const geometry_msgs::Point::ConstPtr &path);

        void findFrontier();
    
    public:
        Quadrotor(ros::NodeHandle& nh, std::string name);
        void enableMotors();
        void run();
        
};