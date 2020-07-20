#include <Navigator.h>

Quadrotor::Quadrotor(ros::NodeHandle& nh, std::string name) : 
    as_(nh_, name, boost::bind(&Quadrotor::executeCB, this, _1), false),
    action_name_(name),
    trajectory_client("/action/trajectory",true)
{
    trajectory_client.waitForServer();
    odom_received = false;
    trajectory_received = false;
    collision = false;

    as_.start();

    base_sub = nh.subscribe<nav_msgs::Odometry>("/ground_truth/state",10,&Quadrotor::poseCallback,this);
    plan_sub = nh.subscribe<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path",1,&Quadrotor::planCallback,this);
    distance_sub = nh.subscribe<geometry_msgs::Point>("/compute_path/point",1,&Quadrotor::computePathLengthCB,this);
    
    distance_pub = nh.advertise<std_msgs::Float64>("/compute_path/length",1);
    frontier_pub = nh.advertise<geometry_msgs::PoseArray>("/frontiers", 1);

    move_group.reset(new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP));
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kmodel = robot_model_loader.getModel();
    
    motor_enable_service = nh.serviceClient<hector_uav_msgs::EnableMotors>("/enable_motors");
    planning_scene_service = nh.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");

    move_group->setPlannerId("RRTConnectkConfigDefault");
    move_group->setNumPlanningAttempts(10);
    move_group->setWorkspace(XMIN,YMIN,ZMIN,XMAX,YMAX,ZMAX);
    
    start_state.reset(new robot_state::RobotState(move_group->getRobotModel()));
    planning_scene.reset(new planning_scene::PlanningScene(kmodel));
    
}

void Quadrotor::executeCB(const hector_moveit_navigation::NavigationGoalConstPtr &goal)
{
    feedback_.feedback_pose = odometry_information;

    std::vector<double> target(7);
    target[0] = goal->goal_pose.position.x;
    target[1] = goal->goal_pose.position.y;
    target[2] = goal->goal_pose.position.z;
    target[3] = goal->goal_pose.orientation.x;
    target[4] = goal->goal_pose.orientation.y;
    target[5] = goal->goal_pose.orientation.z;
    target[6] = goal->goal_pose.orientation.w;
    
    std::vector<double> start_state_(7);
    start_state_[0] = odometry_information.position.x;
    start_state_[1] = odometry_information.position.y;
    start_state_[2] = odometry_information.position.z;
    start_state_[3] = odometry_information.orientation.x;
    start_state_[4] = odometry_information.orientation.y;
    start_state_[5] = odometry_information.orientation.z;
    start_state_[6] = odometry_information.orientation.w;
    
    this->move_group->setJointValueTarget(target);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    
    this->collision = false;
    ROS_INFO("Try to start from [%lf,%lf,%lf] with orienation [%lf, %lf, %lf, %lf]",odometry_information.position.x,odometry_information.position.y,odometry_information.position.z,odometry_information.orientation.x,odometry_information.orientation.y,odometry_information.orientation.z,odometry_information.orientation.w);
    ROS_INFO("Try to go to [%lf,%lf,%lf] with orienation [%lf, %lf, %lf, %lf]",goal->goal_pose.position.x,goal->goal_pose.position.y,goal->goal_pose.position.z,goal->goal_pose.orientation.x,goal->goal_pose.orientation.y,goal->goal_pose.orientation.z,goal->goal_pose.orientation.w);
    this->start_state->setVariablePositions(start_state_);
    this->move_group->setStartState(*start_state);

    this->isPathValid = (move_group->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if(this->isPathValid){
        
        this->plan_start_state = plan.start_state_;
        this->plan_trajectory = plan.trajectory_;
        while(!trajectory_received){
            ROS_INFO("Waiting for trajectory");
            ros::Duration(0.2).sleep();
        }
        hector_moveit_actions::ExecuteDroneTrajectoryGoal goal;
        
        for(int i=0;i<trajectory.size();i++){
            if(i==0){
                double y_diff = trajectory[i].position.y - odometry_information.position.y;
                double x_diff = trajectory[i].position.x - odometry_information.position.x;
                double yaw = atan2(y_diff,x_diff);
                if(fabs(y_diff)>EPSILON || fabs(x_diff)>EPSILON ){ //Prevent 0 division
                    tf::Quaternion q = tf::createQuaternionFromYaw(yaw+M_PI);
                    trajectory[i].orientation.x = q.x();
                    trajectory[i].orientation.y = q.y();
                    trajectory[i].orientation.z = q.z();
                    trajectory[i].orientation.w = q.w();
                }
            }
            else if(i+1<trajectory.size()){
                geometry_msgs::Pose next_waypoint = trajectory[i+1];
                double y_diff = next_waypoint.position.y - trajectory[i].position.y;
                double x_diff = next_waypoint.position.x - trajectory[i].position.x;
                double yaw = atan2(y_diff,x_diff);
                
                if(fabs(y_diff)>EPSILON || fabs(x_diff)>EPSILON ){ //Prevent 0 division
                    
                    tf::Quaternion q = tf::createQuaternionFromYaw(yaw+M_PI);
                    trajectory[i+1].orientation.x = q.x();
                    trajectory[i+1].orientation.y = q.y();
                    trajectory[i+1].orientation.z = q.z();
                    trajectory[i+1].orientation.w = q.w();
                }
            }
            goal.trajectory.push_back(trajectory[i]);
        }    
        ROS_INFO("Send Trajectory Goal");
        trajectory_client.sendGoal(goal,actionlib::SimpleActionClient<hector_moveit_actions::ExecuteDroneTrajectoryAction>::SimpleDoneCallback(),
                            actionlib::SimpleActionClient<hector_moveit_actions::ExecuteDroneTrajectoryAction>::SimpleActiveCallback(),
                            boost::bind(&Quadrotor::collisionCallback,this,_1));
        bool finished = false;
        while(ros::ok()) {
            feedback_.feedback_pose = odometry_information;
            finished = trajectory_client.waitForResult(ros::Duration(0.1));
            if(finished)
                break;
            if(as_.isPreemptRequested() || !ros::ok())
            {
                as_.setPreempted();
                trajectory_client.cancelGoal();
                ROS_INFO("Trajectory cancelled");
                return;
            }
            
        }
        ROS_INFO("Trajectory is traversed");
        this->trajectory_received = false;
        this->odom_received = false;
    }
    result_.result_pose = feedback_.feedback_pose;
    as_.setSucceeded(result_);

    //return this->isPathValid;
}

void Quadrotor::poseCallback(const nav_msgs::Odometry::ConstPtr & msg)
{
    odometry_information = msg->pose.pose;
    odom_received = true;
}

void Quadrotor::planCallback(const moveit_msgs::DisplayTrajectory::ConstPtr& msg)
{
    if(!odom_received) return;
    trajectory.clear();
    for(auto robot_traj: msg->trajectory){
        for(auto point : robot_traj.multi_dof_joint_trajectory.points){
            geometry_msgs::Pose waypoint;
            waypoint.position.x = point.transforms[0].translation.x;
            waypoint.position.y = point.transforms[0].translation.y;
            waypoint.position.z = point.transforms[0].translation.z;

            waypoint.orientation.x = point.transforms[0].rotation.x;
            waypoint.orientation.y = point.transforms[0].rotation.y;
            waypoint.orientation.z = point.transforms[0].rotation.z;
            waypoint.orientation.w = point.transforms[0].rotation.w;

            trajectory.push_back(waypoint);
        }
    }
    trajectory_received = true;
}

void Quadrotor::collisionCallback(const hector_moveit_actions::ExecuteDroneTrajectoryFeedbackConstPtr& feedback)
{
    moveit_msgs::GetPlanningScene srv;
    srv.request.components.components = moveit_msgs::PlanningSceneComponents::OCTOMAP;
    if(planning_scene_service.call(srv)){
        this->planning_scene->setPlanningSceneDiffMsg(srv.response.scene);
        octomap_msgs::Octomap octomap = srv.response.scene.world.octomap.octomap;
        octomap::OcTree* current_map = (octomap::OcTree*)octomap_msgs::msgToMap(octomap);
        
        double resolution = current_map->getResolution();
        int unknown = 0, known = 0;
        for(double ix=XMIN;ix<XMAX;ix+=resolution){
            for(double iy=YMIN;iy<YMAX;iy+=resolution){
                for(double iz=ZMIN;iz<ZMAX;iz+=resolution){
                    if(!current_map->search(ix,iy,iz))
                        unknown++;
                    else
                        known++;
                }
            }
        }
        double rate = known*100.0/(float)(unknown+known);
        std_msgs::Float64 msg;
        msg.data = rate;
        //rate_ack.publish(msg);
        //ROS_INFO("Coverage of Orchard Volume: %lf Percent",rate);
        
        delete current_map;
        std::vector<size_t> invalid_indices;
        this->isPathValid = this->planning_scene->isPathValid(plan_start_state,plan_trajectory,PLANNING_GROUP,true,&invalid_indices);
        ros::spinOnce();
        bool too_close = false;
        for(int i=0;i<invalid_indices.size();i++){
            for(int j=0;j<plan_trajectory.multi_dof_joint_trajectory.points[invalid_indices[i]].transforms.size();j++){
                
                double x = plan_trajectory.multi_dof_joint_trajectory.points[invalid_indices[i]].transforms[j].translation.x;
                double y = plan_trajectory.multi_dof_joint_trajectory.points[invalid_indices[i]].transforms[j].translation.y;
                double z = plan_trajectory.multi_dof_joint_trajectory.points[invalid_indices[i]].transforms[j].translation.z;

                double dist = sqrt(pow(x-odometry_information.position.x,2) + pow(y-odometry_information.position.y,2) + pow(z-odometry_information.position.z,2));
                if(dist < 0.5) too_close = true;
            }
        }
        
       if(!isPathValid && !too_close){
            //TODO: this->move_group->stop(); When migrating to complete MoveIt! ExecuteService, this will work as expected.
            this->trajectory_client.cancelGoal();
            this->collision = true;
            ROS_INFO("Trajectory is now in collision with the world");
        }
    }
    else
        ROS_INFO("Couldn't fetch the planning scene");
}

void Quadrotor::computePathLengthCB(const geometry_msgs::Point::ConstPtr &point)
{

    ROS_INFO("Computing MoveIt distance from (%f,%f,%f) to (%f,%f,%f)", odometry_information.position.x, odometry_information.position.y, odometry_information.position.z, 
                                                                        point->x, point->y, point->z);
    std::vector<double> target(7);
    target[0] = point->x;
    target[1] = point->y;
    target[2] = point->z;
    target[3] = odometry_information.orientation.x;
    target[4] = odometry_information.orientation.y;
    target[5] = odometry_information.orientation.z;
    target[6] = odometry_information.orientation.w;
    
    std::vector<double> start_state_(7);
    /*
    start_state_[0] = path->start.x;
    start_state_[1] = path->start.y;
    start_state_[2] = path->start.z;
    start_state_[3] = 0;
    start_state_[4] = 0;
    start_state_[5] = 0;
    start_state_[6] = 1;
    */
    start_state_[0] = odometry_information.position.x;
    start_state_[1] = odometry_information.position.y;
    start_state_[2] = odometry_information.position.z;
    start_state_[3] = odometry_information.orientation.x;
    start_state_[4] = odometry_information.orientation.y;
    start_state_[5] = odometry_information.orientation.z;
    start_state_[6] = odometry_information.orientation.w;

    std_msgs::Float64 distance;
    distance.data = 0.0;

    this->move_group->setJointValueTarget(target);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    
    this->collision = false;
    this->start_state->setVariablePositions(start_state_);
    this->move_group->setStartState(*start_state);

    this->isPathValid = (move_group->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if(this->isPathValid){
        ROS_INFO("Path is valid");
        this->plan_start_state = plan.start_state_;
        this->plan_trajectory = plan.trajectory_;
        while(!trajectory_received){
            ROS_INFO("Waiting for trajectory");
            //ros::Duration(0.2).sleep();
        }
            
        for(int i=1;i<trajectory.size();i++){
            ROS_INFO("Trajectory #:%d, Point: (%f, %f, %f)", i+1, trajectory[i].position.x, trajectory[i].position.y, trajectory[i].position.z);
            double temp_distance = (trajectory[i].position.x - trajectory[i-1].position.x)*(trajectory[i].position.x - trajectory[i-1].position.x) +
                                   (trajectory[i].position.y - trajectory[i-1].position.y)*(trajectory[i].position.y - trajectory[i-1].position.y) +
                                   (trajectory[i].position.z - trajectory[i-1].position.z)*(trajectory[i].position.z - trajectory[i-1].position.z);
            temp_distance = sqrt(temp_distance);

            distance.data += temp_distance;    
            ROS_INFO("Step Distance: %f", temp_distance);
            ROS_INFO("Running Distance: %f", distance.data);                   
        }    
        distance_pub.publish(distance);
        ROS_INFO("MoveIt distance is %f", distance.data);
    }
}

void Quadrotor::findFrontier()
{
    moveit_msgs::GetPlanningScene srv;
    srv.request.components.components = moveit_msgs::PlanningSceneComponents::OCTOMAP;
    ros::spinOnce();

    if(planning_scene_service.call(srv)){
        this->planning_scene->setPlanningSceneDiffMsg(srv.response.scene);
        octomap_msgs::Octomap octomap = srv.response.scene.world.octomap.octomap;
        octomap::OcTree* current_map = (octomap::OcTree*)octomap_msgs::msgToMap(octomap);
        
        geometry_msgs::PoseArray posearray;
        double resolution = current_map->getResolution();

        std::vector<std::pair<double, geometry_msgs::Pose> > candidate_frontiers;
        for(octomap::OcTree::leaf_iterator n = current_map->begin_leafs(current_map->getTreeDepth()); n != current_map->end_leafs(); ++n)
        {
          //  ROS_INFO("Node size: %f", n.getSize());

            double x_cur = n.getX();
            double y_cur = n.getY();
            double z_cur = n.getZ();
            if(x_cur > XMIN && x_cur < XMAX && 
               y_cur > YMIN && y_cur < YMAX && 
               z_cur > ZMIN && z_cur < ZMAX)
            {
                
                if(!current_map->isNodeOccupied(*n))
                {
                    octomap::OcTreeKey key = n.getKey();
                    octomap::OcTreeKey neighborkey;
                    for(int i = 0; i < 4; i++)
                    {
                        neighborkey = key;
                        switch(i)
                        {
                            case 0:
                                neighborkey[0] += 1;
                                break;
                            case 1:
                                neighborkey[0] -= 1;
                                break;
                            case 2:
                                neighborkey[1] += 1;
                                break;
                            case 3:
                                neighborkey[1] -= 1;
                                break;
                            default:
                                ROS_INFO("Error: Got to default in switch neighbor check");
                                break;
                        }
                        octomap::OcTreeNode* result = current_map->search(neighborkey);
                        if(result == NULL)
                        {
                            geometry_msgs::Pose p;
                            p.position.x = x_cur;
                            p.position.y = y_cur;
                            p.position.z = z_cur;
                            p.orientation.x = odometry_information.orientation.x;
                            p.orientation.y = odometry_information.orientation.y;
                            p.orientation.z = odometry_information.orientation.z;
                            p.orientation.w = odometry_information.orientation.w;

                            double dist = sqrt(pow(p.position.x - odometry_information.position.x,2) + pow(p.position.y - odometry_information.position.y,2) + pow(p.position.z - odometry_information.position.z,2));
                            if(dist > 2)
                                candidate_frontiers.push_back({dist,p});

                            continue;
                        }
                    }
                    
                    
                }
                
            }
        }

        std::vector<int> indices(candidate_frontiers.size());
        for(int i=0;i<indices.size();i++)
            posearray.poses.push_back(candidate_frontiers[i].second);
        if(indices.size() > 0)
        {
            posearray.header.stamp = ros::Time::now();
            frontier_pub.publish(posearray);
        }
    }


}

void Quadrotor::enableMotors()
{
    hector_uav_msgs::EnableMotors srv;
    srv.request.enable = true;
    motor_enable_service.call(srv);
    while(!odom_received)
        ;
    /*
    geometry_msgs::Pose takeoff_pose = odometry_information;
    takeoff_pose.position.z = takeoff_altitude;
    go(takeoff_pose);
    ROS_INFO("Takeoff successful");
    */
}

void Quadrotor::run()
{
    ros::Rate rate(2);
    int count = 1;
    while(ros::ok()){
        while(!odom_received)
            rate.sleep();
        this->findFrontier();

        ros::spinOnce();
        rate.sleep();
    }
}