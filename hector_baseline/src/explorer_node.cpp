#include <baseline.h>


int main(int argc, char** argv)
{

    ros::init(argc, argv, "hector_baseline");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle node_handle("~");
    
    Quadrotor quad(std::ref(node_handle));
    quad.takeoff();
    quad.run();
    return 0;
}