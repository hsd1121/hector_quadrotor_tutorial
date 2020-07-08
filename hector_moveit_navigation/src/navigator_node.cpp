#include <Navigator.h>


int main(int argc, char** argv)
{

    ros::init(argc, argv, "hector_navigator");    
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle node_handle("~");
    
    Quadrotor quad(std::ref(node_handle), "hector_navigator");
    quad.enableMotors();
    quad.run();
    return 0;
}