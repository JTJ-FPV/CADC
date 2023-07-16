#include <offboard_class/offboard_class.h>

using namespace CADC;
int main(int argc, char **argv)
{
	ros::init(argc, argv, "CADC");
	ros::NodeHandle nh;

    //Constructor
    offboard_class offboard_class_node(&nh);//init some param and then start the controller 
    offboard_class_node.InitControllCallBack();
    ros::spin();
    return 0;
}