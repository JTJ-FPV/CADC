#include <offboard_class/offboard_class.h>

const int MINIMUM_POINTS = 20; const double EPSILON = 1 * 1;

using namespace CADC;
int main(int argc, char **argv)
{
	ros::init(argc, argv, "CADC");
	ros::NodeHandle nh;
    DBSCAN::DBSCAN ds(MINIMUM_POINTS, EPSILON);
    //Constructor
    offboard_class offboard_class_node(&nh, ds);//init some param and then start the controller 
    offboard_class_node.InitControllCallBack();
    ros::spin();
    return 0;
}