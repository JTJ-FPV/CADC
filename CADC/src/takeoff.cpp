#include <offboard_class/offboard_class.h>


using namespace CADC;
int main(int argc, char **argv)
{
	ros::init(argc, argv, "takeoff");
	ros::NodeHandle nh;

    //Constructor
    offboard_class offboard_class_node(&nh);//init some param and then start the controller 



    ros::Rate rate(20.0);
    offboard_class_node.offb_set_mode.request.custom_mode = "OFFBOARD";
    offboard_class_node.arm_cmd.request.value = true;
    offboard_class_node.last_request = ros::Time::now();
    while(ros::ok())
    {
        // ROS_INFO("!!!!!!!!!!!!!!!!!!!!!!! LOOP !!!!!!!!!!!!!!!!!!!!!!!");
        if (offboard_class_node.current_state.mode != "OFFBOARD" && (ros::Time::now() - offboard_class_node.last_request > ros::Duration(7.0))) 
        {
            if (offboard_class_node.set_mode_client.call(offboard_class_node.offb_set_mode) && offboard_class_node.offb_set_mode.response.mode_sent)
                ROS_INFO("Offboard enabled");

            offboard_class_node.last_request = ros::Time::now();
        }
        else 
        {
            if (!offboard_class_node.current_state.armed && (ros::Time::now() - offboard_class_node.last_request > ros::Duration(7.0))) {
                if (offboard_class_node.arming_client.call(offboard_class_node.arm_cmd) && offboard_class_node.arm_cmd.response.success)
                    ROS_INFO("Vehicle armed");

                offboard_class_node.last_request = ros::Time::now();
            }
            else if(offboard_class_node.current_state.armed)
            {
                // ROS_INFO("!!!!!!!!!!!!!!!!!!!!!!! TAKE OFF !!!!!!!!!!!!!!!!!!!!!!!");
            }
        }
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}