#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_msgs/TFMessage.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <mavros_msgs/HomePosition.h>
Eigen::Vector3d quat_to_euler(const geometry_msgs::Quaternion &q);
geometry_msgs::PoseStamped local_pos;
void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    local_pos = *msg;
    // ROS_INFO("%lf,%lf,%lf,%lf", local_pos.pose.orientation.w, local_pos.pose.orientation.x, local_pos.pose.orientation.y, local_pos.pose.orientation.z);
	// ROS_INFO("local_pos_cb");
}
geometry_msgs::Quaternion q;
void tf_cb(const tf2_msgs::TFMessage::ConstPtr &tf2_msg_)
{
    q.w = tf2_msg_->transforms[0].transform.rotation.w;
    q.x = tf2_msg_->transforms[0].transform.rotation.x;
    q.y = tf2_msg_->transforms[0].transform.rotation.y;
    q.z = tf2_msg_->transforms[0].transform.rotation.z;
    // ROS_INFO("%lf, %lf, %lf, %lf", tf2_msg_->transforms[0].transform.rotation.w, tf2_msg_->transforms[0].transform.rotation.x,
    //                                    tf2_msg_->transforms[0].transform.rotation.y, tf2_msg_->transforms[0].transform.rotation.z);
    // double sum = pow(q.w, 2) + pow(q.x, 2) + pow(q.y, 2) + pow(q.z, 2);
    // ROS_INFO_STREAM("This Sum is : " << sum);
    // ROS_INFO("TF2");
}
Eigen::Vector3d eular;
void odomCallback(const nav_msgs::OdometryConstPtr &odom)
{
    tf::Quaternion quat;
    tf::quaternionMsgToTF(odom->pose.pose.orientation, quat);

    double roll, pitch, yaw;//定义存储r\p\y的容器
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
    eular << roll, pitch, yaw;
    ROS_INFO("*************************************");
    ROS_INFO_STREAM("ROLL : " << roll * 180 / M_PI);
    ROS_INFO_STREAM("PITCH : " << pitch * 180 / M_PI);
    ROS_INFO_STREAM("YAW : " << yaw * 180 / M_PI);
    ROS_INFO("*************************************");
}
mavros_msgs::HomePosition hp;
Eigen::Vector3d hpeular;
void hp_cb(const mavros_msgs::HomePositionConstPtr &msg)
{
    hp = *msg;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(hp.orientation, quat);
    double roll, pitch, yaw;//定义存储r\p\y的容器
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
    hpeular << roll, pitch, yaw;
    ROS_INFO("*************************************");
    ROS_INFO_STREAM("Hp ROLL : " << roll * 180 / M_PI);
    ROS_INFO_STREAM("Hp PITCH : " << pitch * 180 / M_PI);
    ROS_INFO_STREAM("Hp YAW : " << yaw * 180 / M_PI);
    ROS_INFO("*************************************");
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "attitude");
    ros::NodeHandle nh;
    // ros::Subscriber local_pos_sub;
    // ros::Subscriber vision_pos_sub;
    // ros::Subscriber tf_sub;
    // gps定位信息
    // local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
        // ("mavros/local_position/pose", 10, &offboard_class::local_pos_cb,this);
	// 订阅视觉定位信息
    // vision_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
	// 	("mavros/vision_pose/pose" , 10, local_pos_cb);
    // tf_sub = nh.subscribe<tf2_msgs::TFMessage>
    //     ("/tf", 10, tf_cb);
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>
        ("/mavros/local_position/odom", 1, odomCallback);
    // Eigen::Vector3d eular;
    ros::Rate rate(20.0);
    ros::Subscriber hp = nh.subscribe<mavros_msgs::HomePosition>
        ("/mavros/home_position/home", 1, hp_cb);
    while(ros::ok())
    {
        // eular = quat_to_euler(q);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
Eigen::Vector3d quat_to_euler(const geometry_msgs::Quaternion &q)
{
	Eigen::Quaterniond q_eigen(q.w, q.x, q.y, q.z);
	// X-Y-Z轴旋转次序
	Eigen::Vector3d eular = q_eigen.matrix().eulerAngles(0,1,2);
    ROS_INFO("*************************************");
	ROS_INFO("X AIXED : %lf", eular(0) * 180 / M_PI);
	ROS_INFO("Y AIXED : %lf", eular(1) * 180 / M_PI);
	ROS_INFO("Z AIXED : %lf", eular(2) * 180 / M_PI);
    ROS_INFO("*************************************");
	return eular;
}