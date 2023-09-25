#pragma once

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/HomePosition.h>

#include <nav_msgs/Odometry.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <tf/tf.h>
#include <ros/ros.h>
#include <math.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "DBSCAN/dbscan.h"


namespace CADC{

class offboard_class
{
private:
    ros::Publisher local_pos_pub;
    ros::Publisher pos_pub_FLU;
    ros::Publisher vec_pub;
    ros::Publisher acc_pub;
    ros::Publisher raw_pub_att;
    ros::Publisher step_pub;
    ros::Publisher home_position_pub;
    ros::Publisher servo_pub;
    ros::Publisher center_distence_pub;

	ros::Subscriber local_pos_sub;  
    ros::Subscriber vision_pos_sub;   
	ros::Subscriber state_sub ; 
    ros::Subscriber vel_sub;
    ros::Subscriber body_level_position_sub;
    ros::Subscriber global_position_sub;
    ros::Subscriber aamed_sub;
    ros::Subscriber global_aamed_sub;
    ros::Subscriber odom_sub;
    ros::Subscriber servo_vision_sub;
    ros::Subscriber hp_sub;

    // MAX SPEED
    double MaxSpeed_Z = 3;
    double MaxSpeed_X = 3;
    double MaxSpeed_Y = 3;
    
    double MaxAccel = 5;

    // MAX SPEED
    // double MaxSpeedPlane_X = 0.05;
    // double MaxSpeedPlane_Y = 0.05;

    double MaxSpeedPlane_X = 0.07;
    double MaxSpeedPlane_Y = 0.07;

    // Quaternion
    geometry_msgs::Quaternion q;
    mavros_msgs::AttitudeTarget raw_att;

    double precision = 0.1;

    // throtther
    float thr;
    //define thrust MAX 
    float THRUST_FULL=60;
    float thrust_scale(float thrust);

    ros::NodeHandle nh;//we will need this, to pass between "main" and constructor
    void init_publisher();
    void init_subscriber();
    void init_service();

    void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void state_cb(const mavros_msgs::State::ConstPtr& msg);
    void vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg);
    void position_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void global_position_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void aamed_cb(const std_msgs::Bool::ConstPtr &msg);
    void global_aamed_cb(const std_msgs::Bool::ConstPtr &msg);
    void odomCallback(const nav_msgs::OdometryConstPtr &odom);
    void servo_vision_cb(const std_msgs::Bool::ConstPtr &msg);
    void hp_cb(const mavros_msgs::HomePositionConstPtr &msg);
    // velocity P control
    // *************simulation*************
    // double KP = 0.9, KI = 0, KD = 0.3;
    // *************simulation*************

    // *************realflight*************
    // double KP = 1.6, KI = 0, KD = 0.005;
    // double KP = 1.5, KI = 0.001, KD = 0.04;
    // double KP = 1.4, KI = 0, KD = 0;
    // *************realflight*************

    // double KP_Z = 2.0, KI_Z = 0.007, KD_Z = 0.4;
    // double KP_Z = 1.7, KI_Z = 0.001, KD_Z = 0.4;
    // double KP_Z = 1.5, KI_Z = 0.0001, KD_Z = 0;
    // // double KP_Y = 2, KI_Y = 0.004, KD_Y = 0.7;
    // double KP_Y = 1, KI_Y = 0.0001, KD_Y = 0.3;
    // double KP_X = 1, KI_X = 0.0001, KD_X = 0.3;

    double KP_Z = 2.0, KI_Z = 0.007, KD_Z = 0.4;
    double KP_Y = 2.5, KI_Y = 0.001, KD_Y = 0.5;
    double KP_X = 2.5, KI_X = 0.001, KD_X = 0.4;

	double err_x = 0;     double err_last_x = 0;		double err_next_x = 0;
	double err_y = 0;     double err_last_y = 0;		double err_next_y = 0;
	double err_z = 0;     double err_last_z = 0;		double err_next_z = 0;
	double increase_vel_x, increase_vel_y, increase_vel_z;
    // control flight velocity(expect)
	geometry_msgs::Twist exp_vel;

    // velocity P control plane
    // 空载和满载都比较不错，不过相应较慢
    // double KP_X_P = 0.14, KI_X_P = 0.00008, KD_X_P = 0.045;
    // double KP_Y_P = 0.14, KI_Y_P = 0.00008, KD_Y_P = 0.045;

    double KP_X_P = 0.15, KI_X_P = 0.0001, KD_X_P = 0.045;
    double KP_Y_P = 0.15, KI_Y_P = 0.0001, KD_Y_P = 0.045;

	double err_x_P = 0;     double err_last_x_P = 0;		double err_next_x_P = 0;
	double err_y_P = 0;     double err_last_y_P = 0;		double err_next_y_P = 0;
	double err_z_P = 0;     double err_last_z_P = 0;		double err_next_z_P = 0;
	double increase_vel_x_P, increase_vel_y_P, increase_vel_z_P;

    // control flight plane velocity(expect)
	geometry_msgs::Twist exp_vel_P;

    double acc_KP = 1.1, acc_KI = 0.01, acc_KD = 0.01;
	double err_vel_x = 0;     double err_last_vel_x = 0;		double err_next_vel_x = 0;
	double err_vel_y = 0;     double err_last_vel_y = 0;		double err_next_vel_y = 0;
	double err_vel_z = 0;     double err_last_vel_z = 0;		double err_next_vel_z = 0;
	double increase_acc_x, increase_acc_y, increase_acc_z;
    // control flight acceleration(expect)
    geometry_msgs::TwistStamped real_vel;
    geometry_msgs::Vector3Stamped exp_acc;

    // velocity & acceleration Control
    void local_pos_control(const geometry_msgs::PoseStamped &pose);
    void local_pos_controlPlane(const geometry_msgs::PoseStamped &pose);

    // flight step & counter
    int step = 0;
	int sametimes = 0;

    // home position
    bool ds_home_sign = false;
    bool Rotation_body_ENU = false;
    u_int16_t R_count = 0;
    const int pointSetSize = 100;
    DBSCAN::DBSCAN ds_home;
    DBSCAN::Point tmp;
    geometry_msgs::PoseStamped home_position_;      // enu坐标系下的起飞点坐标
    mavros_msgs::HomePosition home_cb;

    // 旋转角
    Eigen::Vector3d eular;
    Eigen::Vector3d home_eular;
    Eigen::Matrix3d R_ENU_B_ = Eigen::Matrix3d::Identity();

    const double high = 2; // 飞行高度

    std_msgs::Float32 center_distance;
public:
    ros::ServiceClient arming_client;
    ros::ServiceClient set_mode_client;

    offboard_class(ros::NodeHandle* nodehandle);
    offboard_class(ros::NodeHandle* nodehandle, DBSCAN::DBSCAN ds);

    ~offboard_class();
    
    ros::Timer calc_timer;

    // ned to enu
    geometry_msgs::Point ned2enu_angle(geometry_msgs::Point ned_angle);
    geometry_msgs::Quaternion euler_to_quat(geometry_msgs::Point euler_angle_enu);

    // Eigen quat to euler
    Eigen::Vector3d quat_to_euler(const geometry_msgs::Quaternion &q);

    // Eigen::Matrix3d 

    double getPrecision(){ return precision; };
    void setPrecision(double p){ precision = p; };

    void SpeedControl(const geometry_msgs::PoseStamped &pose);

    void SpeedControlPlane(const geometry_msgs::PoseStamped &pose);

    // void SpeedControlPlaneY(const geometry_msgs::PoseStamped &pose);

    void AccelControl(const geometry_msgs::PoseStamped &pose);
    void AngleControl(double x, double y, double z);

    virtual void calc_cb(const ros::TimerEvent&);

    void InitControllCallBack();

    // Judge speed legality 
    double SpeedZ(double vel);
    double SpeedX(double vel);
    double SpeedY(double vel);
    double SpeedPlaneX(double vel);
    double SpeedPlaneY(double vel);
    double Acceleration(double acc);

    double getMaxSpeedZ();
    double getMaxSpeedX();
    double getMaxSpeedY();
    void setMaxSpeedZ(double vel);
    void setMaxSpeedX(double vel);
    void setMaxSpeedY(double vel);

    double getMaxAccel();
    void setMaxAccel(double acc);

    bool chargePositionAegality(geometry_msgs::PoseStamped &local_pos, geometry_msgs::PoseStamped &pose);
    bool chargePositionAegalityPlane(geometry_msgs::PoseStamped &pose);

    void flightWaypoint(const int st, const int Nextstep);
    void flightWaypointPlane(const int st, const int Nextstep, const int Laststep);
    void flightWayPointGlobal(const int st, const int Nextstep);
    void setWaypoint(double x, double y, double z);

    void setWaypointByPosition(double x, double y, double z);
    void flightWaypointLocal(const int st, const int Nextstep);

    void chooseHomePosition(DBSCAN::DBSCAN &ds);

    void setWaypointByPosition_FLU(double x, double y, double z);   // 以起飞点为原点前左上是坐标系方向（FLU）
    void flightWaypointFLU(const int st, const int Nextstep);
    bool chargePositionAegalityFLU(geometry_msgs::PoseStamped &local_pos, mavros_msgs::PositionTarget &pose);

    Eigen::Vector3d Body2ENU(double x, double y, double z);
    
    mavros_msgs::PositionTarget p_FLU;    // 以起飞点为原点前左上是坐标系方向（FLU）
    geometry_msgs::PoseStamped p_local;
	geometry_msgs::PoseStamped pose;
    geometry_msgs::PoseStamped position;      // 机体的相对位置
    geometry_msgs::PoseStamped global_position;     // 以起飞点为世界坐标系的位置      
    geometry_msgs::PoseStamped last_position;

    bool detect_sign = false;       // 是否识别到投放点
    geometry_msgs::PoseStamped current_point;       // 投放点
    std_msgs::Bool servo_vision;
    std_msgs::Bool p_s;     // 机体坐标系下的目标点是否被接收
    std_msgs::Bool g_p_s;   // 世界坐标系（飞机起飞为原点）下的目标点是否被接收

    mavros_msgs::State current_state;
    geometry_msgs::PoseStamped local_pos;
    mavros_msgs::SetMode offb_set_mode;
    mavros_msgs::CommandBool arm_cmd;
    ros::Time last_request;
    ros::Time time_servo;
    ros::Time time_servo_last;

    geometry_msgs::Point enu_euler;
    geometry_msgs::Point ned_euler;
    int count_servo = 100;
    int loop = 10;
};

}