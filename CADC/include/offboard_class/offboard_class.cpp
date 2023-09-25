#include "offboard_class.h"

namespace CADC{
// @brief: offboard_class 解构函数
// @param: void
// @ret: void
// @birth: created by JTJ on 2023.6.4
offboard_class::~offboard_class()
{

}

// @brief: offboard_class 构造函数  Mavros发布话题的频率必须不小于20HZ
// @param: void
// @ret: void
// @birth: created by JTJ on 2023.6.4
offboard_class::offboard_class(ros::NodeHandle* nodehandle):nh(*nodehandle)
{
	init_publisher();
	init_subscriber();
	init_service();
	// InitControllCallBack();
	ROS_INFO("Init");
	exp_vel.linear.x = 0, exp_vel.linear.y = 0, exp_vel.linear.z = 0;
	exp_acc.vector.x = 0, exp_acc.vector.y = 0; exp_acc.vector.z = 0;
	ned_euler.x = 0, ned_euler.y = 0, ned_euler.z = 0;
}

// @brief: offboard_class 构造函数  Mavros发布话题的频率必须不小于20HZ
// @param: void
// @ret: void
// @birth: created by JTJ on 2023.6.4
offboard_class::offboard_class(ros::NodeHandle* nodehandle, DBSCAN::DBSCAN ds):nh(*nodehandle), ds_home(ds)
{
	init_publisher();
	init_subscriber();
	init_service();
	InitControllCallBack();
	ROS_INFO("Init");
	ds_home_sign = false;
	Rotation_body_ENU = false;
	exp_vel.linear.x = 0, exp_vel.linear.y = 0, exp_vel.linear.z = 0;
	exp_acc.vector.x = 0, exp_acc.vector.y = 0; exp_acc.vector.z = 0;
	ned_euler.x = 0, ned_euler.y = 0, ned_euler.z = 0;
}

// @brief: 初始化控制  Mavros发布话题的频率必须不小于20HZ
// @param: void
// @ret: void
// @birth: created by JTJ on 2023.6.4
void offboard_class::InitControllCallBack()
{
	calc_timer = nh.createTimer(ros::Duration(0.03), &offboard_class::calc_cb, this);
}

// @brief: 初始化发布句柄（控制信息）  Mavros发布话题的频率必须不小于20HZ
// @param: void
// @ret: void
// @birth: created by JTJ on 2023.6.4
void offboard_class::init_publisher()
{
	// 发布行点信息
    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
        ("/mavros/setpoint_position/local", 10,this);
	pos_pub_FLU = nh.advertise<mavros_msgs::PositionTarget>
        ("/mavros/setpoint_raw/local", 10,this);
	// 发布速度信息
    vec_pub = nh.advertise<geometry_msgs::Twist>
        ("/mavros/setpoint_velocity/cmd_vel_unstamped", 10, this);
	// 发布加速度信息
	acc_pub = nh.advertise<geometry_msgs::Vector3Stamped>
		("/mavros/setpoint_accel/accel", 10, this);
	// 发布姿态信息
	raw_pub_att = nh.advertise<mavros_msgs::AttitudeTarget>
    	("/mavros/setpoint_raw/attitude",100, this);
	step_pub = nh.advertise<std_msgs::Bool>
		("/cadc_controll/detect", 1, this);
	home_position_pub = nh.advertise<geometry_msgs::PoseStamped>
		("/CADC/HomePosition", 1, this);
	servo_pub = nh.advertise<std_msgs::Bool>
		("/ServoControl", 10, this);

	center_distence_pub = nh.advertise<std_msgs::Float32>
		("/camera/center/distence", 10, this);
}

// @brief: 初始化订阅句柄（反馈信息）  Mavros发布话题的频率必须不小于20HZ
// @param: void
// @ret: void
// @birth: created by JTJ on 2023.6.4
void offboard_class::init_subscriber()
{
	// *************gps*************
	// gps定位信息
    local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
        ("/mavros/local_position/pose", 10, &offboard_class::local_pos_cb, this);
	// 里程计信息
	odom_sub = nh.subscribe<nav_msgs::Odometry>
        ("/mavros/local_position/odom", 1, &offboard_class::odomCallback, this);
	// *************gps*************
	
	// *************vision*************
	// 订阅视觉定位信息
    // vision_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
    //     ("mavros/vision_pose/pose", 1, &offboard_class::local_pos_cb,this);
	// *************vision*************

	// 订阅飞行模式
    state_sub = nh.subscribe<mavros_msgs::State>
		("/mavros/state", 10, &offboard_class::state_cb, this);
	// 订阅速度信息
	vel_sub = nh.subscribe<geometry_msgs::TwistStamped>
		("/mavros/local_position/velocity_body", 10, &offboard_class::vel_cb, this);
	// 订阅机体坐标系下的目标点
	body_level_position_sub = nh.subscribe<geometry_msgs::PoseStamped>
		("/camera/ellipse/body/level/center", 1, &offboard_class::position_cb, this);
	// 订阅世界坐标系下的目标点
	global_position_sub = nh.subscribe<geometry_msgs::PoseStamped>
		("/camera/ellipse/center", 10, &offboard_class::global_position_cb, this);
	// 订阅是否接受到机体坐标系下的目标点
	aamed_sub = nh.subscribe<std_msgs::Bool>
		("/camera/ellipse/bodyBool", 1, &offboard_class::aamed_cb, this);
	global_aamed_sub = nh.subscribe<std_msgs::Bool>
		("/camera/ellipse/globalBool", 10, &offboard_class::global_aamed_cb, this);
	servo_vision_sub = nh.subscribe<std_msgs::Bool>
		("/ServoControl/vision", 10, &offboard_class::servo_vision_cb, this);

	hp_sub = nh.subscribe<mavros_msgs::HomePosition>
        ("/mavros/home_position/home", 1, &offboard_class::hp_cb, this);
}

// @brief: 初始化服务句柄（用于飞机解锁和设置飞行模式）  Mavros发布话题的频率必须不小于20HZ
// @param: void
// @ret: void
// @birth: created by JTJ on 2023.6.4
void offboard_class::init_service()
{
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>
		("mavros/cmd/arming",this);
	set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
		("mavros/set_mode",this);
}

// @brief: 获取定位信息
// @param: const geometry_msgs::PoseStamped::ConstPtr &msg  enu坐标下的坐标信息指针
// @ret: void
// @birth: created by JTJ on 2023.6.4
void offboard_class::local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    local_pos = *msg;
	if(!ds_home_sign)
	{	
		if(local_pos.pose.position.x != 0 || local_pos.pose.position.y != 0)
		{	
			tmp.x = local_pos.pose.position.x;
			tmp.y = local_pos.pose.position.y;
			tmp.z = local_pos.pose.position.z;
			tmp.clusterID = UNCLASSIFIED;
			ds_home.m_points.push_back(tmp);
			if(ds_home.m_points.size() > pointSetSize)
			{
				ds_home.run();
				chooseHomePosition(ds_home);
				ds_home_sign = true;
			}
		}
	}
	else
		home_position_pub.publish(home_position_);
}


// @brief: 获取里程计信息
// @param: const nav_msgs::OdometryConstPtr &odom  enu坐标下的坐标信息指针
// @ret: void
// @birth: created by JTJ on 2023.6.4
void offboard_class::odomCallback(const nav_msgs::OdometryConstPtr &odom)
{
    tf::Quaternion quat;
    tf::quaternionMsgToTF(odom->pose.pose.orientation, quat);

    double roll, pitch, yaw;//定义存储r\p\y的容器
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
    // ROS_INFO("*************************************");
    // ROS_INFO_STREAM("ROLL : " << roll * 180 / M_PI);
    // ROS_INFO_STREAM("PITCH : " << pitch * 180 / M_PI);
    // ROS_INFO_STREAM("YAW : " << yaw * 180 / M_PI);
    // ROS_INFO("*************************************");
	if(!Rotation_body_ENU)
	{
	// 	// 计算机体坐标系到ENU坐标系的旋转矩阵
		eular << roll, pitch, yaw;
		tf::Matrix3x3 /*R_B_ENU,*/ R_ENU_B;

	// 	tf::Quaternion q_h;
	// 	tf::quaternionMsgToTF(home_cb.orientation, q_h);
	// 	double r, p, y;
	// 	tf::Matrix3x3(q_h).getRPY(r, p, y);
		// R_B_ENU.setRPY(roll, pitch, yaw);
		R_ENU_B.setRotation(quat);
	// 	// R_ENU_B = R_B_ENU.inverse();
	// 	// R_ENU_B.setRPY(roll, pitch, yaw - M_PI * 1 / 18);
	// 	// R_ENU_B_ << cos(-yaw) , 0, 0,
	// 	// 			0, cos(M_PI_2 + yaw), 0,
	// 	// 			0, 0, 1;
	// 	R_ENU_B.setRPY(r, p, y);
		R_ENU_B_ << R_ENU_B[0][0], R_ENU_B[0][1], R_ENU_B[0][2],
					R_ENU_B[1][0], R_ENU_B[1][1], R_ENU_B[1][2],
					R_ENU_B[2][0], R_ENU_B[2][1], R_ENU_B[2][2];
		R_count++;
		if(R_count >= 5)			
			Rotation_body_ENU = true;
	}
}

// @brief: 获取飞行模式
// @param: const mavros_msgs::State::ConstPtr &msg  飞行模式指针
// @ret: void
// @birth: created by JTJ on 2023.6.4
void offboard_class::state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
	// ROS_INFO("state_cb");
}

// @brief: 获取速度反馈
// @param: const geometry_msgs::TwistStamped::ConstPtr &msg  速度信息指针
// @ret: void
// @birth: created by JTJ on 2023.6.4
void offboard_class::vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
	real_vel = *msg;
}

// @brief: 获取椭圆中心点三位坐标(机体坐标系)
// @param: const geometry_msgs::PoseStamped::ConstPtr &msg  enu坐标下的坐标信息指针
// @ret: void
// @birth: created by JTJ on 2023.6.28
void offboard_class::position_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
	position = *msg;
	// ROS_INFO_STREAM("x , y ,z : " << position.pose.position.x << " " << position.pose.position.y << " " << position.pose.position.z);
}

// @brief: 获取椭圆中心点三位坐标(以飞机起飞点为世界坐标系)
// @param: const geometry_msgs::PoseStamped::ConstPtr &msg  enu坐标下的坐标信息指针
// @ret: void
// @birth: created by JTJ on 2023.6.28
void offboard_class::global_position_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
	global_position = *msg;
}

// @brief: 是否识别到椭圆(机体坐标系)
// @param: const geometry_msgs::PoseStamped::ConstPtr &msg  enu坐标下的坐标信息指针
// @ret: void
// @birth: created by JTJ on 2023.6.28
void offboard_class::aamed_cb(const std_msgs::Bool::ConstPtr &msg)
{
	p_s = *msg;
}

// @brief: 是否识别到椭圆（世界坐标系）
// @param: const geometry_msgs::PoseStamped::ConstPtr &msg  enu坐标下的坐标信息指针
// @ret: void
// @birth: created by JTJ on 2023.6.28
void offboard_class::global_aamed_cb(const std_msgs::Bool::ConstPtr &msg)
{
	g_p_s = *msg;
}

void offboard_class::servo_vision_cb(const std_msgs::Bool::ConstPtr &msg)
{
	servo_vision = *msg;
}

void offboard_class::hp_cb(const mavros_msgs::HomePositionConstPtr &msg)
{
	home_cb = *msg;
	// if(!Rotation_body_ENU)
	// {
	// 	tf::Matrix3x3 R_ENU_B;
	// 	tf::Quaternion q_h;
	// 	tf::quaternionMsgToTF(home_cb.orientation, q_h);
	// 	double r, p, y;
	// 	tf::Matrix3x3(q_h).getRPY(r, p, y);
	// 	home_eular << r, p, y;
	// 	// R_ENU_B.setRPY(0, 0, y - 6.5 * M_PI / 180);
	// 	R_ENU_B.setRPY(0, 0, y - 8 * M_PI / 180);
	// 	// R_ENU_B.setRPY(0, 0, y);
	// 	R_ENU_B_ << R_ENU_B[0][0], R_ENU_B[0][1], R_ENU_B[0][2],
	// 				R_ENU_B[1][0], R_ENU_B[1][1], R_ENU_B[1][2],
	// 				R_ENU_B[2][0], R_ENU_B[2][1], R_ENU_B[2][2];
	// 	// R_count++;
	// 	// if(R_count >= 2)			
	// 	Rotation_body_ENU = true;
	// }
}

// @brief: 进行速度控制(加速度控制不稳定，建议使用速度控制)
// @param: const geometry_msgs::PoseStamped &pose   enu坐标系下的三维坐标
// @ret: void
// @birth: created by JTJ on 2023.6.4
void offboard_class::SpeedControl(const geometry_msgs::PoseStamped &pose)
{
	local_pos_control(pose);
	vec_pub.publish(exp_vel);
}

// @brief: 进行速度控制(加速度控制不稳定，建议使用速度控制)
// @param: const geometry_msgs::PoseStamped &pose   enu坐标系下的三维坐标
// @ret: void
// @birth: created by JTJ on 2023.6.4
void offboard_class::SpeedControlPlane(const geometry_msgs::PoseStamped &pose)
{
	local_pos_controlPlane(pose);
	vec_pub.publish(exp_vel_P);
}

// @brief: 进行加速度控制(加速度控制不稳定，建议使用速度控制)
// @param: const geometry_msgs::PoseStamped &pose   enu坐标系下的三维坐标
// @ret: void
// @birth: created by JTJ on 2023.6.4
void offboard_class::AccelControl(const geometry_msgs::PoseStamped &pose)
{
	local_pos_control(pose);
	acc_pub.publish(exp_acc);
}

// @brief: 进行角度控制
// @param: double x, double y, double z XYZ轴逆时针旋转的角度
// @ret: void
// @birth: created by JTJ on 2023.6.4
void offboard_class::AngleControl(double x, double y, double z)
{
	ned_euler.x = x;
	ned_euler.y = y;
	ned_euler.z = z;
	enu_euler = ned2enu_angle(ned_euler);

	q = euler_to_quat(enu_euler);
	thr = thrust_scale(33);

	raw_att.header.stamp = ros::Time::now();

	raw_att.thrust=thr;
	raw_att.orientation.x=q.x;
	raw_att.orientation.y=q.y;
	raw_att.orientation.z=q.z;
	raw_att.orientation.w=q.w;
	raw_pub_att.publish(raw_att);
	ROS_INFO("ANGLE");
}

// @brief: 对加速度进行限制
// @param: double acc  输入的加速度
// @ret: double    加速度
// @birth: created by JTJ on 2023.6.4
inline double offboard_class::Acceleration(double acc)
{
	if(acc > 0)
		return acc = (acc > MaxAccel) ? MaxAccel : acc;
	else
		return acc = (acc < -MaxAccel) ? -MaxAccel : acc;
}

// @brief: 对速度进行限制
// @param: double vel  输入的速度
// @ret: double    速度
// @birth: created by JTJ on 2023.6.4
inline double offboard_class::SpeedZ(double vel)
{
	if(vel > 0)
		return vel = (vel > MaxSpeed_Z) ? MaxSpeed_Z : vel;
	else
		return vel = (vel < -MaxSpeed_Z) ? -MaxSpeed_Z : vel;
}

// @brief: 对速度进行限制
// @param: double vel  输入的速度
// @ret: double    速度
// @birth: created by JTJ on 2023.6.4
inline double offboard_class::SpeedX(double vel)
{
	if(vel > 0)
		return vel = (vel > MaxSpeed_X) ? MaxSpeed_X : vel;
	else
		return vel = (vel < -MaxSpeed_X) ? -MaxSpeed_X : vel;
}
// @brief: 对速度进行限制
// @param: double vel  输入的速度
// @ret: double    速度
// @birth: created by JTJ on 2023.6.4
inline double offboard_class::SpeedY(double vel)
{
	if(vel > 0)
		return vel = (vel > MaxSpeed_Y) ? MaxSpeed_Y : vel;
	else
		return vel = (vel < -MaxSpeed_Y) ? -MaxSpeed_Y : vel;
}

// @brief: 对水平速度进行限制
// @param: double vel  输入的速度
// @ret: double    速度
// @birth: created by JTJ on 2023.6.4
inline double offboard_class::SpeedPlaneX(double vel)
{
	if(vel > 0)
		return vel = (vel > MaxSpeedPlane_X) ? MaxSpeedPlane_X : vel;
	else
		return vel = (vel < -MaxSpeedPlane_X) ? -MaxSpeedPlane_X : vel;
}

// @brief: 对水平速度进行限制
// @param: double vel  输入的速度
// @ret: double    速度
// @birth: created by JTJ on 2023.6.4
inline double offboard_class::SpeedPlaneY(double vel)
{
	if(vel > 0)
		return vel = (vel > MaxSpeedPlane_Y) ? MaxSpeedPlane_Y : vel;
	else
		return vel = (vel < -MaxSpeedPlane_Y) ? -MaxSpeedPlane_Y : vel;
}

// @brief: 输入行点之后，进行速度/加速度控制  （加速度控制不稳定，建议使用速度控制）
// @param: const geometry_msgs::PoseStamped &pose  enu坐标系下的三维坐标
// @ret: void
// @birth: created by JTJ on 2023.6.4
inline void offboard_class::local_pos_control(const geometry_msgs::PoseStamped &pose)
{
	// 误差
	err_x = pose.pose.position.x - local_pos.pose.position.x;
	err_y = pose.pose.position.y - local_pos.pose.position.y;
	err_z = pose.pose.position.z - local_pos.pose.position.z;
	
	increase_vel_x = KP_X * (err_x - err_next_x) + KI_X * err_x + KD_X * (err_x - 2 * err_next_x + err_last_x);
	increase_vel_y = KP_Y * (err_y - err_next_y) + KI_Y * err_y + KD_Y * (err_y - 2 * err_next_y + err_last_y);
	increase_vel_z = KP_Z * (err_z - err_next_z) + KI_Z * err_z + KD_Z * (err_z - 2 * err_next_z + err_last_z);
	
	exp_vel.linear.x += increase_vel_x;
	exp_vel.linear.y += increase_vel_y;
	exp_vel.linear.z += increase_vel_z;

	err_last_x = err_next_x;
	err_next_x = err_x;

	err_last_y = err_next_y;
	err_next_y = err_y;

	err_last_z = err_next_z;
	err_next_z = err_z;

	exp_vel.linear.x = SpeedX(exp_vel.linear.x);
	exp_vel.linear.y = SpeedY(exp_vel.linear.y);
	exp_vel.linear.z = SpeedZ(exp_vel.linear.z);

	// err_vel_x = exp_vel.linear.x - real_vel.twist.linear.x;
	// err_vel_y = exp_vel.linear.y - real_vel.twist.linear.y;
	// err_vel_z = exp_vel.linear.z - real_vel.twist.linear.z;
	
	// if(real_vel.twist.linear.x < MaxSpeed || real_vel.twist.linear.y < MaxSpeed || real_vel.twist.linear.y < MaxSpeed)
	// {	
	// 	increase_acc_x = acc_KP * (err_vel_x - err_next_vel_x) + acc_KI * err_vel_x + acc_KD * (err_vel_x - 2 * err_next_vel_x + err_last_vel_x);
	// 	increase_acc_y = acc_KP * (err_vel_y - err_next_vel_y) + acc_KI * err_vel_y + acc_KD * (err_vel_y - 2 * err_next_vel_y + err_last_vel_y);
	// 	increase_acc_z = acc_KP * (err_vel_y - err_next_vel_z) + acc_KI * err_vel_y + acc_KD * (err_vel_z - 2 * err_next_vel_z + err_last_vel_z);
	
	// 	exp_acc.vector.x += increase_acc_x;
	// 	exp_acc.vector.y += increase_acc_y;
	// 	exp_acc.vector.z += increase_acc_z;
	
	// 	err_last_vel_x = err_next_vel_x;
	// 	err_next_vel_x = err_vel_x;

	// 	err_last_vel_y = err_next_vel_y;
	// 	err_next_vel_y = err_vel_y;

	// 	err_last_vel_z = err_next_vel_z;
	// 	err_next_vel_z = err_vel_z;

	// 	exp_acc.vector.x = Acceleration(exp_acc.vector.x);
	// 	exp_acc.vector.y = Acceleration(exp_acc.vector.y);
	// 	exp_acc.vector.z = Acceleration(exp_acc.vector.z);
	// }
	ROS_INFO("exp_vel_x : %lf exp_vel_y : %lf exp_vel_z : %lf", exp_vel.linear.x, exp_vel.linear.y, exp_vel.linear.z);
}

// @brief: 输入行点之后，进行速度/加速度控制  （加速度控制不稳定，建议使用速度控制）
// @param: const geometry_msgs::PoseStamped &pose  enu坐标系下的三维坐标
// @ret: void
// @birth: created by JTJ on 2023.6.4
inline void offboard_class::local_pos_controlPlane(const geometry_msgs::PoseStamped &pose)
{
	// 误差
	err_x_P = pose.pose.position.x;
	err_y_P = pose.pose.position.y;
	err_z_P = 2 - local_pos.pose.position.z;
	ROS_ERROR_STREAM("the Error z is " << this->pose.pose.position.z << ", " << local_pos.pose.position.z << ", " << err_z_P);
	
	// ROS_INFO_STREAM("err_x_P err_y_P err_z_P :" << err_x_P << " " << err_y_P << " " << err_z_P);

	increase_vel_x_P = KP_X_P * (err_x_P - err_next_x_P) + KI_X_P * err_x_P + KD_X_P * (err_x_P - 2 * err_next_x_P + err_last_x_P);
	increase_vel_y_P = KP_Y_P * (err_y_P - err_next_y_P) + KI_Y_P * err_y_P + KD_Y_P * (err_y_P - 2 * err_next_y_P + err_last_y_P);
	increase_vel_z_P = KP_Z * (err_z_P - err_next_z_P) + KI_Z * err_z_P + KD_Z * (err_z_P - 2 * err_next_z_P + err_last_z_P);
	
	exp_vel_P.linear.x += increase_vel_x_P;
	exp_vel_P.linear.y += increase_vel_y_P;
	exp_vel_P.linear.z += increase_vel_z_P;

	err_last_x_P = err_next_x_P;
	err_next_x_P = err_x_P;

	err_last_y_P = err_next_y_P;
	err_next_y_P = err_y_P;

	err_last_z_P = err_next_z_P;
	err_next_z_P = err_z_P;

	exp_vel_P.linear.x = SpeedPlaneX(exp_vel_P.linear.x);
	exp_vel_P.linear.y = SpeedPlaneY(exp_vel_P.linear.y);
	exp_vel_P.linear.z = SpeedZ(exp_vel_P.linear.z);

	ROS_INFO("exp_vel_P_x : %lf exp_vel_P_y : %lf exp_vel_P_z : %lf", exp_vel_P.linear.x, exp_vel_P.linear.y, exp_vel_P.linear.z);

}

// @brief: 获取最大速度
// @param: void
// @ret: double 最大速度
// @birth: created by JTJ on 2023.6.4
inline double offboard_class::getMaxSpeedZ()
{
	return this->MaxSpeed_Z;
}

// @brief: 获取最大速度
// @param: void
// @ret: double 最大速度
// @birth: created by JTJ on 2023.6.4
inline double offboard_class::getMaxSpeedX()
{
	return this->MaxSpeed_X;
}

// @brief: 获取最大速度
// @param: void
// @ret: double 最大速度
// @birth: created by JTJ on 2023.6.4
inline double offboard_class::getMaxSpeedY()
{
	return this->MaxSpeed_Y;
}

// @brief: 设置最大速度
// @param: double vel  最大速度
// @ret: void
// @birth: created by JTJ on 2023.6.4
inline void offboard_class::setMaxSpeedZ(double vel)
{
	this->MaxSpeed_Z = vel;
}

// @brief: 设置最大速度
// @param: double vel  最大速度
// @ret: void
// @birth: created by JTJ on 2023.6.4
inline void offboard_class::setMaxSpeedX(double vel)
{
	this->MaxSpeed_X = vel;
}

// @brief: 设置最大速度
// @param: double vel  最大速度
// @ret: void
// @birth: created by JTJ on 2023.6.4
inline void offboard_class::setMaxSpeedY(double vel)
{
	this->MaxSpeed_Y = vel;
}

// @brief: 获取最大加速度
// @param: void
// @ret: double 最大加速度
// @birth: created by JTJ on 2023.6.4
inline double offboard_class::getMaxAccel()
{
	return this->MaxAccel;
}

// @brief: 设置最大加速度
// @param: double acc  最大加速度
// @ret: void
// @birth: created by JTJ on 2023.6.4
inline void offboard_class::setMaxAccel(double acc)
{
	this->MaxAccel = acc;
}

// @brief: 判断当前的位置是否在精度限制内
// @param: geometry_msgs::PoseStamped &local_pos, geometry_msgs::PoseStamped &pose   真实位置，航点
// @ret: bool	true在精度限制内 false不在精度限制内
// @birth: created by JTJ on 2023.6.4
inline bool offboard_class::chargePositionAegality(geometry_msgs::PoseStamped &local_pos, geometry_msgs::PoseStamped &pose)
{
	if((local_pos.pose.position.x - pose.pose.position.x <= precision && local_pos.pose.position.x - pose.pose.position.x >= -precision) && 
	(local_pos.pose.position.y - pose.pose.position.y <= precision && local_pos.pose.position.y - pose.pose.position.y >= -precision) &&
	(local_pos.pose.position.z - pose.pose.position.z <= precision && local_pos.pose.position.z - pose.pose.position.z >= -precision))
	{
		return true;
	}
	return false;
}

bool offboard_class::chargePositionAegalityFLU(geometry_msgs::PoseStamped &local_pos, mavros_msgs::PositionTarget &pose)
{
	if((local_pos.pose.position.x - pose.position.x <= precision && local_pos.pose.position.x - pose.position.x >= -precision) && 
	(local_pos.pose.position.y - pose.position.y <= precision && local_pos.pose.position.y - pose.position.y >= -precision) &&
	(local_pos.pose.position.z - pose.position.z <= precision && local_pos.pose.position.z - pose.position.z >= -precision))
	{
		return true;
	}
	return false;
}

// @brief: 判断当前的位置是否在精度限制内
// @param: geometry_msgs::PoseStamped &pose   平面目标点
// @ret: bool	true在精度限制内 false不在精度限制内
// @birth: created by JTJ on 2023.6.28
inline bool offboard_class::chargePositionAegalityPlane(geometry_msgs::PoseStamped &pose)
{
	if((pose.pose.position.x <= precision && pose.pose.position.x >= -precision) && 
	(pose.pose.position.y <= precision && pose.pose.position.y >= -precision) &&
	(local_pos.pose.position.z - pose.pose.position.z <= precision && local_pos.pose.position.z - pose.pose.position.z >= -precision))
	{
		return true;
	}
	return false;
}

// @brief: 油门归一化(与姿态控制联合使用)
// @param: float thrust 油门输入值
// @ret: float 返回归一化后的油门数值
// @birth: created by JTJ on 2023.6.4
float offboard_class::thrust_scale(float thrust)
{
	float thrust_scale = 0.0;
	if (thrust>=THRUST_FULL)
	{
		/* code */
		thrust= THRUST_FULL;
	}
	thrust_scale = thrust/ THRUST_FULL;
	return thrust_scale;
}

// @brief: NED坐标系转换为ENU坐标系
// @param: geometry_msgs::Point ned_angle NED坐标系下旋转的角度（欧拉角）
// @ret: geometry_msgs::Point 返回ENU坐标系下的旋转角
// @birth: created by JTJ on 2023.6.4
geometry_msgs::Point offboard_class::ned2enu_angle(geometry_msgs::Point ned_angle)
{
    geometry_msgs::Point enu_angle;
    enu_angle.x = ned_angle.x;
    enu_angle.y = -ned_angle.y;
    enu_angle.z = -ned_angle.z + M_PI/2;
    return enu_angle;
}

// @brief: ENU坐标系下的欧拉角转换为四元数
// @param: geometry_msgs::Point euler_angle_enu  ENU坐标系下的欧拉角
// @ret: geometry_msgs::Quaternion  四元数
// @birth: created by JTJ on 2023.6.4
geometry_msgs::Quaternion offboard_class::euler_to_quat(geometry_msgs::Point euler_angle_enu)
{
	tfScalar yaw,pitch,roll;
    tf::Quaternion q;
    geometry_msgs::Quaternion quat;

    roll = euler_angle_enu.x;
    pitch = euler_angle_enu.y;
    yaw = euler_angle_enu.z;

    q.setEulerZYX(yaw,pitch,roll);
    quat.x = q.x();
    quat.y = q.y();
    quat.z = q.z();
    quat.w = q.w();
    return quat;
}

// @brief: quat to euler
// @param: const geometry_msgs::Quaternion &q   enu坐标系下的四元数
// @ret: Eigen::Vector3d
// @birth: created by JTJ on 2023.6.4
Eigen::Vector3d offboard_class::quat_to_euler(const geometry_msgs::Quaternion &q)
{
	Eigen::Quaterniond q_eigen(q.w, q.x, q.y, q.z);
	// X-Y-Z轴旋转次序
	Eigen::Vector3d eular = q_eigen.matrix().eulerAngles(0,1,2);
	ROS_INFO("X AIXED : %lf", eular(0) * 180 / M_PI);
	ROS_INFO("Y AIXED : %lf", eular(1) * 180 / M_PI);
	ROS_INFO("Z AIXED : %lf", eular(2) * 180 / M_PI);
	return eular;
}

// @brief: 设置ENU坐标系下的航点
// @param: double x, double y, double z  XYZ坐标系的坐标
// @ret: void
// @birth: created by JTJ on 2023.6.4
inline void offboard_class::setWaypoint(double x, double y, double z)
{
	pose.pose.position.x = home_position_.pose.position.x + x;
	pose.pose.position.y = home_position_.pose.position.y + y;
	pose.pose.position.z = home_position_.pose.position.z + z;
	ROS_INFO_STREAM("Pose is " << pose.pose.position.x << ',' << pose.pose.position.y << ',' << pose.pose.position.z);
}

void offboard_class::chooseHomePosition(DBSCAN::DBSCAN &ds)
{
    geometry_msgs::PoseStamped p;
    int lastcount = 0, nowcount = 0;
    int Max = 0,  i = 0;
    std::vector<int> point3d_index;
    for(int j = 1; j <= ds.getClusterPoint(); ++j)
    {
        lastcount = 0;
        point3d_index.clear();
        point3d_index.push_back(lastcount);
        for(i = 0; i < ds.m_points.size(); ++i)
        {
            if(ds.m_points.at(i).clusterID == j)
            {
                ++point3d_index.at(0);
                point3d_index.push_back(i);
            }
        }
        if(point3d_index.at(0) >= Max)
        {
            Max = point3d_index.at(0);
        }
    }
	double x = 0, y = 0, z = 0;
	for(i = 1; i < point3d_index.size(); ++i)
	{
		x += ds.m_points.at(point3d_index.at(i)).x;
        y += ds.m_points.at(point3d_index.at(i)).y;
	}
	x /= (point3d_index.size() - 1);
    y /= (point3d_index.size() - 1);
	home_position_.pose.position.x = x;
	home_position_.pose.position.y = y;
	home_position_.pose.position.z = z;
	ROS_INFO_STREAM("Home position is : " << home_position_.pose.position.x << ", " << home_position_.pose.position.y << ", " << home_position_.pose.position.z);
}

// @brief: 飞到该航点
// @param: const int st, const int step    悬停计数器,下一部
// @ret: void
// @birth: created by JTJ on 2023.6.4
void offboard_class::flightWaypoint(const int st, const int Nextstep)
{
	if(chargePositionAegality(local_pos, pose))
	{
		if(sametimes > st)
		{
			sametimes = 0;
			step = Nextstep;
		}
		else
			sametimes++;
	}
	else
		sametimes = 0;
	SpeedControl(pose);
}

inline void offboard_class::flightWaypointLocal(const int st, const int Nextstep)
{
	ROS_INFO("flightWaypointLocal begin");
	if(chargePositionAegality(local_pos, p_local))
	{
		if(sametimes > st)
		{
			sametimes = 0;
			step = Nextstep;
			ROS_INFO("Step");
		}
		else
			sametimes++;
	}
	else
		sametimes = 0;
	local_pos_pub.publish(p_local);
}

void offboard_class::flightWaypointFLU(const int st, const int Nextstep)
{
	ROS_INFO("flightWaypointFLU begin");
	if(chargePositionAegalityFLU(local_pos, p_FLU))
	{
		if(sametimes > st)
		{
			sametimes = 0;
			step = Nextstep;
			ROS_INFO("Step");
		}
		else
			sametimes++;
	}
	else
		sametimes = 0;
	pos_pub_FLU.publish(p_FLU);
}

// @brief: 飞到该航点
// @param: const int st, const int step    悬停计数器,下一部
// @ret: void
// @birth: created by JTJ on 2023.6.4
inline void offboard_class::flightWaypointPlane(const int st, const int Nextstep, const int Laststep)
{
	ROS_ERROR("flightWaypointPlane before");
	// 投放
	double d_p = sqrt(pow(position.pose.position.x, 2) + pow(position.pose.position.y, 2));
	center_distance.data = d_p;
	center_distence_pub .publish(center_distance);
	ROS_ERROR_STREAM("the plane distance is " << d_p);
	if(d_p <= 0.06 && servo_vision.data)
	{
		detect_sign = true;
		current_point.pose.position = local_pos.pose.position;
		time_servo = ros::Time::now();
		ROS_ERROR("SERVO Begin");
		int count = 100;
		std_msgs::Bool servo_sign;
		servo_sign.data = true;
		while(count > 0)
		{
			servo_pub.publish(servo_sign);
			--count;
		}
		ROS_ERROR("SERVO End");
	}

	// 飞到投放筒正上方
	// 机体坐标系
	if(p_s.data)
	{		
		if(chargePositionAegalityPlane(position))
		{
			if(sametimes > st)
			{
				sametimes = 0;
				step = Nextstep;
			}
			else
				sametimes++;
		}
		else
			sametimes = 0;
		SpeedControlPlane(position);
	}
	// else if(g_p_s.data)
	// {
	// 	// flightWaypointLocal(st, Nextstep);
	// 	flightWayPointGlobal(st, Nextstep);
	// }
	else
	{	
		ROS_INFO("flightWaypointPlane false");
		step = Laststep;
		last_position.pose.position.x = local_pos.pose.position.x;
		last_position.pose.position.y = local_pos.pose.position.y;
		last_position.pose.position.z = high;
		local_pos_pub.publish(last_position);
	}
	if(detect_sign)
	{
		local_pos_pub.publish(current_point);
		if(ros::Time::now() - time_servo >= ros::Duration(5))
		{
			step = Nextstep;
		}
	}
}

// @brief: 飞到该航点
// @param: const int st, const int step    悬停计数器,下一部
// @ret: void
// @birth: created by JTJ on 2023.6.4
void offboard_class::flightWayPointGlobal(const int st, const int Nextstep)
{
	global_position.pose.position.z = high;
	if(chargePositionAegality(local_pos, global_position))
	{
		if(sametimes > st)
		{
			sametimes = 0;
			step = Nextstep;
		}
		else
			sametimes++;
	}
	else
		sametimes = 0;
	// 速度控制
	// SpeedControl(global_position);
	// 位置控制
	local_pos_pub.publish(global_position);
}


inline void offboard_class::setWaypointByPosition(double x, double y, double z)
{
	Eigen::Vector3d hp_ENU_(home_position_.pose.position.x, home_position_.pose.position.y, home_position_.pose.position.z);
	Eigen::Vector3d X_body(x, y, z);
	ROS_ERROR_STREAM("the YAW is : " << eular.z() * 180 / M_PI);
	ROS_ERROR_STREAM("the HOME YAW is : " << home_eular.z() * 180 / M_PI);
	ROS_ERROR_STREAM("The R_ENU_B_ : " << R_ENU_B_);
	// ROS_ERROR_STREAM("Target body x : " << X_body.x());
	// ROS_ERROR_STREAM("Target body y : " << X_body.y());
	// ROS_ERROR_STREAM("Target body z : " << X_body.z());

	Eigen::Vector3d X_ENU = R_ENU_B_ * X_body + hp_ENU_;

	// Eigen::Vector3d X_ENU = Body2ENU(x, y, z);
	p_local.pose.position.x = X_ENU.x();
	p_local.pose.position.y = X_ENU.y();
	p_local.pose.position.z = X_ENU.z();
	ROS_ERROR_STREAM("ENU x : " << p_local.pose.position.x);
	ROS_ERROR_STREAM("ENU y : " << p_local.pose.position.y);
	ROS_ERROR_STREAM("ENU z : " << p_local.pose.position.z);

	// p_local.pose.position.x = home_position_.pose.position.x + x;
	// p_local.pose.position.y = home_position_.pose.position.y + y;
	// p_local.pose.position.z = home_position_.pose.position.z + z;
}

void offboard_class::setWaypointByPosition_FLU(double x, double y, double z)
{	
	// p_FLU.position.x = home_position_.pose.position.x + x;
	// p_FLU.position.y = home_position_.pose.position.y + y;
	// p_FLU.position.z = home_position_.pose.position.x + z;
	// p_FLU.type_mask = mavros_msgs::PositionTarget::IGNORE_PX & mavros_msgs::PositionTarget::IGNORE_PY & mavros_msgs::PositionTarget::IGNORE_PZ;
	p_FLU.type_mask = 1 + 2 /*+ 4 + 8 + 16 + 32*/ + 64 + 128 + 256 + 512 + 1024/* + 2048*/;
	p_FLU.coordinate_frame = 8;
	p_FLU.position.x = x;
	p_FLU.position.y = y;
	p_FLU.position.z = z;
}

inline Eigen::Vector3d offboard_class::Body2ENU(double x, double y, double z)
{
	Eigen::Vector3d hp_ENU_(home_position_.pose.position.x, home_position_.pose.position.y, home_position_.pose.position.z);
	Eigen::Vector3d X_body(x, y, z);
	// Eigen::Vector3d X_ENU = R_ENU_B_ * X_body + hp_ENU_;
	return R_ENU_B_ * X_body + hp_ENU_;
}

// @brief: 飞行行点控制主逻辑（每20HZ调用一次）
// @param: const ros::TimerEvent& event  用于计时，调用为频率20HZ
// @ret: void
// @birth: created by JTJ on 2023.6.4
void offboard_class::calc_cb(const ros::TimerEvent& event)
{
	ROS_INFO_STREAM("Home position is : " << home_position_.pose.position.x << ", " << home_position_.pose.position.y << ", " << home_position_.pose.position.z);
	if(!ds_home_sign)
	{
		ROS_ERROR("The Home Position Not Init");
		return;
	}
	if(!Rotation_body_ENU)
	{
		ROS_ERROR("The R_ENU_B Not Init");
		return;
	}
	switch (step)
	{
	//*******************DEBUG*******************
	// case 0:
	// 	setPrecision(0.1);
	// 	ROS_INFO("Step 0");	
	// 	setWaypointByPosition_FLU(0, 0, 2);
	// 	flightWaypointFLU(100, 1);
	// 	break;
	// case 1:
	// 	setPrecision(0.1);
	// 	ROS_INFO("Step 1");	
	// 	setWaypointByPosition_FLU(2, 0, 2);
	// 	flightWaypointFLU(100, 1);
	// 	break;
	case 0:
		setPrecision(0.1);
		ROS_INFO("Step 0");	
		setWaypointByPosition(0, 0, high);
		flightWaypointLocal(100, 1);
		break;
	case 1:
		setPrecision(0.05);
		ROS_INFO("Step 1");	
		setWaypointByPosition(0, 5, high);
		flightWaypointLocal(100, 2);
		break;
		
	//*******************DEBUG*******************
	//*******************VISION*******************
	// case 1:
	// 	setPrecision(0.1);
	// 	ROS_INFO("Step 1");	
	// 	setWaypointByPosition(5, 0, 2);
	// 	flightWaypointLocal(100, 2);
	// 	break;
	case 2:
		setPrecision(0.08);
		ROS_INFO("*************************************");
		ROS_INFO("Ellipse direction before");
		flightWaypointPlane(100, 3, 2);
		ROS_INFO("Ellipse direction after");
		ROS_INFO("*************************************");
		break;
	case 3:
		setPrecision(0.1);
		ROS_INFO("Step 4");	
		setWaypointByPosition(0, 0, high);
		flightWaypointLocal(100, 8);
		break;
	//*******************VISION*******************
	
	//*******************8.8*******************
	// case 0:
	// 	setPrecision(0.1);
	// 	ROS_INFO("Step 0");	
	// 	setWaypointByPosition(0, 0, 2);
	// 	flightWaypointLocal(100, 1);
	// 	break;
	// case 1:
	// 	setPrecision(0.05);
	// 	ROS_INFO("Step 1");	
	// 	setWaypointByPosition(0, 30.21, 2);
	// 	flightWaypointLocal(100, 2);
	// 	break;
	// case 2:
	// 	while(loop > 0)
	// 	{
	// 		ROS_ERROR("SERVO");
	// 		std_msgs::Bool servo_sign;
	// 		servo_sign.data = true;
	// 		servo_pub.publish(servo_sign);
	// 		loop--;
	// 	}
	// 	step = 3;
	// 	break;
	// case 3:
	// 	setPrecision(0.1);
	// 	ROS_INFO("Step 2");	
	// 	setWaypointByPosition(0, 50, 2);
	// 	flightWaypointLocal(100, 4);
	// 	break;
	// case 4:
	// 	setPrecision(0.05);
	// 	ROS_INFO("Step 4");	
	// 	setWaypointByPosition(0, 52.5, 3);
	// 	flightWaypointLocal(100, 5);
	// 	break;
	// case 5:
	// 	setPrecision(0.05);
	// 	ROS_INFO("Step 5");	
	// 	setWaypointByPosition(-2, 52.5, 3);
	// 	flightWaypointLocal(100, 6);
	// 	break;
	// case 6:
	// 	setPrecision(0.05);
	// 	ROS_INFO("Step 6");	
	// 	setWaypointByPosition(2, 52.5, 3);
	// 	flightWaypointLocal(100, 7);
	// 	break;
	// case 7:
	// 	setPrecision(0.05);
	// 	ROS_INFO("Step 7");	
	// 	setWaypointByPosition(0, 0, 2);
	// 	flightWaypointLocal(100, 8);
	// 	break;
	//*******************8.8*******************

	// case 1:
	// 	setPrecision(0.05);
	// 	ROS_INFO("Step 0");
	// 	setWaypoint(0, 3.5, 2);
	// 	flightWaypoint(100, 2);
	// 	break;
	// case 3:
	// 	ROS_INFO("*************************************");
	// 	ROS_INFO("Ellipse direction before");
	// 	flightWaypointPlane(100, 7, 3);
	// 	ROS_INFO("Ellipse direction after");
	// 	ROS_INFO("*************************************");
	// 	break;
	// case 3:
	// 	setPrecision(0.1);
	// 	setWaypoint(last_position.pose.position.x, last_position.pose.position.y, last_position.pose.position.z);
	// 	flightWaypoint(50, 2);
	// 	break;
	// case 4:
	// 	setPrecision(0.2);
	// 	ROS_INFO("Step 1");
	// 	setWaypoint(0, 50, 3);
	// 	flightWaypoint(100, 4);
	// 	break;
	// case 5:
	// 	setPrecision(0.2);
	// 	ROS_INFO("Step 2");
	// 	setWaypoint(-2, 50, 3);
	// 	flightWaypoint(100, 5);
	// 	break;
	// case 6:
	// 	setPrecision(0.2);
	// 	setWaypoint(2, 50, 1);
	// 	flightWaypoint(100, 6);
	// 	break;
	// case 7:
	// 	setPrecision(0.2);
	// 	setWaypoint(0, 0, 2);
	// 	flightWaypoint(100, 8);
	// 	break;
	// case 7:
	// 	setPrecision(0.2);
	// 	setWaypoint(0, 6, 2);
	// 	flightWaypoint(100, 8);
	case 8:
		offb_set_mode.request.custom_mode = "AUTO.LAND";
		if (current_state.mode != "AUTO.LAND" && (ros::Time::now() - last_request > ros::Duration(5.0)))
		{
			if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
			{
				ROS_INFO("AUTO.LAND enabled");
			}
			last_request = ros::Time::now();
		}
		break;
	default:
		break;
	}
}

}
