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
offboard_class::offboard_class(ros::NodeHandle* nodehandle):nh(*nodehandle){
	init_publisher();
	init_subscriber();
	init_service();
	// InitControllCallBack();
	ROS_INFO("Init");
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
	calc_timer = nh.createTimer(ros::Duration(0.05), &offboard_class::calc_cb, this);
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
}

// @brief: 初始化订阅句柄（反馈信息）  Mavros发布话题的频率必须不小于20HZ
// @param: void
// @ret: void
// @birth: created by JTJ on 2023.6.4
void offboard_class::init_subscriber()
{
	// *************simulation*************
	// gps定位信息
    // local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
    //     ("/mavros/local_position/pose", 10, &offboard_class::local_pos_cb, this);
	// *************simulation*************
	
	// *************realflight*************
	// 订阅视觉定位信息
    vision_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
        ("mavros/vision_pose/pose", 1, &offboard_class::local_pos_cb,this);
	// *************realflight*************

	// 订阅飞行模式
    state_sub = nh.subscribe<mavros_msgs::State>
		("/mavros/state", 10, &offboard_class::state_cb, this);
	// 订阅速度信息
	vel_sub = nh.subscribe<geometry_msgs::TwistStamped>
		("/mavros/local_position/velocity_body", 10, &offboard_class::vel_cb, this);
	position_sub = nh.subscribe<geometry_msgs::PoseStamped>
		("/camera/ellipse/center", 10, &offboard_class::position_cb, this);
	aamed_sub = nh.subscribe<std_msgs::Bool>
		("/camera/ellipse/Bool", 1, &offboard_class::aamed_cb, this);
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
	// ROS_INFO("local_pos_cb");
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

// @brief: 获取椭圆中心点三位坐标(ENU坐标系)
// @param: const geometry_msgs::PoseStamped::ConstPtr &msg  enu坐标下的坐标信息指针
// @ret: void
// @birth: created by JTJ on 2023.6.28
void offboard_class::position_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
	position = *msg;
	// ROS_INFO_STREAM("x , y ,z : " << position.pose.position.x << " " << position.pose.position.y << " " << position.pose.position.z);
}

// @brief: 是否识别到椭圆
// @param: const geometry_msgs::PoseStamped::ConstPtr &msg  enu坐标下的坐标信息指针
// @ret: void
// @birth: created by JTJ on 2023.6.28
void offboard_class::aamed_cb(const std_msgs::Bool::ConstPtr &msg)
{
	p_s = *msg;
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
inline double offboard_class::Speed(double vel)
{
	if(vel > 0)
		return vel = (vel > MaxSpeed) ? MaxSpeed : vel;
	else
		return vel = (vel < -MaxSpeed) ? -MaxSpeed : vel;
}

// @brief: 对水平速度进行限制
// @param: double vel  输入的速度
// @ret: double    速度
// @birth: created by JTJ on 2023.6.4
inline double offboard_class::SpeedPlane(double vel)
{
	if(vel > 0)
		return vel = (vel > MaxSpeedPlane) ? MaxSpeedPlane : vel;
	else
		return vel = (vel < -MaxSpeedPlane) ? -MaxSpeedPlane : vel;
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
	
	increase_vel_x = KP * (err_x - err_next_x) + KI * err_x + KD * (err_x - 2 * err_next_x + err_last_x);
	increase_vel_y = KP * (err_y - err_next_y) + KI * err_y + KD * (err_y - 2 * err_next_y + err_last_y);
	increase_vel_z = KP * (err_z - err_next_z) + KI * err_z + KD * (err_z - 2 * err_next_z + err_last_z);
	
	exp_vel.linear.x += increase_vel_x;
	exp_vel.linear.y += increase_vel_y;
	exp_vel.linear.z += increase_vel_z;

	err_last_x = err_next_x;
	err_next_x = err_x;

	err_last_y = err_next_y;
	err_next_y = err_y;

	err_last_z = err_next_z;
	err_next_z = err_z;

	exp_vel.linear.x = Speed(exp_vel.linear.x);
	exp_vel.linear.y = Speed(exp_vel.linear.y);
	exp_vel.linear.z = Speed(exp_vel.linear.z);

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
	err_z_P = this->pose.pose.position.z - local_pos.pose.position.z;
	
	// ROS_INFO_STREAM("err_x_P err_y_P err_z_P :" << err_x_P << " " << err_y_P << " " << err_z_P);

	increase_vel_x_P = KP_P * (err_x_P - err_next_x_P) + KI_P * err_x_P + KD_P * (err_x_P - 2 * err_next_x_P + err_last_x_P);
	increase_vel_y_P = KP_P * (err_y_P - err_next_y_P) + KI_P * err_y_P + KD_P * (err_y_P - 2 * err_next_y_P + err_last_y_P);
	increase_vel_z_P = KP * (err_z_P - err_next_z_P) + KI * err_z_P + KD * (err_z_P - 2 * err_next_z_P + err_last_z_P);
	
	exp_vel_P.linear.x += increase_vel_x_P;
	exp_vel_P.linear.y += increase_vel_y_P;
	exp_vel_P.linear.z += increase_vel_z_P;

	err_last_x_P = err_next_x_P;
	err_next_x_P = err_x_P;

	err_last_y_P = err_next_y_P;
	err_next_y_P = err_y_P;

	err_last_z_P = err_next_z_P;
	err_next_z_P = err_z_P;

	exp_vel_P.linear.x = SpeedPlane(exp_vel_P.linear.x);
	exp_vel_P.linear.y = SpeedPlane(exp_vel_P.linear.y);
	exp_vel_P.linear.z = SpeedPlane(exp_vel_P.linear.z);

	ROS_INFO("exp_vel_P_x : %lf exp_vel_P_y : %lf exp_vel_P_z : %lf", exp_vel_P.linear.x, exp_vel_P.linear.y, exp_vel_P.linear.z);

}

// @brief: 获取最大速度
// @param: void
// @ret: double 最大速度
// @birth: created by JTJ on 2023.6.4
inline double offboard_class::getMaxSpeed()
{
	return this->MaxSpeed;
}

// @brief: 设置最大速度
// @param: double vel  最大速度
// @ret: void
// @birth: created by JTJ on 2023.6.4
inline void offboard_class::setMaxSpeed(double vel)
{
	this->MaxSpeed = vel;
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
	pose.pose.position.x = x;
	pose.pose.position.y = y;
	pose.pose.position.z = z;
	position.pose.position.x = x;
	position.pose.position.y = y;
	position.pose.position.z = z;
	ROS_INFO_STREAM("Pose is " << pose.pose.position.x << ',' << pose.pose.position.y << ',' << pose.pose.position.z);
}

// @brief: 飞到该航点
// @param: const int st, const int step    悬停计数器,下一部
// @ret: void
// @birth: created by JTJ on 2023.6.4
inline void offboard_class::flightWaypoint(const int st, const int Nextstep)
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

// @brief: 飞到该航点
// @param: const int st, const int step    悬停计数器,下一部
// @ret: void
// @birth: created by JTJ on 2023.6.4
inline void offboard_class::flightWaypointPlane(const int st, const int Nextstep, const int Laststep)
{
	ROS_INFO("flightWaypointPlane before");
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
	else
	{	
		ROS_INFO("flightWaypointPlane false");
		step = Laststep;
		last_position.pose.position.x = local_pos.pose.position.x;
		last_position.pose.position.y = local_pos.pose.position.y;
		last_position.pose.position.z = pose.pose.position.z;
	}
}

// @brief: 飞行行点控制主逻辑（每20HZ调用一次）
// @param: const ros::TimerEvent& event  用于计时，每20HZ调用一次
// @ret: void
// @birth: created by JTJ on 2023.6.4
void offboard_class::calc_cb(const ros::TimerEvent& event)
{
	// ROS_INFO("calc_cb");
	switch (step)
	{
	case 0:
		ROS_INFO("\n");
		ROS_INFO("First Waypoint");
		setWaypoint(0, 0, 0.5);
		flightWaypoint(100, 0);
		ROS_INFO("First Waypoint end");
		break;
	case 1:
		ROS_INFO("*************************************");
		ROS_INFO("Ellipse direction after");
		flightWaypointPlane(100, 1, 2);
		ROS_INFO("Ellipse direction before");
		ROS_INFO("*************************************");
		break;
	case 2:
		setWaypoint(last_position.pose.position.x, last_position.pose.position.y, last_position.pose.position.z);
		flightWaypoint(100, 1);
		break;
	case 3:
		setWaypoint(1, 0, 1);
		flightWaypoint(100, 5);
		break;
	// 降落
	case 5:
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
