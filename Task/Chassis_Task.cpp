#include "Chassis_Task.h"
#include "tasks.h"
#include "arm_math.h"

#include <vector>

Chassis_Ctrl Chassis;
extern bool UI_Send;

	void Chassis_Task(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {		
		
		xQueueSend(Message_Queue, &ID_Data[ChassisData], 0);

    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

//// 底盘初始化
//void Chassis_Ctrl::Chassis_Init(void)
//{
//	RC_Ptr = get_remote_control_point();
//	Mode = CHASSIS_NO_MOVE;

//	PID.Init(&Velocity_Pid, POSITION, VELOCILY_SPEED_PID_KP, VELOCILY_SPEED_PID_KI, VELOCILY_SPEED_PID_KD, VELOCILY_SPEED_PID_MAX_OUT, VELOCILY_SPEED_PID_MAX_IOUT, VELOCILY_SPEED_PID_BAND_I);

//	for(uint8_t i = 0; i < 4; i++)
//	{
//		Motor[i].chassis_motor_measure = CAN_Cmd.Chassis.Get_Motor_Measure_Pointer(i);
//		PID.Init(&Speed_Pid[i], POSITION, M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT, M3505_MOTOR_SPEED_PID_BAND_I);

//	}
//#ifdef useSteering /* useSteering */
//	for(uint8_t i = 0; i < 4; i++)
//	{
//		Steering[i].chassis_motor_measure = CAN_Cmd.Gimbal.Get_Motor_Measure_Pointer(i);
//		PID.Init(&steering_Angle_Pid[i], POSITION, M6020_MOTOR_ANGLE_PID_KP, M6020_MOTOR_ANGLE_PID_KI, M6020_MOTOR_ANGLE_PID_KD, M6020_MOTOR_ANGLE_PID_MAX_OUT, M6020_MOTOR_ANGLE_PID_MAX_IOUT, M6020_MOTOR_ANGLE_PID_BAND_I);
//		PID.Init(&steering_Speed_Pid[i], POSITION, M6020_MOTOR_SPEED_PID_KP, M6020_MOTOR_SPEED_PID_KI, M6020_MOTOR_SPEED_PID_KD, M6020_MOTOR_SPEED_PID_MAX_OUT, M6020_MOTOR_SPEED_PID_MAX_IOUT, M6020_MOTOR_SPEED_PID_BAND_I);

//		Steering[i].offset_ecd = MOTOR_6020_offset[i];
//	}
//#endif
//	//初始化旋转PID
//	PID.Init(&Follow_Gimbal_Pid, POSITION, CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT, CHASSIS_FOLLOW_GIMBAL_PID_BAND_I);
//	
//	// 最大 最小速度
//	Velocity.vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X;
//	Velocity.vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;

//	Velocity.vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y;
//	Velocity.vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;

//	Matrix<3, 1> v0 CHASSIS_SPEED_GEAR_0;
//	Matrix<3, 1> v1 CHASSIS_SPEED_GEAR_1;
//	Matrix<3, 1> v2 CHASSIS_SPEED_GEAR_2;
//	Matrix<3, 1> v3 CHASSIS_SPEED_GEAR_3;

//	Velocity.Speed_Set_m = Matrix<3, 4>::concat_from(v0, v1, v2, v3);

//	Feedback_Update();
//}
//// 数据更新
//void Chassis_Ctrl::Feedback_Update(void)
//{
//	uint8_t i = 0;
//	Velocity.Speed = 0;
//	for(i = 0; i < 4; i++)
//	{
//		// 更新电机速度，加速度是速度的PID微分
//		Motor[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * Motor[i].chassis_motor_measure->speed_rpm;
//		Motor[i].accel = Speed_Pid[i].Dbuf[0] * CHASSIS_CONTROL_FREQUENCE;
//		Velocity.Speed += abs(Motor[i].speed);
//	}
//	Velocity.Speed /= 4;
//	Velocity.Gear = Message.robo->game_robot_state.robot_level;

//	chassis_relative_ECD = Message.GimbalR.ECD;
//	chassis_relative_RAD = chassis_relative_ECD * ECD_TO_PI;
//#ifdef useMecanum
//	// 更新底盘前进速度 x，平移速度y，旋转速度wz，坐标系为右手系
//	Velocity.vx = (-Motor[0].speed + Motor[1].speed + Motor[2].speed - Motor[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
//	Velocity.vy = (-Motor[0].speed - Motor[1].speed + Motor[2].speed + Motor[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
//	Velocity.wz = (-Motor[0].speed - Motor[1].speed - Motor[2].speed - Motor[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_WZ / MOTOR_DISTANCE_TO_CENTER;
//#endif
//#ifdef useSteering
//	for(i = 0; i < 4; i++)
//	{
//		// 更新电机速度，加速度是速度的PID微分
//		Steering[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * Motor[i].chassis_motor_measure->speed_rpm;
//		Steering[i].accel = steering_Speed_Pid[i].Dbuf[0] * CHASSIS_CONTROL_FREQUENCE;
//		Steering[i].data.angle_last = Steering[i].data.angle;
//		Steering[i].data.angle = motor_ecd_to_relative_ecd(Steering[i].chassis_motor_measure->ecd, Steering[i].offset_ecd);
//		Steering[i].data.angle_set_last = Steering[i].data.angle_set;
//	}
//#endif
//}
//// 底盘行为状态设置
//void Chassis_Ctrl::Behaviour_Mode(void)
//{
//	if(switch_is_up(RC_Ptr->rc.s[CHANNEL_RIGHT]))
//		Flags.RC_Flag = false;
//	else
//		Flags.RC_Flag = true;
//	// 按键控制
//	if(Flags.RC_Flag == false)
//	{
//		if(RC.read_key(&RC.Key.C, single, true))
//		{
//			Mode = CHASSIS_NO_MOVE; // 底盘保持不动
//		}
//		if(RC.read_key(&RC.Key.X, single, true))
//		{
//			if(Mode == CHASSIS_NO_MOVE || Mode == CHASSIS_FOLLOW_YAW || Mode == CHASSIS_LITTLE_TOP)
//			{
//				Mode = CHASSIS_NO_FOLLOW_YAW; // 底盘不跟随云台
//			}
//			else if(Mode == CHASSIS_NO_FOLLOW_YAW)
//			{
//				Mode = CHASSIS_FOLLOW_YAW; // 底盘跟随云台
//			}
//		}
//		if(RC.read_key(&RC.Key.G, single, true))
//		{ // 开启小陀螺
//			if(Mode != CHASSIS_LITTLE_TOP && Mode != CHASSIS_NO_MOVE)
//			{
//				Mode = CHASSIS_LITTLE_TOP;
//			}
//			else if(Mode != CHASSIS_NO_MOVE)
//			{
//				Mode = CHASSIS_NO_FOLLOW_YAW;
//			}
//		}
//		// 视觉开关
//		if(RC.read_key(&RC.Press.R, single, &Flags.Visual_Flag))
//		{ // 能量机关开关
//			RC.read_key(&RC.Key.E, single, &Flags.Energy_Flag);
//		}
//		// UI添加
//		if(RC.read_key(&RC.Key.B, single, true))
//		{
//			UI_Send = true;
//		}
//		// 装弹开关
//		RC.read_key(&RC.Key.R, single, &Flags.Looding_Flag);
//		// 摩擦轮开关
//		if(RC.read_key(&RC.Key.Q, single, &Flags.Fric_Flag))
//		{ // 拨弹轮开关
//			RC.read_key(&RC.Press.L, even, &Flags.Shoot_Flag);
//		}
//		else
//		{
//			Flags.Shoot_Flag = false;
//		}
//		RC.read_key(&RC.Key.ctrl, even, &Flags.Shoot_Reversal_Flag);
//		// 提速开关
//		RC.read_key(&RC.Key.shift, even, &Flags.Speed_Up_Flag);
//	}
//	else
//	{
//		Velocity.Gear = 0;
//	}
//	// 遥控控制模式切换
//	if(switch_is_down(RC_Ptr->rc.s[CHANNEL_RIGHT]))
//	{
//		Mode = CHASSIS_NO_MOVE;
//	}
//#if RC_CONTRAL_MODE == 0
//	if(switch_is_mid(RC_Ptr->rc.s[CHANNEL_RIGHT]) && switch_is_down(RC_Ptr->rc.s[CHANNEL_LEFT]))
//	{
//		Mode = CHASSIS_NO_FOLLOW_YAW;
////		Mode = CHASSIS_NO_MOVE;
//	}
//	else if(switch_is_mid(RC_Ptr->rc.s[CHANNEL_RIGHT]) && switch_is_mid(RC_Ptr->rc.s[CHANNEL_LEFT]))
//	{
//		Mode = CHASSIS_FOLLOW_YAW;
////		Mode = CHASSIS_NO_MOVE;
//	}
//	if(switch_is_mid(RC_Ptr->rc.s[CHANNEL_RIGHT]) && switch_is_up(RC_Ptr->rc.s[CHANNEL_LEFT]))
//	{
//		Mode = CHASSIS_LITTLE_TOP;
////		Mode = CHASSIS_NO_MOVE;
//	}
//#elif RC_CONTRAL_MODE == 1
//	if(switch_is_mid(RC_Ptr->rc.s[CHANNEL_RIGHT]) && switch_is_down(RC_Ptr->rc.s[CHANNEL_LEFT]))
//	{
//		Mode = CHASSIS_NO_FOLLOW_YAW;
//		Flags.Fric_Flag = false;
//		Flags.Shoot_Flag = false;
//		Flags.Visual_Flag = false;
//	}
//	else if(switch_is_mid(RC_Ptr->rc.s[CHANNEL_RIGHT]) && switch_is_mid(RC_Ptr->rc.s[CHANNEL_LEFT]))
//	{
//		Mode = CHASSIS_NO_FOLLOW_YAW;
//		Flags.Fric_Flag = false;
//		Flags.Shoot_Flag = false;
//		Flags.Visual_Flag = true;
//	}
//	else if(switch_is_mid(RC_Ptr->rc.s[CHANNEL_RIGHT]) && switch_is_up(RC_Ptr->rc.s[CHANNEL_LEFT]))
//	{
//		Mode = CHASSIS_NO_FOLLOW_YAW;
//		Flags.Fric_Flag = true;
//		Flags.Shoot_Flag = true;
//		Flags.Visual_Flag = true;
//	}
//#endif
//	else if(switch_is_up(RC_Ptr->rc.s[CHANNEL_RIGHT]) && switch_is_up(RC_Ptr->rc.s[CHANNEL_LEFT]))
//	{
//		System_Reset();
//	}
//	else if(switch_is_up(RC_Ptr->rc.s[CHANNEL_RIGHT]) && switch_is_mid(RC_Ptr->rc.s[CHANNEL_LEFT]))
//	{
//		UI_Send = true;
//	}
//	Flag_Behaviour_Control();
//}

//void Chassis_Ctrl::Flag_Behaviour_Control()
//{
//	if(Guard.Return(SupercapData) == false ||Message.robo->game_robot_state.chassis_power_limit == 0)
//	{
//		Velocity.Speed_Set = Velocity.Speed_Set_m(Flags.Speed_Up_Flag, Velocity.Gear);
//	}
//	else if(Rate_Do_Execute(5) && Flags.Velocity_Clac_Flag == true)
//	{
//		Flags.Velocity_Clac_Flag = false;
//		if(Flags.Speed_Up_Flag == false)
//		{
//			Velocity.Speed_Set = PID.Calc(&Velocity_Pid, Message.SuperCapR.power, Message.robo->game_robot_state.chassis_power_limit - 10);
//		}
//		else if(Flags.Speed_Up_Flag == true)
//		{
//			Velocity.Speed_Set = PID.Calc(&Velocity_Pid, Message.SuperCapR.power, Message.robo->game_robot_state.chassis_power_limit + 40);
//		}
//		if(Velocity.Speed_Set < 0.f)
//		{
//			Velocity.Speed_Set = 0.f;
//		}
//	}
//		Velocity.Speed_Set = 0.7f*Velocity.Speed_Set+0.3f*Velocity.Speed_Set_Last;
//		Velocity.Speed_Set_Last = Velocity.Speed_Set;

//	// if(Message.robo->game_robot_state.robot_id != 0 && Message.robo->game_robot_state.remain_HP == 0)
//	// {//死亡处理
//	// 	Mode = CHASSIS_NO_MOVE;
//	// 	Flags.Speed_Up_Flag = false;
//	// 	Flags.Fric_Flag = false;
//	// 	Flags.Shoot_Flag = false;
//	// 	Flags.Visual_Flag = false;
//	// 	Flags.Energy_Flag = false;
//	// }

//	if(Mode == Last_Mode)
//	{
//		return;
//	}
//	else if(Mode != Last_Mode)
//	{
//		Last_Mode = Mode;
//	}
//}
//// 遥控器的数据处理成底盘的前进vx速度，vy速度
//void Chassis_Ctrl::RC_to_Control(fp32 *vx_set, fp32 *vy_set)
//{
//	if(IsInvalid(*vx_set) || IsInvalid(*vy_set))
//	{
//		return;
//	}
//	// 遥控器原始通道值
//	static int16_t vx_channel, vy_channel;
//	static fp32 vx_set_channel, vy_set_channel, temp_set_channel;

//	if(Flags.RC_Flag == false)
//	{
//		// 用WDAS控制
//		if(RC.read_key(&RC.Key.W, even, false) || RC.read_key(&RC.Key.S, even, false) || RC.read_key(&RC.Key.A, even, false) || RC.read_key(&RC.Key.D, even, false))
//		{
//			Flags.Velocity_Clac_Flag = true;
//			if(RC.read_key(&RC.Key.W, even, true)) // 方向可能改动
//			{
//				vx_set_channel = Velocity.Speed_Set;
//			}
//			else if(RC.read_key(&RC.Key.S, even, true))
//			{
//				vx_set_channel = -Velocity.Speed_Set;
//			}
//			else
//			{
//				vx_set_channel = 0.f;
//			}
//			if(RC.read_key(&RC.Key.A, even, true))
//			{
//				vy_set_channel = -Velocity.Speed_Set;
//			}
//			else if(RC.read_key(&RC.Key.D, even, true))
//			{
//				vy_set_channel = Velocity.Speed_Set;
//			}
//			else
//			{
//				vy_set_channel = 0.f;
//			}
//		}
//		else // 无WSAD输入则一直静止
//		{
//			vx_set_channel = 0;
//			vy_set_channel = 0;
//		}
//		vx_set_channel = Velocity.Vx_Set_Last*0.6f + vx_set_channel*0.4f;
//		vy_set_channel = Velocity.Vy_Set_Last*0.6f + vy_set_channel*0.4f;
//				
//		Velocity.Vx_Set_Last = vx_set_channel;
//		Velocity.Vy_Set_Last = vy_set_channel;
//	}
//	else
//	{
//		// 将遥控器的数据处理死区 int16_t yaw_channel,pitch_channel
//		rc_deadline_limit(RC_Ptr->rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);
//		rc_deadline_limit(RC_Ptr->rc.ch[CHASSIS_Y_CHANNEL], vy_channel, CHASSIS_RC_DEADLINE);

//		vx_set_channel = vx_channel * CHASSIS_VX_RC_SEN;
//		vy_set_channel = vy_channel * CHASSIS_VY_RC_SEN;
//	}

//	// 停止信号，不需要缓慢加速，直接减速到零
//	if(vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
//	{
//		vx_set_channel = 0.0f;
//	}

//	if(vy_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN && vy_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN)
//	{
//		vy_set_channel = 0.0f;
//	}

//	*vx_set = vx_set_channel;
//	*vy_set = -vy_set_channel;
//}

//void Chassis_Ctrl::Behaviour_Control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set)
//{
//	fp32 vw_set;

//	if(Mode == CHASSIS_NO_MOVE) // 无力状态
//	{
//		*vx_set = 0.0f;
//		*vy_set = 0.0f;
//		*angle_set = 0.0f;
//	}
//	else if(Mode == CHASSIS_FOLLOW_YAW) // 跟随云台
//	{
//		RC_to_Control(vx_set, vy_set); // 将遥控值转换为底盘设定量

//		if(chassis_relative_ECD > 30) // 最大到800
//		{
//			PID.Calc(&Follow_Gimbal_Pid, chassis_relative_ECD, 50);
//			vw_set = Follow_Gimbal_Pid.out * 0.001f;
//		}
//		else if(chassis_relative_ECD < -30)
//		{
//			PID.Calc(&Follow_Gimbal_Pid, chassis_relative_ECD, -50);
//			vw_set = Follow_Gimbal_Pid.out * 0.001f;
//		}
//		else
//		{
//			vw_set = 0;
//		}
//		*angle_set = vw_set;
//	}
//	else if(Mode == CHASSIS_NO_FOLLOW_YAW)
//	{
//		RC_to_Control(vx_set, vy_set);
//	}
//	else if(Mode == CHASSIS_LITTLE_TOP)
//	{
//		RC_to_Control(vx_set, vy_set);

//		if(Guard.Return(SupercapData) == false)
//		{
//			*angle_set = Velocity.Speed_Set_m(2, Velocity.Gear);
//		}
//		else
//		{
//			Flags.Velocity_Clac_Flag = true;
//			*angle_set = Velocity.Speed_Set;
//		}
//	}
//	if((ABS(*vx_set) >= 0.5f || ABS(*vy_set) >= 0.5f) && ABS(*angle_set) >= 0.5f)
//	{
//		//复杂状态降低速度
//		*vx_set *= 0.7f;
//		*vy_set *= 0.7f;
//		*angle_set *= 0.3f;
//	}
//}
//// 控制
//void Chassis_Ctrl::Control(void)
//{
//	// 设置速度
//	fp32 vx_set = 0.0f, vy_set = 0.0f, angle_set = 0.0f;
//	fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
//	Behaviour_Control(&vx_set, &vy_set, &angle_set);

//	// 跟随云台模式
//	if(Mode == CHASSIS_FOLLOW_YAW)
//	{
//		sin_yaw = arm_sin_f32(-chassis_relative_RAD);
//		cos_yaw = arm_cos_f32(-chassis_relative_RAD);
//		Velocity.vx_set = cos_yaw * vx_set - sin_yaw * vy_set;
//		Velocity.vy_set = sin_yaw * vx_set + cos_yaw * vy_set;
//		Velocity.wz_set = angle_set;
//	}
//	// 不跟随云台模式
//	else if(Mode == CHASSIS_NO_FOLLOW_YAW)
//	{
//		// 旋转控制底盘速度方向，保证前进方向是云台方向
//		sin_yaw = arm_sin_f32(-chassis_relative_RAD);
//		cos_yaw = arm_cos_f32(-chassis_relative_RAD);

//		if(1) // 底盘前进方向为云台正方向
//		{
//			Velocity.vx_set = cos_yaw * vx_set - sin_yaw * vy_set;
//			Velocity.vy_set = sin_yaw * vx_set + cos_yaw * vy_set;
//			Velocity.wz_set = angle_set;
//		}
//		else // 底盘前进方向为底盘正方向
//		{
//			Velocity.vx_set = vx_set;
//			Velocity.vy_set = vy_set;
//			Velocity.wz_set = angle_set;
//		}
//	}
//	else if(Mode == CHASSIS_LITTLE_TOP)
//	{
//		// 旋转控制底盘速度方向，保证前进方向是云台方向，有利于运动平稳
//		sin_yaw = arm_sin_f32(-chassis_relative_RAD);
//		cos_yaw = arm_cos_f32(-chassis_relative_RAD);
//		Velocity.vx_set = cos_yaw * vx_set - sin_yaw * vy_set;
//		Velocity.vy_set = sin_yaw * vx_set + cos_yaw * vy_set;

//		Velocity.wz_set = angle_set;
//		// 速度限幅
//		Velocity.vx_set = fp32_constrain(Velocity.vx_set, Velocity.vx_min_speed, Velocity.vx_max_speed);
//		Velocity.vy_set = fp32_constrain(Velocity.vy_set, Velocity.vy_min_speed, Velocity.vy_max_speed);
//	}
//	// 无力模式
//	else if(Mode == CHASSIS_NO_MOVE)
//	{
//		Velocity.vx_set = 0;
//		Velocity.vy_set = 0;
//		Velocity.wz_set = 0;
//	}
//}

//void Chassis_Ctrl::Vector_to_Wheel_Speed(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set)
//{

//	fp32 vx_temp = *vx_set;
//	fp32 vy_temp = *vy_set;
//	fp32 wz_temp = *wz_set;
//#ifdef useMecanum
//	// 旋转的时候， 由于云台靠前，所以是前面两轮 0 ，1 旋转的速度变慢， 后面两轮 2,3 旋转的速度变快
//	Motor[0].speed_set = vx_temp - vy_temp - MOTOR_DISTANCE_TO_CENTER * wz_temp;
//	Motor[1].speed_set = -vx_temp - vy_temp - MOTOR_DISTANCE_TO_CENTER * wz_temp;
//	Motor[2].speed_set = vx_temp + vy_temp - MOTOR_DISTANCE_TO_CENTER * wz_temp;
//	Motor[3].speed_set = -vx_temp + vy_temp - MOTOR_DISTANCE_TO_CENTER * wz_temp;
//#endif
//#ifdef useSteering
//	uint8_t i = 0;
//	fp32 wheel_speed[4], wheel_angle[4];
//	fp32 vx_mid[2], vy_mid[2];
//	fp32 vz_mid = sin45 * wz_temp;

//	vx_mid[0] = vx_temp - vz_mid;
//	vx_mid[1] = vx_temp + vz_mid;
//	vy_mid[0] = vy_temp - vz_mid;
//	vy_mid[1] = vy_temp + vz_mid;

//	wheel_speed[0] = sqrt(sq(vx_mid[0]) + sq(vy_mid[0]));
//	wheel_speed[1] = sqrt(sq(vx_mid[1]) + sq(vy_mid[0]));
//	wheel_speed[2] = sqrt(sq(vx_mid[1]) + sq(vy_mid[1]));
//	wheel_speed[3] = sqrt(sq(vx_mid[0]) + sq(vy_mid[1]));

//	wheel_angle[0] = atan2f(vy_mid[1], vx_mid[0]);
//	wheel_angle[1] = atan2f(vy_mid[0], vx_mid[0]);
//	wheel_angle[2] = atan2f(vy_mid[0], vx_mid[1]);
//	wheel_angle[3] = atan2f(vy_mid[1], vx_mid[1]);

//	for(i = 0; i < 4; i++)
//	{
//		Motor[i].speed_set = -wheel_speed[i];
//		Steering[i].data.angle_set = -wheel_angle[i] * PI_TO_ECD;
//	}
//#endif
//}
//// 底盘控制计算
//void Chassis_Ctrl::Control_loop(void)
//{
//	uint8_t i = 0;
//#ifdef useMecanum
//	Vector_to_Wheel_Speed(&Velocity.vx_set, &Velocity.vy_set, &Velocity.wz_set);
//	// 计算pid
//	for(i = 0; i < 4; i++)
//	{
//		PID_Calc(&Speed_Pid[i], Motor[i].speed, Motor[i].speed_set, 0);
//	}
//	// 赋值电流值
//	for(i = 0; i < 4; i++)
//	{
//		Motor[i].give_current = (int16_t)(Speed_Pid[i].out);
//	}
//#endif
//#ifdef useSteering
//	// 舵轮运动分解
//	Steering_Behaviour_Control(&Velocity.vx_set, &Velocity.vy_set, &Velocity.wz_set);

//	// 带圈数的角度及设定值运算
//	Steering_Round_Calc();
//	// 计算pid
//	for(i = 0; i < 4; i++)
//	{
//		PID.Calc(&Speed_Pid[i], Motor[i].speed, Motor[i].speed_set);
//		PID.Calc(&steering_Angle_Pid[i], Steering[i].angle_real, Steering[i].angle_set_real);
//		PID.Calc(&steering_Speed_Pid[i], Steering[i].chassis_motor_measure->speed_rpm, steering_Angle_Pid[i].out);
//	}
//	// 赋值电流值
//	for(i = 0; i < 4; i++)
//	{
//		Motor[i].give_current = (int16_t)(Speed_Pid[i].out);
//		Steering[i].give_current = (int16_t)(steering_Speed_Pid[i].out);
//	}
//#endif
//}

//// 规整ECD(范围±4096)
//fp32 motor_ecd_to_relative_ecd(fp32 angle, fp32 offset_ecd)
//{
//	int32_t relative_angle_change = angle - offset_ecd;
//	if(relative_angle_change > 4096)
//	{
//		relative_angle_change -= 8192;
//	}
//	else if(relative_angle_change < -4096)
//	{
//		relative_angle_change += 8192;
//	}
//	return relative_angle_change;
//}

//void rc_key_v_fresh(RC_ctrl_t *RC)
//{
//	Chassis.RC.rc_key_v_set(RC);
//}

//Chassis_Ctrl *get_chassis_ctrl_pointer(void)
//{
//	return &Chassis;
//}

//// 寄存器软件复位
//void System_Reset(void)
//{
//	SCB->AIRCR = (uint32_t)((0x5FAUL << SCB_AIRCR_VECTKEY_Pos) |
//		(SCB->AIRCR & SCB_AIRCR_PRIGROUP_Msk) |
//		SCB_AIRCR_SYSRESETREQ_Msk);
//}

///*------------------------------以下舵轮专属------------------------------*/
//#ifdef useSteering
//// 根据底盘模式控制舵轮行为
//void Chassis_Ctrl::Steering_Mode_Control(void)
//{
//	if(Mode == CHASSIS_NO_MOVE)
//	{
//		Steering_Mode = STEERING_STOP;
//	}
//	else if(Mode == CHASSIS_FOLLOW_YAW)
//	{
//		Steering_Mode = STEERING_LIMIT;
//	}
//	else if(Mode == CHASSIS_NO_FOLLOW_YAW)
//	{
//		Steering_Mode = STEERING_LIMIT_UPDATE;
//	}
//	else if(Mode == CHASSIS_LITTLE_TOP)
//	{
//		Steering_Mode = STEERING_LITTLE_TOP;
//	}
//}
//fp32 Temp_Angle_Set = 0;
//// 舵轮模式控制
//void Chassis_Ctrl::Steering_Behaviour_Control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set)
//{
//	uint8_t i = 0;
//	if(IsInvalid(*vx_set) || IsInvalid(*vy_set) || IsInvalid(*wz_set))
//	{
//		return;
//	}

//	Steering_Mode_Control();
//	// 死区限制
//	if(ABS(*vx_set) > 0.01f || ABS(*vy_set) > 0.01f || ABS(*wz_set) > 0.01f)
//	{
//		// 舵轮解算
//		Vector_to_Wheel_Speed(vx_set, vy_set, wz_set);
//	}
//	else
//	{
//		if(Steering_Mode == STEERING_NORMAL)
//		{ // 无速度输入时进入舵轮不旋转状态
//			Steering_Mode = STEERING_VECTOR_NO_FOLLOW;
//		}
//		else if(Steering_Mode == STEERING_LIMIT_UPDATE)
//		{ // 无速度输入时进入舵轮不旋转状态
//			Steering_Mode = STEERING_VECTOR_NO_FOLLOW;
//		}
//		for(i = 0; i < 4; i++)
//		{ // 立即停止
//			Motor[i].speed_set = 0;
//		}
//	}

//	switch(Steering_Mode)
//	{
//	case STEERING_STOP:
//	for(i = 0; i < 4; i++)
//	{
//		Steering[i].data.angle_set = Steering[i].data.angle_set_last;
//		Motor[i].speed_set = 0;
//	}
//	break;
//	case STEERING_FOLLOW_GIMBAL:
//	for(i = 0; i < 4; i++)
//	{
//		Steering[i].data.angle_set = chassis_relative_ECD;
//	}
//	break;
//	case STEERING_FOLLOW_CHASSIS:
//	for(i = 0; i < 4; i++)
//	{
//		Steering[i].data.angle_set = 0;
//	}
//	break;
//	case STEERING_VECTOR_NO_FOLLOW:
//	for(i = 0; i < 4; i++)
//	{
//		Steering[i].data.angle_set = Steering[i].data.angle_set_last;
//	}
//	break;
//	case STEERING_LIMIT:
//	for(i = 0; i < 4; i++)
//	{
//		if(Steering[i].data.angle_set < -2972) // 非对称非特殊角度
//		{
//			Steering[i].data.angle_set += 4096;
//			Motor[i].speed_set = -Motor[i].speed_set;
//		}
//		else if(Steering[i].data.angle_set > 1124)
//		{
//			Steering[i].data.angle_set -= 4096;
//			Motor[i].speed_set = -Motor[i].speed_set;
//		}
//	}
//	break;
//	case STEERING_LIMIT_UPDATE:
//	{
//		for(i = 0; i < 4; i++)
//		{
//			Temp_Angle_Set = motor_ecd_to_relative_ecd(Steering[i].data.angle_set,chassis_relative_ECD);
//			if(Temp_Angle_Set < -2972) // 非对称非特殊角度
//			{
//				Steering[i].data.angle_set += 4096;
//				Motor[i].speed_set = -Motor[i].speed_set;
//			}
//			else if(Temp_Angle_Set > 1124)
//			{
//				Steering[i].data.angle_set -= 4096;
//				Motor[i].speed_set = -Motor[i].speed_set;
//			}
//		}
//		break;
//	}
//	default:
//	break;
//	}
//}
//// 带圈数的完整角度计算
//void Chassis_Ctrl::Steering_Round_Calc(void)
//{
//	int16_t relative_angle_round = 0;
//	for(int i = 0; i < 4; i++)
//	{
//		// 实际角度完整计算
//		relative_angle_round = Steering[i].data.angle - Steering[i].data.angle_last;
//		if(relative_angle_round > 4096)
//		{
//			Steering[i].data.angle_round--;
//		}
//		else if(relative_angle_round < -4096)
//		{
//			Steering[i].data.angle_round++;
//		}
//		Steering[i].angle_real =
//			Steering[i].data.angle_round * 8191 + Steering[i].data.angle;
//		// 设定角度完整计算
//		relative_angle_round = Steering[i].data.angle_set - Steering[i].data.angle_set_last;
//		if(relative_angle_round > 4096)
//		{
//			Steering[i].data.angle_set_round--;
//		}
//		else if(relative_angle_round < -4096)
//		{
//			Steering[i].data.angle_set_round++;
//		}
//		Steering[i].angle_set_real =
//			Steering[i].data.angle_set_round * 8191 + Steering[i].data.angle_set;
//		// 优化旋转的范围
//		if(Steering[i].angle_set_real - Steering[i].angle_real > 4096)
//			Steering[i].data.angle_set_round--;
//		else if(Steering[i].angle_set_real - Steering[i].angle_real < -4096)
//			Steering[i].data.angle_set_round++;
//	}
//}
//#endif
