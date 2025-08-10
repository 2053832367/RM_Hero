#include "Robot_Task.h"
#include "tasks.h"
#include "arm_math.h"
#include <vector>

Chassis_Ctrl Chassis;
//extern bool UI_Send;
extern int8_t turn_top;

/*地盘电调id
 * 0    1
 * 2    3
 * 
 * 电调编号和电机编号对应关系：
 * 电调0对应电机0，电调1对应电机1，电调2对应电机2，电调3对应电机3
 * 
 * 电调编号和CAN_ID对应关系：
 * 电调0对应CAN_ID 0x200
 * 电调1对应CAN_ID 0x201
 * 电调2对应CAN_ID 0x202
 * 电调3对应CAN_ID 0x203
*/




void Chassis_Task(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	Chassis.Chassis_Init();
  /* Infinite loop */
  for(;;)
  {		
		Chassis.Behaviour_Mode();
		Chassis.Feedback_Update();
		

		if(Chassis.Mode == CHASSIS_NO_MOVE)
		{
			CAN_Cmd.SendData(&CAN_Cmd.Chassis, 0, 0, 0, 0);
		}
		else
		{
			Chassis.Control();
			Chassis.Control_loop();
			//CAN_Cmd.SendData(&CAN_Cmd.Chassis, 0, 0, 0, 0);
			CAN_Cmd.SendData(&CAN_Cmd.Chassis,Chassis.Motor[0].give_current, Chassis.Motor[1].give_current, Chassis.Motor[2].give_current, Chassis.Motor[3].give_current);
		}
		xQueueSend(Message_Queue, &ID_Data[ChassisData], 0);
		Chassis.Statistic_Update(xTaskGetTickCount());
    
    osDelay(CHASSIS_CONTROL_TIME_MS);
  }
  /* USER CODE END StartDefaultTask */
}

// 底盘初始化
void Chassis_Ctrl::Chassis_Init(void)
{
	RC_Ptr = get_remote_control_point();
	Mode   = CHASSIS_NO_MOVE;

	//初始化旋转PID
	PID.Init(&Follow_Gimbal_Pid, POSITION, CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT, CHASSIS_FOLLOW_GIMBAL_PID_BAND_I);
	PID.Init(&Power_buffer_Pid,  POSITION, POWER_BUFFER_PID_KP, POWER_BUFFER_PID_KI, POWER_BUFFER_PID_KD, POWER_BUFFER_PID_MAX_OUT, POWER_BUFFER_PID_MAX_IOUT, POWER_BUFFER_PID_BAND_I);
	
	/*功率环的P权重*/
	Power_Set_KP=0.40;
	//底盘电机的测量数据在CAN2_Rx_Task任务中Message.CAN2_Process((CanRxMsg *)CAN2_Rx_Data.Data_Ptr);
	for(uint8_t i = 0; i < 4; i++)
	{
		Motor[i].chassis_motor_measure = CAN_Cmd.Chassis.Get_Motor_Measure_Pointer(i);
		PID.Init(&Speed_Pid[i], POSITION, M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT, M3505_MOTOR_SPEED_PID_BAND_I);
	}
	// 最大 最小速度
	Velocity.vx_max_speed =  NORMAL_MAX_CHASSIS_SPEED_X;
	Velocity.vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;
	Velocity.vy_max_speed =  NORMAL_MAX_CHASSIS_SPEED_Y;
	Velocity.vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;
	
	Feedback_Update();
}
// 数据更新
void Chassis_Ctrl::Feedback_Update(void)
{
	uint8_t i = 0;
	Velocity.Speed = 0;
	for(i = 0; i < 4; i++)
	{
		// 更新电机速度，加速度是速度的PID微分
		Motor[i].speed  = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * Motor[i].chassis_motor_measure->speed_rpm;
		Motor[i].accel  = Speed_Pid[i].Dbuf[0] * CHASSIS_CONTROL_FREQUENCE;
		Velocity.Speed += abs(Motor[i].speed);
	}
	Velocity.Speed /= 4;
	
	Velocity.Gear = Message.robo->game_robot_state.robot_level;
  Power_Ctrl.Power_Feedback_Update();//功率模型更新函数
	
	chassis_relative_ECD = motor_ecd_to_relative_ecd(Gimbal.Yaw.gimbal_motor_measure->ecd, Gimbal_Motor_Yaw_Offset_ECD);
	chassis_relative_RAD = chassis_relative_ECD * ECD_TO_PI;

	// 更新底盘前进速度 x，平移速度y，旋转速度wz，坐标系为右手系
	Velocity.vx = (+Motor[0].speed - Motor[1].speed + Motor[2].speed - Motor[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
	Velocity.vy = (-Motor[0].speed - Motor[1].speed + Motor[2].speed + Motor[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
	Velocity.wz = (-Motor[0].speed - Motor[1].speed - Motor[2].speed - Motor[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_WZ / MOTOR_DISTANCE_TO_CENTER;
/*
做出如下解释：
公式为：
vx=-w1 -w2 +w3 +w4
vy=w1 -w2 -w3 +w4
w=w1+w2+w3+w4/(a+b)
由于该英雄机器人的轮胎标号为0231，公式是0123
代入相应位置
vx= -w1 +w2	-w3 +w4 
vy= w1 +w2	-w3 -w4 
w= w1 +w2 +w3 +w4 /(a+b)
总体异号，咦，好像有问题
*/




}
// 底盘行为状态设置
void Chassis_Ctrl::Behaviour_Mode(void)
{
	if(switch_is_up(RC_Ptr->rc.s[CHANNEL_RIGHT]))
		Flags.RC_Flag = false;
	else
		Flags.RC_Flag = true;
	// 按键控制
	if(Flags.RC_Flag == false)
	{
		if(RC.read_key(&RC.Key.C, single, true))
		{
			Mode    = CHASSIS_NO_MOVE; // 底盘保持不动
		}
		if(RC.read_key(&RC.Key.X, single, true))
		{
			if(Mode == CHASSIS_NO_MOVE || Mode == CHASSIS_FOLLOW_YAW || Mode == CHASSIS_LITTLE_TOP)
			{
				Mode = CHASSIS_NO_FOLLOW_YAW; //底盘不跟随云台
			}
			else if(Mode == CHASSIS_NO_FOLLOW_YAW)
			{
				Mode = CHASSIS_FOLLOW_YAW;//底盘跟随云台
			}
			//else if(Gimbal.Mode ==GIMBAL_LOB_SHOT)
			//{
			//	
			//	Mode = CHASSIS_FOLLOW_YAW;
			//	
			//}
		}
		if(RC.read_key(&RC.Key.F, single, true))
		{ // 开启小陀螺
			if(Mode != CHASSIS_LITTLE_TOP && Mode != CHASSIS_NO_MOVE)
			{
				Mode = CHASSIS_LITTLE_TOP;
			}
			else if(Mode != CHASSIS_NO_MOVE)
			{
				Mode = CHASSIS_FOLLOW_YAW;
			}
		}
		// UI添加
		if(RC.read_key(&RC.Key.Z,single, true))
		{
//			UI_Send = true;
		}
		if(RC.read_key(&RC.Key.shift, even, true))//加速
		{	
			Power_Ctrl.cap_state=1;
		}
		else Power_Ctrl.cap_state=0;
	}
	else
	{
		  Velocity.Gear = 0;
	}
	// 遥控控制模式切换
	if(switch_is_down(RC_Ptr->rc.s[CHANNEL_RIGHT]))
	{
		Mode = CHASSIS_NO_MOVE;
		
	}
	
//	CHASSIS_NO_MOVE = 0
//	CHASSIS_FOLLOW_YAW      //跟随云台
//	CHASSIS_NO_FOLLOW_YAW   //不跟随云台
//	CHASSIS_LITTLE_TOP      //小陀螺
#if Chassis_RC_CONTRAL_MODE == 0
	if(switch_is_mid(RC_Ptr->rc.s[CHANNEL_RIGHT]) && switch_is_down(RC_Ptr->rc.s[CHANNEL_LEFT]))
	{
		
		if(judge_type.game_robot_HP.blue_1_robot_HP == 600 || judge_type.game_robot_HP.red_1_robot_HP == 600)
		{
			/*-------------滑杆上下正反小陀螺 一坨屎-----------*/
			
			if(RC_Ptr->rc.ch[4]<=200 && RC_Ptr->rc.ch[4]>=-200)		
			Mode = CHASSIS_FOLLOW_YAW;
			else if(RC_Ptr->rc.ch[4] >=650)
			{
			Mode = CHASSIS_LITTLE_TOP;
			turn_top = 1;//CHASSIS_LITTLE_TOP
				
			}
			else if(RC_Ptr->rc.ch[4] <=-650)
			{
			Mode = CHASSIS_LITTLE_TOP;
				turn_top = -1;
				
			}
		}
		/*------------------------------------------------*/
		else 
		{
		/*-----*/
		Mode = CHASSIS_NO_FOLLOW_YAW;
			
		}
		
	}
	else if(switch_is_mid(RC_Ptr->rc.s[CHANNEL_RIGHT]) && switch_is_mid(RC_Ptr->rc.s[CHANNEL_LEFT]))
	{
	  	Mode = CHASSIS_NO_FOLLOW_YAW;
	}
	if(switch_is_mid(RC_Ptr->rc.s[CHANNEL_RIGHT]) && switch_is_up(RC_Ptr->rc.s[CHANNEL_LEFT]))
	{
		  Mode = CHASSIS_NO_FOLLOW_YAW;
	}
#elif Chassis_RC_CONTRAL_MODE == 1
	if(switch_is_mid(RC_Ptr->rc.s[CHANNEL_RIGHT]) && switch_is_down(RC_Ptr->rc.s[CHANNEL_LEFT]))
	{
		Mode = CHASSIS_NO_FOLLOW_YAW;
		Flags.Fric_Flag = false;
		Flags.Shoot_Flag = false;
		Flags.Visual_Flag = false;
	}
	else if(switch_is_mid(RC_Ptr->rc.s[CHANNEL_RIGHT]) && switch_is_mid(RC_Ptr->rc.s[CHANNEL_LEFT]))
	{
		Mode = CHASSIS_NO_FOLLOW_YAW;
		Flags.Fric_Flag = false;
		Flags.Shoot_Flag = false;
		Flags.Visual_Flag = true;
	}
	else if(switch_is_mid(RC_Ptr->rc.s[CHANNEL_RIGHT]) && switch_is_up(RC_Ptr->rc.s[CHANNEL_LEFT]))
	{
		Mode = CHASSIS_NO_FOLLOW_YAW;
		Flags.Fric_Flag = true;
		Flags.Shoot_Flag = true;
		Flags.Visual_Flag = true;
	}
#endif
	else if(switch_is_up(RC_Ptr->rc.s[CHANNEL_RIGHT]) && switch_is_up(RC_Ptr->rc.s[CHANNEL_LEFT]))
	{
		
	}
	else if(switch_is_up(RC_Ptr->rc.s[CHANNEL_RIGHT]) && switch_is_mid(RC_Ptr->rc.s[CHANNEL_LEFT]))
	{
//		UI_Send = true;
	}
	Flag_Behaviour_Control();
}

void Chassis_Ctrl::Flag_Behaviour_Control()
{
 if(Message.robo->game_robot_state.robot_id != 0 && Message.robo->game_robot_state.current_HP == 0)
 {//死亡处理
		Mode = CHASSIS_NO_MOVE;
		Flags.Speed_Up_Flag = false;
		PID.Clear(&Follow_Gimbal_Pid);
		PID.Clear(&Velocity_Pid);
		PID.Clear(&Speed_Pid[0]); 
		PID.Clear(&Speed_Pid[1]); 
		PID.Clear(&Speed_Pid[2]); 
		PID.Clear(&Speed_Pid[3]);
 }

	if(Mode == Last_Mode)
	{
		return;
	}
	else if(Mode != Last_Mode)
	{
		Last_Mode = Mode;
	}
}
// 遥控器的数据处理成底盘的前进vx速度，vy速度
void Chassis_Ctrl::RC_to_Control(fp32 *vx_set, fp32 *vy_set)
{
	if(IsInvalid(*vx_set) || IsInvalid(*vy_set))
	{
		return;
	}
	// 遥控器原始通道值
	static int16_t vx_channel, vy_channel;
	static fp32 vx_set_channel, vy_set_channel, temp_set_channel;

	if(Flags.RC_Flag == false)
	{
		// 用WDAS控制
		if(RC.read_key(&RC.Key.W, even, false) || RC.read_key(&RC.Key.S, even, false) || RC.read_key(&RC.Key.A, even, false) || RC.read_key(&RC.Key.D, even, false))
		{
				Flags.Velocity_Clac_Flag = true;
				if(RC.read_key(&RC.Key.W, even, true)) // 方向可能改动
				{
						vx_set_channel =Power_Set_KP*sqrt(Power_Ctrl.Power_limit.Chassis_Max_power);//,功率和速度的平方存线性关系
				}
				else if(RC.read_key(&RC.Key.S, even, true))
				{
						vx_set_channel =-Power_Set_KP*sqrt(Power_Ctrl.Power_limit.Chassis_Max_power);
				}
				else
				{
					  vx_set_channel = 0.f;
				}
				if(RC.read_key(&RC.Key.A, even, true))
				{
						vy_set_channel=-Power_Set_KP*sqrt(Power_Ctrl.Power_limit.Chassis_Max_power);
				}
				else if(RC.read_key(&RC.Key.D, even, true))
				{
						vy_set_channel=Power_Set_KP*sqrt(Power_Ctrl.Power_limit.Chassis_Max_power);
				}
				else
				{
					  vy_set_channel = 0.0f;
				}

		}
		else//无WSAD输入则一直静止
		{
			vx_set_channel = 0;
			vy_set_channel = 0;
		}

		vx_set_channel = Velocity.Vx_Set_Last*0.99f + vx_set_channel*0.01f;
		vy_set_channel = Velocity.Vy_Set_Last*0.99f + vy_set_channel*0.01f;
		 		
		Velocity.Vx_Set_Last = vx_set_channel;
		Velocity.Vy_Set_Last = vy_set_channel;
	}
	else
	{
//		//遥控器功率控制
//		if(RC_Ptr->rc.ch[CHASSIS_X_CHANNEL]>0)
//			vx_set_channel= Power_Set_KP*sqrt(Power_Ctrl.Power_limit.Chassis_Max_power);
//		else if(RC_Ptr->rc.ch[CHASSIS_X_CHANNEL]<0)
//			vx_set_channel=-Power_Set_KP*sqrt(Power_Ctrl.Power_limit.Chassis_Max_power);
//		else
//			vx_set_channel=0;
//		
//		if(RC_Ptr->rc.ch[CHASSIS_Y_CHANNEL]>0)
//			vy_set_channel= Power_Set_KP*sqrt(Power_Ctrl.Power_limit.Chassis_Max_power);
//		else if(RC_Ptr->rc.ch[CHASSIS_Y_CHANNEL]<0)
//			vy_set_channel=-Power_Set_KP*sqrt(Power_Ctrl.Power_limit.Chassis_Max_power);
//		else
//			vy_set_channel=0;
		
    //将遥控器的数据处理死区 int16_t yaw_channel,pitch_channel
		rc_deadline_limit(RC_Ptr->rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);
		rc_deadline_limit(RC_Ptr->rc.ch[CHASSIS_Y_CHANNEL], vy_channel, CHASSIS_RC_DEADLINE);
		vx_set_channel = vx_channel / 660.0f * Power_Set_KP*sqrt(Power_Ctrl.Power_limit.Chassis_Max_power);
		vy_set_channel = vy_channel / 660.0f * Power_Set_KP*sqrt(Power_Ctrl.Power_limit.Chassis_Max_power);
	}

	*vx_set =  vx_set_channel;
	*vy_set =  -vy_set_channel;
}
float turn_kp=0.5;//小陀螺时转速权重比
float ZC_power_xy=0.6;
void Chassis_Ctrl::Behaviour_Control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set)
{
	fp32 vw_set;
	
	if(Mode == CHASSIS_NO_MOVE) // 无力状态
	{
			*vx_set = 0.0f;
			*vy_set = 0.0f;
			*angle_set = 0.0f;
	}
	else if(Mode == CHASSIS_FOLLOW_YAW) // 跟随云台
	{
		 Power_Set_KP = ZC_power_xy;
	 	 RC_to_Control(vx_set, vy_set); // 将遥控值转换为底盘设定量
		if (chassis_relative_ECD > 50)// 最大到800
		{
			PID.Calc(&Follow_Gimbal_Pid, chassis_relative_ECD,0);
			vw_set = Follow_Gimbal_Pid.out * 0.001f;
		}
		else if (chassis_relative_ECD < -50)
		{
			PID.Calc(&Follow_Gimbal_Pid, chassis_relative_ECD, -0);
			vw_set = Follow_Gimbal_Pid.out * 0.001f;
		}else{
			vw_set = 0;
		}
		
		*angle_set = vw_set;
	}
	else if(Mode == CHASSIS_NO_FOLLOW_YAW)
	{
		RC_to_Control(vx_set, vy_set);//第一处错误，vx和xy逆置
	}
	else if(Mode == CHASSIS_LITTLE_TOP)
	{
		Power_Set_KP=(1-turn_kp)*ZC_power_xy;
		RC_to_Control(vx_set, vy_set);

  	if(RC.read_key(&RC.Key.W, even, true) || RC.read_key(&RC.Key.S, even, true) || RC.read_key(&RC.Key.A, even, true) || RC.read_key(&RC.Key.D, even, true))
			  *angle_set =turn_kp*ZC_power_xy*sqrt(Power_Ctrl.Power_limit.Chassis_Max_power)/MOTOR_DISTANCE_TO_CENTER;
	  else
			  *angle_set =ZC_power_xy*sqrt(Power_Ctrl.Power_limit.Chassis_Max_power)* turn_top/MOTOR_DISTANCE_TO_CENTER ;

	}
}
// 控制
void Chassis_Ctrl::Control(void)
{
	// 设置速度
	fp32 vx_set = 0.0f, vy_set = 0.0f, angle_set = 0.0f;
	fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
	Behaviour_Control(&vx_set, &vy_set, &angle_set);

	// 跟随云台模式
	if(Mode == CHASSIS_FOLLOW_YAW)
	{
		sin_yaw = arm_sin_f32(-chassis_relative_RAD);
		cos_yaw = arm_cos_f32(-chassis_relative_RAD);

		// 旋转矩阵坐标变换
		Velocity.vx_set = (cos_yaw * vx_set - sin_yaw * vy_set);
		Velocity.vy_set = (sin_yaw * vx_set + cos_yaw * vy_set);
		Velocity.wz_set = angle_set;
/*
上述原理解释q = R(a)p
其中q是目标坐标系(底盘)下的坐标，R(a)是旋转矩阵，p是源坐标系(云台枪管方向)下的坐标。
R(a) = 	| cos(a) -sin(a) 0 |
        | sin(a)  cos(a) 0 |
R(a)^T=R(a)^-1
矩阵的转置等于矩阵的逆
*/



	}
	// 不跟随云台模式
	else if(Mode == CHASSIS_NO_FOLLOW_YAW)
	{
		// 旋转控制底盘速度方向，保证前进方向是云台方向
		sin_yaw = arm_sin_f32(chassis_relative_RAD);
		cos_yaw = arm_cos_f32(chassis_relative_RAD);

		
		/*                    */
		if(1) // 底盘前进方向为云台正方向
		{
			Velocity.vx_set = cos_yaw * vx_set - sin_yaw * vy_set;
//			Velocity.vy_set = 0;
			Velocity.vy_set = sin_yaw * vx_set + cos_yaw * vy_set;
			Velocity.wz_set = angle_set;
		}
		else // 底盘前进方向为底盘正方向
		{
			Velocity.vx_set = vx_set;
			Velocity.vy_set = vy_set;
			Velocity.wz_set = angle_set;
		}
	}
	else if(Mode == CHASSIS_LITTLE_TOP)
	{
		// 旋转控制底盘速度方向，保证前进方向是云台方向，有利于运动平稳
		sin_yaw = arm_sin_f32(chassis_relative_RAD);
		cos_yaw = arm_cos_f32(chassis_relative_RAD);

		Velocity.vx_set = cos_yaw * vx_set - sin_yaw * vy_set;
		Velocity.vy_set = sin_yaw * vx_set + cos_yaw * vy_set;
		Velocity.wz_set = angle_set;
		// 速度限幅
		Velocity.vx_set = fp32_constrain(Velocity.vx_set, Velocity.vx_min_speed, Velocity.vx_max_speed);
		Velocity.vy_set = fp32_constrain(Velocity.vy_set, Velocity.vy_min_speed, Velocity.vy_max_speed);
	}
	// 无力模式
	else if(Mode == CHASSIS_NO_MOVE)
	{
		Velocity.vx_set = 0;
		Velocity.vy_set = 0;
		Velocity.wz_set = 0;
	}
}

void Chassis_Ctrl::Vector_to_Wheel_Speed(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set)
{

	fp32 vx_temp = *vx_set;
	fp32 vy_temp = *vy_set;
	fp32 wz_temp = *wz_set;

	// 旋转的时候， 由于云台靠前，所以是前面两轮 0 ，1 旋转的速度变慢， 后面两轮 2,3 旋转的速度变快
	Motor[0].speed_set =  vx_temp - vy_temp - MOTOR_DISTANCE_TO_CENTER * wz_temp;
	Motor[1].speed_set = -vx_temp - vy_temp - MOTOR_DISTANCE_TO_CENTER * wz_temp;
	Motor[2].speed_set =  vx_temp + vy_temp - MOTOR_DISTANCE_TO_CENTER * wz_temp;
	Motor[3].speed_set = -vx_temp + vy_temp - MOTOR_DISTANCE_TO_CENTER * wz_temp;

}
// 底盘控制计算
void Chassis_Ctrl::Control_loop(void)
{
	uint8_t i = 0;
	
		if(RC.read_key_even(&RC.Key.ctrl)){
					Velocity.wz_set = -Velocity.wz_set;
		}
	Vector_to_Wheel_Speed(&Velocity.vx_set, &Velocity.vy_set, &Velocity.wz_set);
	
	PID.Calc(&Power_buffer_Pid,Message.robo->power_heat_data.buffer_energy, Power_Ctrl.power_buffer_set);
	Power_Ctrl.Power_limit.power_buffer_out=Power_buffer_Pid.out;//更新缓冲能量输出
		
	// 计算pid
	//todo 改为功率PID
	for(i = 0; i < 4; i++)
	{
		PID.Calc(&Speed_Pid[i], Motor[i].speed, Motor[i].speed_set);
	}
	
	Power_Ctrl.Power_calc.send_current_value[0]=( Speed_Pid[0].out);
	Power_Ctrl.Power_calc.send_current_value[1]=(-Speed_Pid[1].out);
	Power_Ctrl.Power_calc.send_current_value[2]=( Speed_Pid[2].out);
	Power_Ctrl.Power_calc.send_current_value[3]=(-Speed_Pid[3].out);
	
	/*功率模型计算*/
	Power_Ctrl.Power_Calc();
	
	Motor[0].give_current = (int16_t)( Power_Ctrl.Power_calc.send_current_value[0]);
	Motor[1].give_current = (int16_t)(-Power_Ctrl.Power_calc.send_current_value[1]);
	Motor[2].give_current = (int16_t)( Power_Ctrl.Power_calc.send_current_value[2]);
	Motor[3].give_current = (int16_t)(-Power_Ctrl.Power_calc.send_current_value[3]);
	
//	Motor[0].give_current = (int16_t)Speed_Pid[0].out;
//	Motor[1].give_current = (int16_t)Speed_Pid[1].out;
//	Motor[2].give_current = (int16_t)Speed_Pid[2].out;
//	Motor[3].give_current = (int16_t)Speed_Pid[3].out;

}
// 规整ECD(范围±4096)
fp32 motor_ecd_to_relative_ecd(fp32 angle, fp32 offset_ecd)
{
	int32_t relative_angle_change = angle - offset_ecd;
	if(relative_angle_change > 4096)
	{
		relative_angle_change -= 8192;
	}
	else if(relative_angle_change < -4096)
	{
		relative_angle_change += 8192;
	}
	return relative_angle_change;
}

void rc_key_v_fresh_Chassis(RC_ctrl_t *RC)
{
	Chassis.RC.rc_key_v_set(RC);
}

// 返回指向全局 Chassis_Ctrl 实例的指针。
// 注意：Chassis 对象在线程间共享；在访问或修改其成员时请确保线程安全。
Chassis_Ctrl *get_chassis_ctrl_pointer(void)
{
	return &Chassis;
}

