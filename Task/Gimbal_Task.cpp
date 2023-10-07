#include "Gimbal_Task.h"
#include "tasks.h"
#include "arm_math.h"

#include <vector>

Gimbal_Ctrl Gimbal;

// 云台任务
void Gimbal_Task(void *pvParameters)
{
	/* USER CODE BEGIN StartDefaultTask */
	Gimbal.Gimbal_Init();
  /* Infinite loop */
  for(;;)
  {		
		Gimbal.Behaviour_Mode();
		Gimbal.Feedback_Update();
		if(Gimbal.Mode == GIMBAL_NO_MOVE)
		{
			CAN_Cmd.SendData(&CAN_Cmd.Gimbal, 0, 0, 0);
			CAN_Cmd.SendData(&CAN_Cmd.Fric, 0, 0);
		}
		else
		{
			Gimbal.Control();
			Gimbal.Control_loop();
			CAN_Cmd.SendData(&CAN_Cmd.Gimbal, Gimbal.Yaw.give_current, Gimbal.Pitch.give_current, Gimbal.Trigger.give_current);
			CAN_Cmd.SendData(&CAN_Cmd.Fric, Gimbal.Fric1.give_current, Gimbal.Fric2.give_current);
//			CAN_Cmd.SendData(&CAN_Cmd.Gimbal, 0, 0, Gimbal.Trigger.give_current);
//			CAN_Cmd.SendData(&CAN_Cmd.Fric, Gimbal.Fric1.give_current, Gimbal.Fric2.give_current);
		}
		xQueueSend(Message_Queue, &ID_Data[GimbalData], 0);

		osDelay(GIMBAL_CONTROL_TIME);
	}
	/* USER CODE END StartDefaultTask */
}
// 初始化
void Gimbal_Ctrl::Gimbal_Init(void)
{
	uint8_t i;
	RC_Ptr = get_remote_control_point();

	Yaw.gimbal_motor_measure = CAN_Cmd.Gimbal.Get_Motor_Measure_Pointer(0);
	Pitch.gimbal_motor_measure = CAN_Cmd.Gimbal.Get_Motor_Measure_Pointer(1);
	Trigger.gimbal_motor_measure = CAN_Cmd.Gimbal.Get_Motor_Measure_Pointer(2);
	Fric1.gimbal_motor_measure = CAN_Cmd.Fric.Get_Motor_Measure_Pointer(0);
	Fric2.gimbal_motor_measure = CAN_Cmd.Fric.Get_Motor_Measure_Pointer(1);

	// 手动
	PID.Init(&Yaw.PositinPid, POSITION, YAW_POSITION_PID_KP, YAW_POSITION_PID_KI, YAW_POSITION_PID_KD, YAW_POSITION_PID_MAX_OUT, YAW_POSITION_PID_MAX_IOUT, YAW_POSITION_PID_BAND_I);
	PID.Init(&Yaw.SpeedPid, POSITION, YAW_SPEED_PID_KP, YAW_SPEED_PID_KI, YAW_SPEED_PID_KD, YAW_SPEED_PID_MAX_OUT, YAW_SPEED_PID_MAX_IOUT, YAW_SPEED_PID_BAND_I);
	PID.Init(&Pitch.PositinPid, POSITION, PITCH_POSITION_PID_KP, PITCH_POSITION_PID_KI, PITCH_POSITION_PID_KD, PITCH_POSITION_PID_MAX_OUT, PITCH_POSITION_PID_MAX_IOUT, PITCH_POSITION_PID_BAND_I);
	PID.Init(&Pitch.SpeedPid, POSITION, PITCH_SPEED_PID_KP, PITCH_SPEED_PID_KI, PITCH_SPEED_PID_KD, PITCH_SPEED_PID_MAX_OUT, PITCH_SPEED_PID_MAX_IOUT, PITCH_SPEED_PID_BAND_I);
	// 视觉
	PID.Init(&Yaw.FollowPositinPid, POSITION, YAW_FOLLOW_POSITION_PID_KP, YAW_FOLLOW_POSITION_PID_KI, YAW_FOLLOW_POSITION_PID_KD, YAW_FOLLOW_POSITION_PID_MAX_OUT, YAW_FOLLOW_POSITION_PID_MAX_IOUT, YAW_FOLLOW_POSITION_PID_BAND_I);
	PID.Init(&Yaw.FollowSpeedPid, POSITION, YAW_FOLLOW_SPEED_PID_KP, YAW_FOLLOW_SPEED_PID_KI, YAW_FOLLOW_SPEED_PID_KD, YAW_FOLLOW_SPEED_PID_MAX_OUT, YAW_FOLLOW_SPEED_PID_MAX_IOUT, YAW_FOLLOW_SPEED_PID_BAND_I);
	PID.Init(&Pitch.FollowPositinPid, POSITION, PITCH_FOLLOW_POSITION_PID_KP, PITCH_FOLLOW_POSITION_PID_KI, PITCH_FOLLOW_POSITION_PID_KD, PITCH_FOLLOW_POSITION_PID_MAX_OUT, PITCH_FOLLOW_POSITION_PID_MAX_IOUT, PITCH_FOLLOW_POSITION_PID_BAND_I);
	PID.Init(&Pitch.FollowSpeedPid, POSITION, PITCH_FOLLOW_SPEED_PID_KP, PITCH_FOLLOW_SPEED_PID_KI, PITCH_FOLLOW_SPEED_PID_KD, PITCH_FOLLOW_SPEED_PID_MAX_OUT, PITCH_FOLLOW_SPEED_PID_MAX_IOUT, PITCH_FOLLOW_SPEED_PID_BAND_I);
	// 能量机关
	PID.Init(&Yaw.EnergyPositinPid, POSITION, YAW_ENERGY_POSITION_PID_KP, YAW_ENERGY_POSITION_PID_KI, YAW_ENERGY_POSITION_PID_KD, YAW_ENERGY_POSITION_PID_MAX_OUT, YAW_ENERGY_POSITION_PID_MAX_IOUT, YAW_ENERGY_POSITION_PID_BAND_I);
	PID.Init(&Yaw.EnergySpeedPid, POSITION, YAW_ENERGY_SPEED_PID_KP, YAW_ENERGY_SPEED_PID_KI, YAW_ENERGY_SPEED_PID_KD, YAW_ENERGY_SPEED_PID_MAX_OUT, YAW_ENERGY_SPEED_PID_MAX_IOUT, YAW_ENERGY_SPEED_PID_BAND_I);
	PID.Init(&Pitch.EnergyPositinPid, POSITION, PITCH_ENERGY_POSITION_PID_KP, PITCH_ENERGY_POSITION_PID_KI, PITCH_ENERGY_POSITION_PID_KD, PITCH_ENERGY_POSITION_PID_MAX_OUT, PITCH_ENERGY_POSITION_PID_MAX_IOUT, PITCH_ENERGY_POSITION_PID_BAND_I);
	PID.Init(&Pitch.EnergySpeedPid, POSITION, PITCH_ENERGY_SPEED_PID_KP, PITCH_ENERGY_SPEED_PID_KI, PITCH_ENERGY_SPEED_PID_KD, PITCH_ENERGY_SPEED_PID_MAX_OUT, PITCH_ENERGY_SPEED_PID_MAX_IOUT, PITCH_ENERGY_SPEED_PID_BAND_I);
	// 摩擦轮
	PID.Init(&Fric1.SpeedPid, POSITION, FRIC1_SPEED_PID_KP, FRIC1_SPEED_PID_KI, FRIC1_SPEED_PID_KD, FRIC1_PID_MAX_OUT, FRIC1_PID_MAX_IOUT, FRIC1_PID_BAND_I);
	PID.Init(&Fric2.SpeedPid, POSITION, FRIC2_SPEED_PID_KP, FRIC2_SPEED_PID_KI, FRIC2_SPEED_PID_KD, FRIC2_PID_MAX_OUT, FRIC2_PID_MAX_IOUT, FRIC2_PID_BAND_I);
	// 拨弹轮
	PID.Init(&Trigger.PositinPid, POSITION, TRIGGER_ANGLE_PID_KP, TRIGGER_ANGLE_PID_KI, TRIGGER_ANGLE_PID_KD, TRIGGER_ANGLE_PID_MAX_OUT, TRIGGER_ANGLE_PID_MAX_IOUT, TRIGGER_ANGLE_PID_BAND_I);
	PID.Init(&Trigger.SpeedPid, POSITION, TRIGGER_SPEED_PID_KP, TRIGGER_SPEED_PID_KI, TRIGGER_SPEED_PID_KD, TRIGGER_SPEED_PID_MAX_OUT, TRIGGER_SPEED_PID_MAX_IOUT, TRIGGER_SPEED_PID_BAND_I); /*  */

	Flags.Autoshoot_Flag = AUTOSHOOT;

	Data.pitch_offset_ecd = GIMBAL_PITCH_OFFSET_ECD;
	Data.pitch_max_angle = PITCH_MOTOR_REVERSE * motor_relative_ECD_to_angle(GIMBAL_PITCH_MAX_ECD, GIMBAL_PITCH_OFFSET_ECD);
	Data.pitch_min_angle = PITCH_MOTOR_REVERSE * motor_relative_ECD_to_angle(GIMBAL_PITCH_MIN_ECD, GIMBAL_PITCH_OFFSET_ECD);
	Data.Trigger_offset_ecd = Trigger.gimbal_motor_measure->ecd;
	Data.Gear = 0;
	Data.Fric_Gear[0] = FRIC_GEAR_SET_1;
	Data.Fric_Gear[1] = FRIC_GEAR_SET_2;
	Data.Fric_Gear[2] = FRIC_GEAR_SET_3;
	Data.Shoot_Frequency_m[0] = TRIGGER_ONE_S_SHOOT_NUM1;
	Data.Shoot_Frequency_m[1] = TRIGGER_ONE_S_SHOOT_NUM2;
	Data.Shoot_Frequency_m[2] = TRIGGER_ONE_S_SHOOT_NUM3;
	Data.Loading_open = LOADING_OPEN_DUTY;
	Data.Loading_close = LOADING_CLOSE_DUTY;

	for(i = 0; i < 3; i++)
	{
		switch(Data.Fric_Gear[i])
		{
		case 15:
		Data.Fric_Set[i] = FRIC_SPEED_SET_15;
		break;
		case 18:
		Data.Fric_Set[i] = FRIC_SPEED_SET_18;
		break;
		case 30:
		Data.Fric_Set[i] = FRIC_SPEED_SET_30;
		break;
		default:
		Data.Fric_Set[i] = FRIC_SPEED_SET_15;
		break;
		}
	}

	Feedback_Update();
}
// 数据更新
void Gimbal_Ctrl::Feedback_Update(void)
{
	Yaw.angle = Message.Gyro.Yaw_angle;
	Yaw.speed = Message.Gyro.Yaw_speed * 0.1f;
	Pitch.angle = PITCH_MOTOR_REVERSE * motor_relative_ECD_to_angle(Pitch.gimbal_motor_measure->ecd, Data.pitch_offset_ecd);
	Pitch.speed = PITCH_MOTOR_REVERSE * Pitch.gimbal_motor_measure->speed_rpm;
	Trigger.speed = Trigger.gimbal_motor_measure->speed_rpm;
	Fric1.speed = Fric1.gimbal_motor_measure->speed_rpm;
	Fric2.speed = Fric2.gimbal_motor_measure->speed_rpm;

	Trig.new_angle = motor_relative_ECD_to_angle(Trigger.gimbal_motor_measure->ecd, Data.Trigger_offset_ecd);
	if(Trig.new_angle - Trig.last_angle > 180)
	{
		Trig.cycle--;
	}
	else if(Trig.new_angle - Trig.last_angle < -180)
	{
		Trig.cycle++;
	}
	Trig.last_angle = Trig.new_angle;
	Trigger.angle = Trig.new_angle + Trig.cycle * 360.0f;

	Data.Gear = Message.Game_State.robot_level;
	if(Flags.RC_Flag == false)
	{
	Data.Shoot_Frequency = Data.Shoot_Frequency_m[Data.Gear - 1];
	}
	if(Flags.RC_Flag == true)
	{
	Data.Shoot_Frequency = Data.Shoot_Frequency_m[2];
	}

	Statistic_Update(xTaskGetTickCount());
}
// 按键和拨杆控制
void Gimbal_Ctrl::Behaviour_Mode(void)
{
	if(switch_is_up(RC_Ptr->rc.s[CHANNEL_RIGHT]))
	{
		Flags.RC_Flag = false;
	}
	else
	{
		Flags.RC_Flag = true;
	}

	if(Flags.RC_Flag == false)
	{ // 按键控制
		if(RC.read_key(&RC.Key.C, single, true))
		{ // 无力
			Mode = GIMBAL_NO_MOVE;
		}
		if(RC.read_key(&RC.Key.X, single, true) && Mode == GIMBAL_NO_MOVE)
		{ // 启动后为人为控制
			Mode = GIMBAL_Normal;
		}
		// 视觉开关
		RC.read_key(&RC.Press.R, even, &Flags.Visual_Flag);
		// 能量机关开关
		RC.read_key(&RC.Key.E, single, &Flags.Energy_Flag);
		// 装弹开关
		RC.read_key(&RC.Key.R, single, &Flags.Loading_Flag);
		// 摩擦轮开关
		RC.read_key(&RC.Key.Q, single, &Flags.Fric_Flag);
		// 拨弹轮开关
		if(Flags.Fric_Flag == true && Flags.Autoshoot_Flag == true)
		{
			RC.read_key(&RC.Press.L, even, &Flags.Shoot_Flag);
		}
		else if(Flags.Fric_Flag == true && Flags.Autoshoot_Flag == false)
		{
			Flags.Shoot_Flag = RC.read_key(&RC.Press.L, single, true);
		}
		RC.read_key(&RC.Key.ctrl, even, &Flags.Shoot_Reversal_Flag);
	}

	// 拨杆控制
	if(switch_is_down(RC_Ptr->rc.s[CHANNEL_RIGHT]))
	{
		Mode = GIMBAL_NO_MOVE;
	}
	else if(switch_is_mid(RC_Ptr->rc.s[CHANNEL_RIGHT]))
	{
		Mode = GIMBAL_Normal;
	}
#if RC_CONTRAL_MODE == 0
	if(switch_is_mid(RC_Ptr->rc.s[CHANNEL_RIGHT]) && switch_is_down(RC_Ptr->rc.s[CHANNEL_LEFT]))
	{
		Mode = GIMBAL_Normal;
	}
	else if(switch_is_mid(RC_Ptr->rc.s[CHANNEL_RIGHT]) && switch_is_mid(RC_Ptr->rc.s[CHANNEL_LEFT]))
	{
		Mode = GIMBAL_Normal;
	}
	else if(switch_is_mid(RC_Ptr->rc.s[CHANNEL_RIGHT]) && switch_is_up(RC_Ptr->rc.s[CHANNEL_LEFT]))
	{
		Mode = GIMBAL_Normal;
	}
#elif RC_CONTRAL_MODE == 1
	if(switch_is_mid(RC_Ptr->rc.s[CHANNEL_RIGHT]) && switch_is_down(RC_Ptr->rc.s[CHANNEL_LEFT]))
	{
		Flags.Visual_Flag = false;
		Flags.Fric_Flag = false;
		Flags.Shoot_Flag = false;
		Flags.Loading_Flag = true;
	}
	else if(switch_is_mid(RC_Ptr->rc.s[CHANNEL_RIGHT]) && switch_is_mid(RC_Ptr->rc.s[CHANNEL_LEFT]))
	{
		Flags.Visual_Flag = true;
		Flags.Fric_Flag = true;
		Flags.Shoot_Flag = false;
		Flags.Loading_Flag = false;
	}
	else if(switch_is_mid(RC_Ptr->rc.s[CHANNEL_RIGHT]) && switch_is_up(RC_Ptr->rc.s[CHANNEL_LEFT]))
	{
		Flags.Visual_Flag = true;
		Flags.Fric_Flag = true;
		Flags.Shoot_Flag = true;
		Flags.Loading_Flag = false;
	}
#endif
	else if(switch_is_up(RC_Ptr->rc.s[CHANNEL_RIGHT]) && switch_is_up(RC_Ptr->rc.s[CHANNEL_LEFT]))
	{
		System_Reset();
	}
	Flag_Behaviour_Control();
}

// 标志位行为设置
void Gimbal_Ctrl::Flag_Behaviour_Control()
{
	if(Message.Game_State.robot_id != 0 && Message.Game_State.remain_HP == 0)
	{//死亡处理
		Flags.Shoot_Flag = false;
		Flags.Visual_Flag = false;
		Flags.Energy_Flag = false;
	}

	if(Flags.Fric_Flag == true && Guard.Return(ChassisData) == true && Message.Game_State.bullet_speed_limit != 0)
	{
		if(Data.Last_bullet_speed_limit != Message.Game_State.bullet_speed_limit)
		{
			switch(Message.Game_State.bullet_speed_limit)
			{
			case 15:
			Data.Fric_Set[Data.Gear - 1] = FRIC_SPEED_SET_15;
			break;
			case 18:
			Data.Fric_Set[Data.Gear - 1] = FRIC_SPEED_SET_18;
			break;
			case 30:
			Data.Fric_Set[Data.Gear - 1] = FRIC_SPEED_SET_30;
			break;
			default:
			Data.Fric_Set[Data.Gear - 1] = FRIC_SPEED_SET_15;
			break;
			}
			Data.Last_bullet_speed_limit = Message.Game_State.bullet_speed_limit;
		}
//		if(Flags.RC_Flag == false)
//			{
//				Data.FricSpeedSet = Data.Fric_Set[Data.Gear - 1];
//			}
//		if(Flags.RC_Flag == true)
//			{
//				Data.FricSpeedSet = Data.Fric_Set[1];
//			}
		Data.FricSpeedSet = Data.Fric_Set[Data.Gear - 1];
	}
	else if(Flags.Fric_Flag == true && (Guard.Return(ChassisData) == false || Message.Game_State.bullet_speed_limit == 0))
	{
		Data.FricSpeedSet = Data.Fric_Set[Data.Gear - 1];
	}
	else
	{
		Data.FricSpeedSet = 0.0f;
		Flags.Shoot_Flag = false;
	}
	Fric1.speed_set = -Data.FricSpeedSet;
	Fric2.speed_set = Data.FricSpeedSet;

	if(Flags.Shoot_Flag == true)
	{
		if(Flags.Autoshoot_Flag == true)
		{
			Trigger.speed_set = TRIGGER_MOTOR_REVERSE * Data.Shoot_Frequency * 60.0f / TRIGGER_ONCE_SHOOT_NUM * TRIGGER_REDUCTION_RATIO;
		}
		else
		{
			Trigger.angle_set += TRIGGER_REDUCTION_RATIO * TRIGGER_MOTOR_REVERSE * 360.0f / TRIGGER_ONCE_SHOOT_NUM;
		}
	}
	else
	{
		if(Flags.Autoshoot_Flag == true)
		{
			Trigger.speed_set = 0.0f;
		}
		else if(Flags.Fric_Flag == false)
		{
			Trigger.angle_set = Trigger.angle;
		}
	}

	if(Flags.Shoot_Reversal_Flag == true)
	{
		if(Flags.Autoshoot_Flag == true)
		{
			Trigger.speed_set = -TRIGGER_MOTOR_REVERSE * Data.Shoot_Frequency * 60.0f / TRIGGER_ONCE_SHOOT_NUM * TRIGGER_REDUCTION_RATIO;
		}
		else
		{
			Trigger.angle_set -= TRIGGER_REDUCTION_RATIO * TRIGGER_MOTOR_REVERSE * 360.0f / TRIGGER_ONCE_SHOOT_NUM;
		}
	}

	if(Flags.Loading_Flag == true)
	{
//		pwmWrite(PWM_IO, Data.Loading_open);
	}
	else
	{
//		pwmWrite(PWM_IO, Data.Loading_close);
	}

	if(Flags.Visual_Flag == true && Message.VisualR.Goal == true && Mode != GIMBAL_NO_MOVE)
	{
		if(Flags.Energy_Flag == true)
		{
			Mode = GIMBAL_ENERGY;
		}
		else
		{
			Mode = GIMBAL_AIM;
		}
	}
	else if(Mode != GIMBAL_NO_MOVE)
	{
//		Mode = GIMBAL_NO_MOVE;
		Mode = GIMBAL_Normal;
	}

	if(Mode == Last_Mode)
	{
		return;
	}
	else if(Mode != Last_Mode)
	{
		Yaw.angle_set = Yaw.angle;
		Pitch.angle_set = Pitch.angle;
		Trigger.angle_set = Trigger.angle;
		Last_Mode = Mode;
	}
}
// 摇杆和鼠标输入
void Gimbal_Ctrl::RC_to_Control(fp32 *yaw_set, fp32 *pitch_set)
{
	if(IsInvalid(*yaw_set) || IsInvalid(*pitch_set))
	{
		return;
	}
	// 遥控器原始通道值
	int16_t yaw_channel, pitch_channel;
	fp32 yaw_set_channel, pitch_set_channel;

	if(Flags.RC_Flag == true)
	{
		// 将遥控器的数据处理死区
		rc_deadline_limit(RC_Ptr->rc.ch[YawChannel], yaw_channel, RC_DEADLINE);
		rc_deadline_limit(RC_Ptr->rc.ch[PitchChannel], pitch_channel, RC_DEADLINE);

		yaw_set_channel = -(yaw_channel * Yaw_RC_SEN);
		pitch_set_channel = pitch_channel * Pitch_RC_SEN;
	}
	else if(Flags.RC_Flag == false)
	{
		yaw_set_channel = -(RC_Ptr->mouse.x * Yaw_Mouse_SEN);
		pitch_set_channel = -(RC_Ptr->mouse.y * Pitch_Mouse_SEN);
	}

	*yaw_set = yaw_set_channel;
	*pitch_set = pitch_set_channel;
}

// 云台控制设定
void Gimbal_Ctrl::Behaviour_Control(fp32 *yaw_set, fp32 *pitch_set)
{
	if(Mode == GIMBAL_NO_MOVE)
	{
		*yaw_set = 0;
		*pitch_set = 0;
	}
	else if(Mode == GIMBAL_Normal)
	{
		RC_to_Control(yaw_set, pitch_set);
	}
	else if(Mode == GIMBAL_AIM || Mode == GIMBAL_ENERGY)
	{
		*yaw_set = Message.VisualR.Error_X_kalman;
		*pitch_set = Message.VisualR.Error_Y_kalman;
	}
}

// 电机设定量控制
void Gimbal_Ctrl::Control(void)
{
	fp32 yaw_set;
	fp32 pitch_set;

	Behaviour_Control(&yaw_set, &pitch_set);

	if(Mode == GIMBAL_NO_MOVE)
	{
		yaw_set = 0;
		pitch_set = 0;
		Fric1.speed_set = 0.0f;
		Fric2.speed_set = 0.0f;
	}
	else if(Mode == GIMBAL_Normal)
	{
		Yaw.angle_set += yaw_set;
		Pitch.angle_set += pitch_set;
	}
	else if(Mode == GIMBAL_AIM || Mode == GIMBAL_ENERGY)
	{
		Yaw.angle_set = yaw_set;
		Pitch.angle_set = pitch_set;
	}

//	if(Trigger.gimbal_motor_measure->given_current > TRIGGER_BLOCKED_CURRENT && TRIGGER_MOTOR_REVERSE == -1)
//	{ // 拨弹轮防堵转
//		Trigger.angle_set = Trigger.angle_set - TRIGGER_MOTOR_REVERSE * TRIGGER_BLOCKED_ANGLE;
//		Trigger.speed_set = -TRIGGER_MOTOR_REVERSE * TRIGGER_BLOCKED_SPEED;
//	}
//	else if(Trigger.gimbal_motor_measure->given_current < -TRIGGER_BLOCKED_CURRENT && TRIGGER_MOTOR_REVERSE == 1)
//	{ // 拨弹轮防堵转
//		Trigger.angle_set = Trigger.angle_set - TRIGGER_MOTOR_REVERSE * TRIGGER_BLOCKED_ANGLE;
//		Trigger.speed_set = TRIGGER_MOTOR_REVERSE * TRIGGER_BLOCKED_SPEED;
//	}
	Pitch.angle_set = constrain(Pitch.angle_set, Data.pitch_min_angle, Data.pitch_max_angle);
}

// 云台控制PID运算
void Gimbal_Ctrl::Control_loop(void)
{
	fp32 YAW_out = 0;
	fp32 PITCH_out = 0;

	if(Mode == GIMBAL_Normal)
	{
		PID.Calc(&Yaw.PositinPid, Yaw.angle, Yaw.angle_set);
		PID.Calc(&Yaw.SpeedPid, Yaw.speed * 5.0f, Yaw.PositinPid.out);

		PID.Calc(&Pitch.PositinPid, Pitch.angle, Pitch.angle_set);
		PID.Calc(&Pitch.SpeedPid, Pitch.speed, Pitch.PositinPid.out);

		YAW_out = Yaw.SpeedPid.out;
		PITCH_out = Pitch.SpeedPid.out;
	}
	else if(Mode == GIMBAL_AIM)
	{
//		if(Rate_Do_Execute(1))
		{ // 以15次代码执行为周期
			PID.Calc(&Yaw.FollowPositinPid, Corres.Data.Yaw_visual_angle, Yaw.angle_set);
			PID.Calc(&Yaw.FollowSpeedPid, Yaw.speed, Yaw.FollowPositinPid.out);
//Message.VisualR.Yaw_Speed_kalman * 360.f
			PID.Calc(&Pitch.FollowPositinPid, Pitch.angle, Pitch.angle_set);
			PID.Calc(&Pitch.FollowSpeedPid, Pitch.speed, Pitch.FollowPositinPid.out);
		}

		YAW_out = Yaw.FollowSpeedPid.out;
		PITCH_out = Pitch.FollowSpeedPid.out;
	}
	else if(Mode == GIMBAL_ENERGY)
	{
		PID.Calc(&Yaw.EnergyPositinPid, Yaw.angle, Yaw.angle_set);
		PID.Calc(&Yaw.EnergySpeedPid, Yaw.speed, Yaw.EnergyPositinPid.out);

		PID.Calc(&Pitch.EnergyPositinPid, Pitch.angle, Pitch.angle_set);
		PID.Calc(&Pitch.EnergySpeedPid, Pitch.speed, Pitch.EnergyPositinPid.out);

		YAW_out = Yaw.EnergySpeedPid.out;
		PITCH_out = Pitch.EnergySpeedPid.out;
	}

	PID.Calc(&Fric1.SpeedPid, Fric1.speed, Fric1.speed_set);
	PID.Calc(&Fric2.SpeedPid, Fric2.speed, Fric2.speed_set);

	if(Flags.Autoshoot_Flag == false)
	{
		PID.Calc(&Trigger.PositinPid, Trigger.angle, Trigger.angle_set);
		PID.Calc(&Trigger.SpeedPid, Trigger.speed, Trigger.PositinPid.out);
	}
	else if(Flags.Autoshoot_Flag == true)
	{
		PID.Calc(&Trigger.SpeedPid, Trigger.speed, Trigger.speed_set);
	}

	Yaw.give_current = YAW_out;
	Pitch.give_current = PITCH_MOTOR_REVERSE * PITCH_out;
	Fric1.give_current = Fric1.SpeedPid.out;
	Fric2.give_current = Fric2.SpeedPid.out;
	Trigger.give_current = Trigger.SpeedPid.out;
}

// 规整ECD后转化成角度DEG，范围±180
fp32 Gimbal_Ctrl::motor_relative_ECD_to_angle(uint16_t angle, uint16_t offset_ecd)
{
	fp32 relative_angle;
	int32_t relative_ecd = angle - offset_ecd;
	if(relative_ecd > Half_ecd_range)
	{
		relative_ecd -= ecd_range;
	}
	else if(relative_ecd < -Half_ecd_range)
	{
		relative_ecd += ecd_range;
	}
	relative_angle = relative_ecd * ECD_TO_DEG;
	return relative_angle;
}

void rc_key_v_fresh(RC_ctrl_t *RC)
{
	Gimbal.RC.rc_key_v_set(RC);
}

Gimbal_Ctrl *get_gimbal_ctrl_pointer(void)
{
	return &Gimbal;
}
// 软件复位
void System_Reset(void)
{
	SCB->AIRCR = (uint32_t)((0x5FAUL << SCB_AIRCR_VECTKEY_Pos) |
		(SCB->AIRCR & SCB_AIRCR_PRIGROUP_Msk) |
		SCB_AIRCR_SYSRESETREQ_Msk);
}
