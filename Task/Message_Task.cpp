#include "Message_Task.h"
#include "tasks.h"

Message_Ctrl Message;
uint8_t r[20];

void Message_Task(void *pvParameters)
{
	/* USER CODE BEGIN StartDefaultTask */
	Message.Init();
  /* Infinite loop */
  for(;;)
  {		
    if(xQueueReceive(Message_Queue, &ID_Data[MessageData], portMAX_DELAY))
		{
			Guard.Feed(ID_Data[MessageData].Data_ID);
			Guard.Feed(MessageData);
			Message.Statistic_Update(xTaskGetTickCount());
		}
  }
  /* USER CODE END StartDefaultTask */
}

void CAN1_Rx_Task(void *pvParameters)
{
	/* USER CODE BEGIN StartDefaultTask */
	static ID_Data_t CAN1_Rx_Data;
  /* Infinite loop */
  for(;;)
  {		
		if(xQueueReceive(CAN1_Rx_Queue, &CAN1_Rx_Data, portMAX_DELAY))
		{
			Message.CAN1_Process((CanRxMsg *)CAN1_Rx_Data.Data_Ptr);
			Guard.Feed(CanData1);
		}
  }
  /* USER CODE END StartDefaultTask */
}

void CAN2_Rx_Task(void *pvParameters)
{
	/* USER CODE BEGIN StartDefaultTask */
	static ID_Data_t CAN2_Rx_Data;
  /* Infinite loop */
  for(;;)
  {		
    if(xQueueReceive(CAN2_Rx_Queue, &CAN2_Rx_Data, portMAX_DELAY))
		{
			Message.CAN2_Process((CanRxMsg *)CAN2_Rx_Data.Data_Ptr);
			Guard.Feed(CanData2);
		}
  }
  /* USER CODE END StartDefaultTask */
}

void CAN3_Rx_Task(void *pvParameters)
{
	/* USER CODE BEGIN StartDefaultTask */
	static ID_Data_t CAN3_Rx_Data;
  /* Infinite loop */
  for(;;)
  {		
    if(xQueueReceive(CAN3_Rx_Queue, &CAN3_Rx_Data, portMAX_DELAY))
		{
			Message.CAN3_Process((CanRxMsg *)CAN3_Rx_Data.Data_Ptr);
			Guard.Feed(CanData3);
		}
  }
  /* USER CODE END StartDefaultTask */
}

void Serial_Rx_Task(void *pvParameters)
{
	/* USER CODE BEGIN StartDefaultTask */
	static ID_Data_t Serial_Rx_Data;
  /* Infinite loop */
  for(;;)
  {		
    if(xQueueReceive(Serial_Rx_Queue, &Serial_Rx_Data, portMAX_DELAY))
		{
			switch(Serial_Rx_Data.Data_ID)
			{
			case SerialData1:
			Message.Serialx_Hook((uint8_t *)Serial_Rx_Data.Data_Ptr, &Serial1_Ctrl);
			break;
			case SerialData3:
			Message.Serialx_Hook((uint8_t *)Serial_Rx_Data.Data_Ptr, &Serial3_Ctrl);
			break;
			case SerialData4:
			Message.Serialx_Hook((uint8_t *)Serial_Rx_Data.Data_Ptr, &Serial4_Ctrl);
			break;
			case SerialData7:
			Message.Serialx_Hook((uint8_t *)Serial_Rx_Data.Data_Ptr, &Serial7_Ctrl);
			break;
			case SerialData8:
			Message.Serialx_Hook((uint8_t *)Serial_Rx_Data.Data_Ptr, &Serial8_Ctrl);
			break;
			default:
			break;
			}
			Guard.Feed(Serial_Rx_Data.Data_ID);
		}
  }
  /* USER CODE END StartDefaultTask */
}

void DR16_Rx_Task(void *pvParameters)
{
	/* USER CODE BEGIN StartDefaultTask */
	static ID_Data_t DR16_Rx_Data;	
	//remote control data 
	//遥控器控制变量
	
  /* Infinite loop */
  for(;;)
  {		
		if(xQueueReceive(DR16_Rx_Queue, &DR16_Rx_Data.Data_Ptr, portMAX_DELAY))
		{
			sbus_to_rc(&( ((uint8_t *)(DR16_Rx_Data.Data_Ptr))[1]) ,Message.RC_Ptr);
			rc_key_v_fresh(Message.RC_Ptr);
			Guard.Feed(RCData);
		}
  }
  /* USER CODE END StartDefaultTask */
}

void Message_Ctrl::Init()
{	
	Message.RC_Ptr = &RC_ctrl;
	CAN_ALL_Init();
	Prefence_Init();
	Serial_ALL_Init();
}

void Message_Ctrl::Serialx_Hook(uint8_t *Rx_Message, Serialctrl *Serialx_Ctrl)
{
	if(Serialx_Ctrl == &VISUAL_SERIAL)
	{
		Visual_Serial_Hook(Rx_Message);
	}
	if(Serialx_Ctrl == &CHASSIS_SERIAL)
	{
		Chassis_Serial_Hook(Rx_Message);
	}
	if(Serialx_Ctrl == &GYRO_SERIAL)
	{
		Gyro_Serial_Hook(Rx_Message);
	}
	if(Serialx_Ctrl == &DR16_SERIAL)
	{
		xQueueSend(DR16_Rx_Queue, &Rx_Message, 0);
	}
}

void Message_Ctrl::CAN1_Process(CanRxMsg *Rx_Message)
{
	CanRxMsg Rx_Data;
	for(uint8_t x=0;x<4;x++)
	{
					Rx_Data.StdId.u8[x]=CAN1_Ctrl.read();
	}
	for(uint8_t x=0;x<8;x++)
	{
					Rx_Data.Data[x]=CAN1_Ctrl.read();
	}
	switch(Rx_Data.StdId.u32)
	{
	case CAN_DJI_Motor5_ID:
	case CAN_DJI_Motor6_ID:
	case CAN_DJI_Motor7_ID:
	{
		static uint8_t i = 0;
		//处理电机ID号
		i = Rx_Data.StdId.u32 - CAN_DJI_Motor5_ID;
		//处理电机数据宏函数
		MA_get_motor_measure(CAN_Cmd.Gimbal.GetData(i), Rx_Data.Data);
		break;
	}
	default:
	{
		break;
	}
	}
}

void Message_Ctrl::CAN2_Process(CanRxMsg *Rx_Message)
{
	CanRxMsg Rx_Data;
	for(uint8_t x=0;x<4;x++)
	{
					Rx_Data.StdId.u8[x]=CAN2_Ctrl.read();
	}
	for(uint8_t x=0;x<8;x++)
	{
					Rx_Data.Data[x]=CAN2_Ctrl.read();
	}
	switch(Rx_Data.StdId.u32)
	{
	case CAN_DJI_Motor1_ID:
	case CAN_DJI_Motor2_ID:
	{
		static uint8_t i = 0;
		//处理电机ID号
		i = Rx_Data.StdId.u32 - CAN_DJI_Motor1_ID;
		//处理电机数据宏函数
		MA_get_motor_measure(CAN_Cmd.Fric.GetData(i), Rx_Data.Data);
		break;
	}
	default:
	{
		Gyro_CAN_Hook(&Rx_Data.StdId.u32 , Rx_Data.Data);
		break;
	}
	}
}

void Message_Ctrl::CAN3_Process(CanRxMsg *Rx_Message)
{
	CanRxMsg Rx_Data;
	for(uint8_t x=0;x<4;x++)
	{
					Rx_Data.StdId.u8[x]=CAN3_Ctrl.read();
	}
	for(uint8_t x=0;x<8;x++)
	{
					Rx_Data.Data[x]=CAN3_Ctrl.read();
	}
	switch(Rx_Data.StdId.u32)
	{
	case CAN_DJI_Motor1_ID:
	case CAN_DJI_Motor2_ID:
	{
//		static uint8_t i = 0;
//		//处理电机ID号
//		i = Rx_Data.StdId.u32 - CAN_DJI_Motor1_ID;
//		//处理电机数据宏函数
//		MA_get_motor_measure(CAN_Cmd.Fric.GetData(i), Rx_Data.Data);
//		break;
//	}
//	default:
//	{
//		Gyro_CAN_Hook(&Rx_Data.StdId.u32 , Rx_Data.Data);
//		break;
	}
	}
}

void Message_Ctrl::Visual_Serial_Hook(uint8_t *Rx_Message)
{
//	uint8_t i;
//	uint8_t len = Rx_Message[0];
//	VisualR.Fps.Statistic_Update(xTaskGetTickCount());
//	if(Rx_Message[2] == 0)
//	{
//		VisualR.State = false;
//	}
//	else
//	{
//		VisualR.State = true;

//		for(i = 0;i < 4;i++)
//		{
//			Error_X.I[i] = Rx_Message[i + 3];
//			Error_Y.I[i] = Rx_Message[i + 7];
//		}
////		VisualR.Error_X = -Error_X.F + Gimbal.Yaw.angle;
////		VisualR.Error_Y = -Error_Y.F + Gimbal.Pitch.angle;
//		VisualR.Error_X = Error_X.F;
//		VisualR.Error_Y = Error_Y.F;
//	}
//	if(VisualR.State)
//	{
//		VisualR.Goal = true;
//	}
//	if(VisualR.target == 0)
//	{
//		VisualR.Goal = false;
//		VisualR.Error_X = Gimbal.Yaw.angle;
//		VisualR.Error_Y = Gimbal.Pitch.angle;
//	}

//	if(VisualR.target > 0)
//	{
//		VisualR.Yaw_Speed = Target_Speed_Calc(&Visual_Yaw_Speed, xTaskGetTickCount(), VisualR.Error_X);
//		VisualR.Pitch_Speed = Target_Speed_Calc(&Visual_Pitch_Speed, xTaskGetTickCount(), VisualR.Error_Y);
//	}

for(int i = 0;i < Rx_Message[0]+1;i++)
	{
		r[i]=Rx_Message[i];
	}
}

void Message_Ctrl::Chassis_Serial_Hook(uint8_t *Rx_Message)
{
//	uint8_t i;
//	uint8_t len = Rx_Message[0];

//	if(Verify_CRC8_Check_Sum(&Rx_Message[1], Rx_Message[0]))
//	{
//		if(Rx_Message[2] == game_robot_state_id_)
//		{
//			memcpy(&Game_State, &Rx_Message[1], sizeof(game_robot_state_t));

//			uint8_t Bullet_Speed_Set;
//			if(Game_State.bullet_speed != bullet_speed_last)
//			{
//				bullet_speed_last = Game_State.bullet_speed;
//				if(Message.Game_State.bullet_speed_limit != 0)
//				{
//					Bullet_Speed_Set = Message.Game_State.bullet_speed_limit;
//				}
//				else
//				{
//					Bullet_Speed_Set = Gimbal.Data.Fric_Gear[Gimbal.Data.Gear - 1];
//				}
//				//待改成pid,无固定周期,没想法
//				if(Game_State.bullet_speed - Bullet_Speed_Set > -(Bullet_Speed_Set*0.05f))
//				{
//					Gimbal.Data.Fric_Set[Gimbal.Data.Gear - 1] -= 40;
//				}
//				else if(Game_State.bullet_speed - Bullet_Speed_Set > -(Bullet_Speed_Set*0.08f))
//				{
//					Gimbal.Data.Fric_Set[Gimbal.Data.Gear - 1] -= 20;
//				}
//				else if(Game_State.bullet_speed - Bullet_Speed_Set < -(Bullet_Speed_Set*0.15f))
//				{
//					Gimbal.Data.Fric_Set[Gimbal.Data.Gear - 1] += 80;
//				}
//				else if(Game_State.bullet_speed - Bullet_Speed_Set < -(Bullet_Speed_Set*0.08f))
//				{
//					Gimbal.Data.Fric_Set[Gimbal.Data.Gear - 1] += 20;
//				}
//			}
//		}
////		else if(Rx_Message[2] == game_robot_HP_id_)
////		{
////			memcpy(&Message.Game_HP, &Rx_Message[1], sizeof(ext_game_robot_HP_t));
////		}
//		else if(Rx_Message[2] == game_status_id_)
//		{
//			memcpy(&Message.game_status, &Rx_Message[1], sizeof(game_status_t));
//		}
//		else if(Rx_Message[2] == chassis_data_id)
//		{
//			memcpy(&Message.ChassisR, &Rx_Message[1], sizeof(Chassis_Receive_Data_t));
//		}
//	}
	for(int i = 0;i < Rx_Message[0]+1;i++)
	{
		r[i]=Rx_Message[i];
	}
}

void Message_Ctrl::Gyro_Serial_Hook(uint8_t *Rx_Message)
{
	int16_t temp = 0;
	uint8_t len = Rx_Message[0];

	if(len == GYRO_SERIAL_Data_Lenth1)
	{
	switch(Rx_Message[2])
	{
	case 0x52:
	{
		memcpy(&temp, &Rx_Message[7], 2);
		Gyro.data.SpeedZ = temp / 32768.0f * 2000.0f;
		Gyro.Gyro_Speed_fps.Statistic_Update(xTaskGetTickCount());
		break;
	}
	case 0x53:
	{
		memcpy(&temp, &Rx_Message[7], 2);
		Gyro.data.AngleY = temp / 32768.0f * 180.0f;
		Gyro.Gyro_Angle_fps.Statistic_Update(xTaskGetTickCount());
		Gyro.time[1] = xTaskGetTickCount();
		break;
	}
	default:
	break;
	}
	}
	else if(len == GYRO_SERIAL_Data_Lenth2)
	{
	switch(Rx_Message[2])
	{
	case 0x52:
	{
		memcpy(&temp, &Rx_Message[7], 2);
		Gyro.data.SpeedZ = temp / 32768.0f * 2000.0f;
		Gyro.Gyro_Speed_fps.Statistic_Update(xTaskGetTickCount());
		
		memcpy(&temp, &Rx_Message[18], 2);
		Gyro.data.AngleY = temp / 32768.0f * 180.0f;
		Gyro.Gyro_Angle_fps.Statistic_Update(xTaskGetTickCount());
		Gyro.time[1] = xTaskGetTickCount();

		break;
	}
	case 0x53:
	{
		memcpy(&temp, &Rx_Message[7], 2);
		Gyro.data.AngleY = temp / 32768.0f * 180.0f;
		Gyro.Gyro_Angle_fps.Statistic_Update(xTaskGetTickCount());
		Gyro.time[1] = xTaskGetTickCount();
		
		memcpy(&temp, &Rx_Message[18], 2);
		Gyro.data.SpeedZ = temp / 32768.0f * 2000.0f;
		Gyro.Gyro_Speed_fps.Statistic_Update(xTaskGetTickCount());

		break;
	}
	default:
	break;
	}
	}

	if(Gyro.time[1] == Gyro.last_time[1])
	{
		return;
	}

	Gyro.Error_angle = Gyro.data.AngleY - Gyro.Last_angle;
	if(Gyro.Error_angle > 270.0f) Gyro.Yaw_cycle--;
	if(Gyro.Error_angle < -270.0f) Gyro.Yaw_cycle++;
	Gyro.Yaw_angle = -Gyro.data.AngleY - Gyro.Yaw_cycle * 360.0f;
	Gyro.Yaw_speed = -Gyro.data.SpeedZ;

	Gyro.last_time[1] = Gyro.time[1];
	Gyro.Last_angle = Gyro.data.AngleY;
}

void Message_Ctrl::Gyro_CAN_Hook(uint32_t *Rx_Message ,uint8_t *Rx_Data)
{
	uint8_t i;
	switch(*Rx_Message)
	{
	case 0x514:
	{
		Gyro.data.AccX = (int16_t)((Rx_Data[1] << 8) + Rx_Data[0]);
		Gyro.data.AccY = (int16_t)((Rx_Data[3] << 8) + Rx_Data[2]);
		Gyro.data.AccZ = (int16_t)((Rx_Data[5] << 8) + Rx_Data[4]);
		Gyro.Gyro_Acc_fps.Statistic_Update(xTaskGetTickCount());
		Gyro.time[0] = xTaskGetTickCount();
		break;
	}
	case 0x515:
	{
		Gyro.data.SpeedX = (int16_t)((Rx_Data[1] << 8) + Rx_Data[0]);
		Gyro.data.SpeedY = (int16_t)((Rx_Data[3] << 8) + Rx_Data[2]);
		Gyro.data.SpeedZ = (int16_t)((Rx_Data[5] << 8) + Rx_Data[4]);
		Gyro.Gyro_Speed_fps.Statistic_Update(xTaskGetTickCount());
		Gyro.time[1] = xTaskGetTickCount();
		break;
	}
	case 0x516:
	{
		Gyro.data.MagX = (int16_t)((Rx_Data[1] << 8) + Rx_Data[0]);
		Gyro.data.MagY = (int16_t)((Rx_Data[3] << 8) + Rx_Data[2]);
		Gyro.data.MagZ = (int16_t)((Rx_Data[5] << 8) + Rx_Data[4]);
		Gyro.Gyro_Mag_fps.Statistic_Update(xTaskGetTickCount());
		Gyro.time[2] = xTaskGetTickCount();
		break;
	}
	case 0x517:
	{
		Gyro.data.AngleP = (int16_t)((Rx_Data[1] << 8) + Rx_Data[0]);
		Gyro.data.AngleR = (int16_t)((Rx_Data[3] << 8) + Rx_Data[2]);
		Gyro.data.AngleY = (int16_t)((Rx_Data[5] << 8) + Rx_Data[4]);
		Gyro.Gyro_Angle_fps.Statistic_Update(xTaskGetTickCount());
		Gyro.time[3] = xTaskGetTickCount();
		break;
	}
	default:
	return;
	}
	if(Gyro.time[1] == Gyro.last_time[1])
	{
		return;
	}

	for(i = 0;i < 4;i++)
	{
		Gyro.differ_time[i] = Gyro.time[i] - Gyro.last_time[i];
	}

	if(ABS(Gyro.data.SpeedZ) < 8)
	{
		Gyro.data.SpeedZ = 0;
	}

	//	Gyro.angle.AngleP_Speed += Gyro.data.SpeedX * Gyro.differ_time[1] * 0.001f * 0.1f;
	//	Gyro.angle.AngleR_Speed += Gyro.data.SpeedY * Gyro.differ_time[1] * 0.001f * 0.1f;
	Gyro.angle.AngleY_Speed += Gyro.data.SpeedZ * (fp32)Gyro.differ_time[1] * 0.001f * 0.1f;

	//	Gyro.angle.AngleP_Acc = 180 * atanf(Gyro.data.AccX * 0.001f / sqrtf(sq(Gyro.data.AccY * 0.001f) + sq(Gyro.data.AccZ * 0.001f))) / PI;
	//	Gyro.angle.AngleR_Acc = 180 * atanf(Gyro.data.AccY * 0.001f / sqrtf(sq(Gyro.data.AccX * 0.001f) + sq(Gyro.data.AccZ * 0.001f))) / PI;

	//	Gyro.Yaw_angle = Gyro.angle.AngleY_Acc * 0.3f + Gyro.angle.AngleY_Speed * 0.7f;
	// 	Kalman_Yaw = kalman_filter_calc(&Gyro_Yaw_Temp,Gyro.data.SpeedZ* 0.1f,Gyro.data.AccZ*Gyro.differ_time* 0.001f);
	// 	Gyro.Yaw_angle = Kalman_Yaw[0];

	Gyro.Yaw_speed = Gyro.data.SpeedZ;
	// Gyro.Yaw_angle = Gyro.angle.AngleY_Speed;
	//	Gyro.Yaw_angle = Gyro.angle.AngleY_Calc;

	Gyro.Pitch_speed = Gyro.data.SpeedX;
	Gyro.Yaw_angle = Gyro.angle.AngleY_Speed;
	for(i = 0;i < 4;i++)
	{
		Gyro.last_time[i] = Gyro.time[i];
	}
}

RC_ctrl_t *get_remote_control_point(void)
{
    return &RC_ctrl;
}

Message_Ctrl *get_message_ctrl_pointer(void)
{
	return &Message;
}

void Message_Ctrl::MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))
	{
		return;
	}

	Gyro.calc.recipNorm = invSqrt(ax * ax + ay * ay + az * az);
	ax *= Gyro.calc.recipNorm;
	ay *= Gyro.calc.recipNorm;
	az *= Gyro.calc.recipNorm;

	// Normalise magnetometer measurement
	// 将磁力计得到的实际磁场向量m单位化
	Gyro.calc.recipNorm = invSqrt(mx * mx + my * my + mz * mz);
	mx *= Gyro.calc.recipNorm;
	my *= Gyro.calc.recipNorm;
	mz *= Gyro.calc.recipNorm;

	// Auxiliary variables to avoid repeated arithmetic
	// 辅助变量，以避免重复运算
	Gyro.calc.q0q0 = Gyro.calc.q0 * Gyro.calc.q0;
	Gyro.calc.q0q1 = Gyro.calc.q0 * Gyro.calc.q1;
	Gyro.calc.q0q2 = Gyro.calc.q0 * Gyro.calc.q2;
	Gyro.calc.q0q3 = Gyro.calc.q0 * Gyro.calc.q3;
	Gyro.calc.q1q1 = Gyro.calc.q1 * Gyro.calc.q1;
	Gyro.calc.q1q2 = Gyro.calc.q1 * Gyro.calc.q2;
	Gyro.calc.q1q3 = Gyro.calc.q1 * Gyro.calc.q3;
	Gyro.calc.q2q2 = Gyro.calc.q2 * Gyro.calc.q2;
	Gyro.calc.q2q3 = Gyro.calc.q2 * Gyro.calc.q3;
	Gyro.calc.q3q3 = Gyro.calc.q3 * Gyro.calc.q3;

	// Reference direction of Earth's magnetic field
	// 通过磁力计测量值与坐标转换矩阵得到大地坐标系下的理论地磁向量
	Gyro.calc.hx = 2.0f * (mx * (0.5f - Gyro.calc.q2q2 - Gyro.calc.q3q3) + my * (Gyro.calc.q1q2 - Gyro.calc.q0q3) + mz * (Gyro.calc.q1q3 + Gyro.calc.q0q2));
	Gyro.calc.hy = 2.0f * (mx * (Gyro.calc.q1q2 + Gyro.calc.q0q3) + my * (0.5f - Gyro.calc.q1q1 - Gyro.calc.q3q3) + mz * (Gyro.calc.q2q3 - Gyro.calc.q0q1));
	Gyro.calc.hz = 2.0f * (mx * (Gyro.calc.q1q3 - Gyro.calc.q0q2) + my * (Gyro.calc.q2q3 + Gyro.calc.q0q1) + mz * (0.5f - Gyro.calc.q1q1 - Gyro.calc.q2q2));
	Gyro.calc.bx = sqrt(Gyro.calc.hx * Gyro.calc.hx + Gyro.calc.hy * Gyro.calc.hy);
	Gyro.calc.bz = 2.0f * (mx * (Gyro.calc.q1q3 - Gyro.calc.q0q2) + my * (Gyro.calc.q2q3 + Gyro.calc.q0q1) + mz * (0.5f - Gyro.calc.q1q1 - Gyro.calc.q2q2));

	// Estimated direction of gravity and magnetic field
	// 将理论重力加速度向量与理论地磁向量变换至机体坐标系
	Gyro.calc.halfvx = Gyro.calc.q1q3 - Gyro.calc.q0q2;
	Gyro.calc.halfvy = Gyro.calc.q0q1 + Gyro.calc.q2q3;
	Gyro.calc.halfvz = Gyro.calc.q0q0 - 0.5f + Gyro.calc.q3q3;
	Gyro.calc.halfwx = Gyro.calc.bx * (0.5f - Gyro.calc.q2q2 - Gyro.calc.q3q3) + Gyro.calc.bz * (Gyro.calc.q1q3 - Gyro.calc.q0q2);
	Gyro.calc.halfwy = Gyro.calc.bx * (Gyro.calc.q1q2 - Gyro.calc.q0q3) + Gyro.calc.bz * (Gyro.calc.q0q1 + Gyro.calc.q2q3);
	Gyro.calc.halfwz = Gyro.calc.bx * (Gyro.calc.q0q2 + Gyro.calc.q1q3) + Gyro.calc.bz * (0.5f - Gyro.calc.q1q1 - Gyro.calc.q2q2);

	// Error is sum of cross product between estimated direction and measured direction of field vectors
	// 通过向量外积得到重力加速度向量和地磁向量的实际值与测量值之间误差
	Gyro.calc.halfex = (ay * Gyro.calc.halfvz - az * Gyro.calc.halfvy) + (my * Gyro.calc.halfwz - mz * Gyro.calc.halfwy);
	Gyro.calc.halfey = (az * Gyro.calc.halfvx - ax * Gyro.calc.halfvz) + (mz * Gyro.calc.halfwx - mx * Gyro.calc.halfwz);
	Gyro.calc.halfez = (ax * Gyro.calc.halfvy - ay * Gyro.calc.halfvx) + (mx * Gyro.calc.halfwy - my * Gyro.calc.halfwx);

	// Compute and apply integral feedback if enabled
	// 在PI补偿器中积分项使能情况下计算并应用积分项
	if(Gyro.calc.twoKi > 0.0f)
	{
		// integral error scaled by Ki
		// 积分过程
		Gyro.calc.integralFBx += Gyro.calc.twoKi * Gyro.calc.halfex * (1.0f / sampleFreq);
		Gyro.calc.integralFBy += Gyro.calc.twoKi * Gyro.calc.halfey * (1.0f / sampleFreq);
		Gyro.calc.integralFBz += Gyro.calc.twoKi * Gyro.calc.halfez * (1.0f / sampleFreq);

		// apply integral feedback
		// 应用误差补偿中的积分项
		gx += Gyro.calc.integralFBx;
		gy += Gyro.calc.integralFBy;
		gz += Gyro.calc.integralFBz;
	}
	else
	{
		// prevent integral windup
		// 避免为负值的Ki时积分异常饱和
		Gyro.calc.integralFBx = 0.0f;
		Gyro.calc.integralFBy = 0.0f;
		Gyro.calc.integralFBz = 0.0f;
	}

	// Apply proportional feedback
	// 应用误差补偿中的比例项
	gx += Gyro.calc.twoKp * Gyro.calc.halfex;
	gy += Gyro.calc.twoKp * Gyro.calc.halfey;
	gz += Gyro.calc.twoKp * Gyro.calc.halfez;

	// Integrate rate of change of quaternion
	// 微分方程迭代求解
	gx *= (0.5f * (1.0f / sampleFreq));		// pre-multiply common factors
	gy *= (0.5f * (1.0f / sampleFreq));
	gz *= (0.5f * (1.0f / sampleFreq));
	Gyro.calc.qa = Gyro.calc.q0;
	Gyro.calc.qb = Gyro.calc.q1;
	Gyro.calc.qc = Gyro.calc.q2;
	Gyro.calc.q0 += (-Gyro.calc.qb * gx - Gyro.calc.qc * gy - Gyro.calc.q3 * gz);
	Gyro.calc.q1 += (Gyro.calc.qa * gx + Gyro.calc.qc * gz - Gyro.calc.q3 * gy);
	Gyro.calc.q2 += (Gyro.calc.qa * gy - Gyro.calc.qb * gz + Gyro.calc.q3 * gx);
	Gyro.calc.q3 += (Gyro.calc.qa * gz + Gyro.calc.qb * gy - Gyro.calc.qc * gx);

	// Normalise quaternion
	// 单位化四元数 保证四元数在迭代过程中保持单位性质
	Gyro.calc.recipNorm = invSqrt(Gyro.calc.q0 * Gyro.calc.q0 + Gyro.calc.q1 * Gyro.calc.q1 + Gyro.calc.q2 * Gyro.calc.q2 + Gyro.calc.q3 * Gyro.calc.q3);
	Gyro.calc.q0 *= Gyro.calc.recipNorm;
	Gyro.calc.q1 *= Gyro.calc.recipNorm;
	Gyro.calc.q2 *= Gyro.calc.recipNorm;
	Gyro.calc.q3 *= Gyro.calc.recipNorm;
}

void Message_Ctrl::ToEulerAngles(fp32 q0, fp32 q1, fp32 q2, fp32 q3)
{
	Gyro.angle.AngleR_Calc = atan2f(2 * (q2 * q3 + q0 * q1), sq(q0) - sq(q1) - sq(q2) + sq(q3));
	Gyro.angle.AngleP_Calc = asinf(-2 * (q1 * q3 - q0 * q2));
	Gyro.angle.AngleY_Calc = atan2f(2 * (q1 * q2 + q0 * q3), sq(q0) + sq(q1) - sq(q2) - sq(q3));

	Gyro.angle.AngleR_Calc = degrees(Gyro.angle.AngleR_Calc);
	Gyro.angle.AngleP_Calc = degrees(Gyro.angle.AngleP_Calc);
	Gyro.angle.AngleY_Calc = degrees(Gyro.angle.AngleY_Calc);
}

float Message_Ctrl::Target_Speed_Calc(speed_calc_data_t *S, uint32_t time, float position)
{
	S->delay_cnt++;

	if(time != S->last_time)
	{
		S->speed = (position - S->last_position) / (time - S->last_time) * 2; //计算速度

		S->processed_speed = S->speed;

		S->last_time = time;
		S->last_position = position;
		S->last_speed = S->speed;
		S->delay_cnt = 0;
	}

	if(S->delay_cnt > 300 /*100*/) // delay 200ms speed = 0
	{
		S->processed_speed = 0; //时间过长则认为速度不变
	}

	return S->processed_speed; //计算出的速度
}

//统计按键 按下次数：eg:  按下-松开  按下-松开  2次
//key_num==1代表有键盘按下
//key_num==0代表键盘松开
void rc_key_c::sum_key_count(int16_t key_num, count_num_key *temp_count)
{
	if(key_num == 1 && temp_count->key_flag == 0)
	{
		temp_count->key_flag = 1;
	}
	if(temp_count->key_flag == 1 && key_num == 0)
	{
		temp_count->count++;
		temp_count->key_flag = 0;
	}
}

void rc_key_c::clear_key_count(count_num_key *temp_count)
{
	temp_count->count = 0;
	temp_count->key_flag = 0;
}
//按键单点赋值
bool rc_key_c::read_key_single(count_num_key *temp_count, bool *temp_bool)
{
	if((temp_count->count >= 1) && *temp_bool == 0)
	{
		temp_count->count = 0;
		*temp_bool = true;
	}
	else if((temp_count->count >= 1) && *temp_bool == 1)
	{
		temp_count->count = 0;
		*temp_bool = false;
	}
	return *temp_bool;
}
//按键单点
bool rc_key_c::read_key_single(count_num_key *temp_count)
{
	if(temp_count->count >= 1)
	{
		temp_count->count = 0;
		return true;
	}
	else
	{
		temp_count->count = 0;
		return false;
	}
}
//按键长按赋值
bool rc_key_c::read_key_even(count_num_key *temp_count, bool *temp_bool)
{
	if(temp_count->key_flag == 1)
	{
		*temp_bool = true;
	}
	else if(temp_count->key_flag == 0)
	{
		*temp_bool = false;
	}
	return *temp_bool;
}
//按键长按
bool rc_key_c::read_key_even(count_num_key *temp_count)
{
	if(temp_count->key_flag == 1)
	{
		return true;
	}
	else
	{
		return false;
	}
}

uint8_t rc_key_c::read_key(count_num_key *temp_count, key_count_e mode, bool clear)
{
	uint8_t result;
	if(clear == true)
	{
		if(mode == single)
		{
			result = read_key_single(temp_count);
		}
		else if(mode == even)
		{
			result = read_key_even(temp_count);
		}
	}
	else
	{
		if(mode == single)
		{
			result = temp_count->count;
		}
		else if(mode == even)
		{
			result = temp_count->key_flag;
		}
	}
	return result;
}

bool rc_key_c::read_key(count_num_key *temp_count, key_count_e mode, bool *temp_bool)
{
	if(mode == single)
	{
		read_key_single(temp_count, temp_bool);
	}
	else if(mode == even)
	{
		read_key_even(temp_count, temp_bool);
	}
	return *temp_bool;
}

//更新按键
void rc_key_c::rc_key_v_set(RC_ctrl_t *RC)
{
	count_num_key *p = &Key.W;
	for(uint8_t i = 0; i < 16; i++)
	{
		if(RC->key.v & ((uint16_t)1 << i))
		{
			sum_key_count(1, (p + i));
		}
		else
		{
			sum_key_count(0, (p + i));
		}
	}
	//鼠标
	if(RC->mouse.press_l == 1)
	{
		sum_key_count(1, &Press.L);
	}
	else
	{
		sum_key_count(0, &Press.L);
	}
	if(RC->mouse.press_r == 1)
	{
		sum_key_count(1, &Press.R);
	}
	else
	{
		sum_key_count(0, &Press.R);
	}
}
