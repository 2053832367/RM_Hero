#include "Correspond_Task.h"
#include "tasks.h"

union F Gimbal_Union;
uint8_t s[11]={0xff,2,3,4,5,6,7,8,9,0xfe,0};
correspondence_ctrl Corres;

	void Correspond_Task(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	Corres.Corres_Init();
  /* Infinite loop */
  for(;;)
  {		
		Corres.Corres_Feedback();
		Corres.Corres_Send();
		Corres.Corres_Calc();
		
		xQueueSend(Message_Queue, &ID_Data[CorrespondenceData], 0);
    osDelay(Correspondence_Task_Control_Time);
  }
  /* USER CODE END StartDefaultTask */
}

void correspondence_ctrl::Corres_Init(void)
{
	WS2812_INIT();
	
	Data.Shoot_Unoin.F = 14.0f;

	kalman_filter_init_matrix(&Visual_Yaw_Temp, &Visual_Yaw_Init_Matrix);
	kalman_filter_init_matrix(&Visual_Pitch_Temp, &Visual_Pitch_Init_Matrix);
}

void correspondence_ctrl::Corres_Send(void)
{
//	if(Rate_Do_Execute(5))
//	{//10MS一次
//		VISUAL_SERIAL.sendData(0xff);
//		VISUAL_SERIAL.sendData(&Visual_Posture.Yaw[0], 14);
//		VISUAL_SERIAL.sendData(0xfe);
//	}
//	// if(Rate_Do_Execute(5))
//	// {//10MS一次
//	// 	Append_CRC8_Check_Sum(&Visual_Posture.Header, sizeof(Visual_Posture_Data_t));
//	// 	VISUAL_SERIAL.sendData(&Visual_Posture, sizeof(Visual_Posture_Data_t));
//	// }
//	// if(Rate_Do_Execute(50))
//	// {
//	// 	Append_CRC8_Check_Sum(&Game_Sate.Header, sizeof(game_robot_state_t_));
//	// 	VISUAL_SERIAL.sendData(&Game_Sate, sizeof(game_robot_state_t_));
//	// }
//	// if(Rate_Do_Execute(50))
//	// {
//	// 	Append_CRC8_Check_Sum(&Visual_Mode.Header, sizeof(Visual_Mode_Data_t));
//	// 	VISUAL_SERIAL.sendData(&Visual_Mode, sizeof(Visual_Mode_Data_t));
//	// }
//	// if(Rate_Do_Execute(50))
//	// {
//	// 	Append_CRC8_Check_Sum(&Game_HP.Header, sizeof(Game_HP_t));
//	// 	VISUAL_SERIAL.sendData(&Game_HP, sizeof(Game_HP_t));
//	// }
//	Append_CRC8_Check_Sum(&ChassisS.Header, sizeof(Chassis_Send_Data_t));
//	CHASSIS_SERIAL.sendData(&ChassisS, sizeof(Chassis_Send_Data_t));
//	Corres.RGB_Send();

	if(Rate_Do_Execute(500))
		{
			Append_CRC8_Check_Sum(s, sizeof(s));
			Serial1_Ctrl.sendData(s, sizeof(s));
			Serial3_Ctrl.sendData(s, sizeof(s));
			Serial7_Ctrl.sendData(s, sizeof(s));
			Serial8_Ctrl.sendData(s, sizeof(s));
		}
}

void correspondence_ctrl::Corres_Feedback(void)
{
	//数据更新
	uint8_t i;
	if(Rate_Do_Execute(5))
	{
		if(Gimbal.Mode == GIMBAL_AIM)
		{//开视觉yaw从0开始发送
			Data.Yaw_Unoin.F = Gimbal.Yaw.angle - Data.Yaw_error;
			Data.Yaw_visual_angle = Gimbal.Yaw.angle - Data.Yaw_error;
		}
		else
		{
			Data.Yaw_Unoin.F = 0.0f;
			Data.Yaw_error = Gimbal.Yaw.angle;
		}

		Data.Pitch_Unoin.F = Gimbal.Pitch.angle;
		if(Message.Game_State.bullet_speed < 10)
		{
			Data.Shoot_Unoin.F = Gimbal.Data.Fric_Gear[Gimbal.Data.Gear - 1] - 1;
		}
		else
		{
			Data.Shoot_Unoin.F = Message.Game_State.bullet_speed;
		}
		if(Message.Game_State.robot_id != NULL)
		{
			Visual_Posture.Color = Message.Game_State.robot_id / 100;
		}
		if(Message.game_status.game_progress == 4)
		{
			Visual_Posture.Progress = 1;
		}
		else if(Message.game_status.game_progress != 4)
		{
			Visual_Posture.Progress = 0;
		}

		for(i = 0;i < 4;i++)
		{
			Visual_Posture.Yaw[i] = Data.Yaw_Unoin.I[i];
			Visual_Posture.Pitch[i] = Data.Pitch_Unoin.I[i];
			Visual_Posture.Shoot[i] = Data.Shoot_Unoin.I[i];
		}
	}

	// if(Rate_Do_Execute(50))
	// {
	// 	Game_Sate.robot_id = Message.Game_State.robot_id;
	// }

	// if(Rate_Do_Execute(50))
	// {
	// 	if(Gimbal.Flags.Energy_Flag == true && Gimbal.Flags.Visual_Flag == true)
	// 	{
	// 		Visual_Mode.mode = 0x04;
	// 	}
	// 	else if(Gimbal.Flags.Visual_Flag == true)
	// 	{
	// 		Visual_Mode.mode = 0x02;
	// 	}
	// 	else
	// 	{
	// 		Visual_Mode.mode = 0x01;
	// 	}
	// }

	// if(Rate_Do_Execute(50))
	// {
	// 	memcpy(&Game_HP.game_robot_HP, &Message.Game_HP, sizeof(ext_game_robot_HP_t));
	// }

	ChassisS.ECD = (int16_t)Gimbal.Yaw.gimbal_motor_measure->ecd;
	ChassisS.goal = Message.VisualR.Goal;

	Statistic_Update(xTaskGetTickCount());
}

void correspondence_ctrl::Corres_Calc(void)
{
	//视觉防丢
	if(Message.VisualR.State == true)
	{
		if(Message.VisualR.target < 200)
		{
			Message.VisualR.target++;
		}
	}
	else
	{
		if(Message.VisualR.target > 0)
		{
			Message.VisualR.target--;
		}
	}

	Message.VisualR.Yaw_Kalman_result = kalman_filter_calc(&Visual_Yaw_Temp, Message.VisualR.Error_X, Message.VisualR.Yaw_Speed);
	Message.VisualR.Pitch_Kalman_result = kalman_filter_calc(&Visual_Pitch_Temp, Message.VisualR.Error_Y, Message.VisualR.Pitch_Speed);

	Message.VisualR.Error_X_kalman = Message.VisualR.Yaw_Kalman_result[0] * 200.0f;
	Message.VisualR.Error_Y_kalman = Message.VisualR.Pitch_Kalman_result[0] * 200.0f;

	Message.VisualR.Yaw_Speed_kalman = Message.VisualR.Yaw_Kalman_result[1] * 200.0f;
	Message.VisualR.Pitch_Speed_kalman = Message.VisualR.Pitch_Kalman_result[1] * 200.0f;

	// if(Rate_Do_Execute(5))
	// {
	// 	Message.MahonyAHRSupdate((float)Message.Gyro.data.SpeedX * 0.1f * DEG_TO_RAD, (float)Message.Gyro.data.SpeedY * 0.1f * DEG_TO_RAD, (float)Message.Gyro.data.SpeedZ * 0.1f * DEG_TO_RAD,
	// 		(float)Message.Gyro.data.AccX * 0.001f, (float)Message.Gyro.data.AccY * 0.001f, (float)Message.Gyro.data.AccZ * 0.001f,
	// 		(float)Message.Gyro.data.MagX * 0.001f, (float)Message.Gyro.data.MagY * 0.001f, (float)Message.Gyro.data.MagZ * 0.001f);
	// 	Message.ToEulerAngles(Message.Gyro.calc.q0, Message.Gyro.calc.q1, Message.Gyro.calc.q2, Message.Gyro.calc.q3);
	// }
	if(Message.Gyro.Yaw_angle_offset == 0)
	{
		Message.Gyro.Yaw_angle_offset = Message.Gyro.angle.AngleY_Calc;
		return;
	}
	// Message.Gyro.Yaw_angle = Message.Gyro.angle.AngleY_Calc;
}

void correspondence_ctrl::RGB_Send(void)
{
	if (Gimbal.Flags.Fric_Flag == 1)//黄
		{
			WS2812_SET_HSV(60,100,1,4);
		}
		else if (Gimbal.Flags.Fric_Flag == 0)
		{
			WS2812_SET_HSV(0,0,0,4);
		}
		if (Gimbal.Mode == GIMBAL_Normal)
		{
			WS2812_SET_HSV(120,100,1,6);
		}
		if (Gimbal.Mode == GIMBAL_Normal && Message.VisualR.Goal == 1)
		{
			
			WS2812_SET_HSV(60,100,1,6);
//			WS2812_SET_HSV(210,88,1,3);//lan
//			WS2812_SET_HSV(210,88,1,3);//lv
//			WS2812_SET_HSV(60,100,1,4);//huang
//			WS2812_SET_HSV(33,100,1,5);//cheng
//			WS2812_SET_HSV(0,100,1,6);//hong
//			WS2812_SET_HSV(283,100,1,7);//zi
			
		}
		else if (Gimbal.Mode == GIMBAL_AIM)
		{
			
			WS2812_SET_HSV(0,100,1,6);
//			WS2812_SET_HSV(210,88,1,3);//lan
//			WS2812_SET_HSV(210,88,1,3);//lv
//			WS2812_SET_HSV(60,100,1,4);//huang
//			WS2812_SET_HSV(33,100,1,5);//cheng
//			WS2812_SET_HSV(0,100,1,6);//hong
//			WS2812_SET_HSV(283,100,1,7);//zi
			
		}
		else if (Gimbal.Mode == GIMBAL_NO_MOVE)
		{
			WS2812_SET_HSV(0,0,0,6);
//			WS2812_SET_HSV(60,100,1,4);
		}
		switch(Message.ChassisR.mode) //不亮：无力；1，2，3：跟随，不跟随，小陀螺 ：红绿蓝
		{
			case 0x01:
				WS2812_SET_HSV(0,100,1,3);
				break;
			case 0x02:
				WS2812_SET_HSV(120,100,1,3);
				break;
			case 0x03:
				WS2812_SET_HSV(228,100,1,3);
				break;
			default:
				WS2812_SET_HSV(0,0,0,3);
				break;
		}
//		switch(Message.ChassisR.supercap)
//		{
//			case 1:
//				WS2812_SET_HSV(0,0,0,0);
//			case 2:
//				WS2812_SET_HSV(0,0,0,0);
//			case 3:
//				WS2812_SET_HSV(0,0,0,0);
//			default:
//				break;
//		}
		if (Gimbal.Flags.Loading_Flag == 1)
		{
			WS2812_SET_HSV(60,100,1,2);//黄
		}
		else if (Gimbal.Flags.Loading_Flag == 0)
		{
			WS2812_SET_HSV(0,0,0,2);
		}
}

