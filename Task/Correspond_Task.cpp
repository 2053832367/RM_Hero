#include "Correspond_Task.h"

union F Gimbal_Union;

correspondence_ctrl Corres;
uint8_t s[10]={1,2,3,4,5,6,7,8,9,0};
	void Correspond_Task(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {		
		Corres.Corres_Feedback();
		Corres.Corres_Send();
		
		xQueueSend(Message_Queue, &ID_Data[CorrespondenceData], 0);
    osDelay(2);
  }
  /* USER CODE END StartDefaultTask */
}

void correspondence_ctrl::Corres_Init(void)
{

}

void correspondence_ctrl::Corres_Send(void)
{
//	if(Rate_Do_Execute(4))
//	{
//		Append_CRC8_Check_Sum(&game_status.Header, sizeof(game_status_t));
//		GIMBAL_SERIAL.sendData(&game_status, sizeof(game_status_t));
//	}
//	if(Rate_Do_Execute(5))
//	{
//		Append_CRC8_Check_Sum(&game_robot_state.Header, sizeof(game_robot_state_t));
//		GIMBAL_SERIAL.sendData(&game_robot_state, sizeof(game_robot_state_t));
//	}
//	if(Rate_Do_Execute(9))
//	{
//		Append_CRC8_Check_Sum(&GimbalS.Header, sizeof(Gimbal_Send_Data_t));
//		GIMBAL_SERIAL.sendData(&GimbalS, sizeof(Gimbal_Send_Data_t));
//	}
////	if(Rate_Do_Execute(500))
////	{
////		Append_CRC8_Check_Sum(&game_robot_HP.Header, sizeof(game_robot_HP_t));
////		GIMBAL_SERIAL.sendData(&game_robot_HP, sizeof(game_robot_HP_t));
////	}

//	CAN_Cmd.SendData(CAN1, CAN_CAP_SENT_ID, &SuperCapS, 8);

		if(Rate_Do_Execute(5))
		{
//			GIMBAL_SERIAL.sendData(&s, sizeof(s));
		}

}

void correspondence_ctrl::Corres_Feedback(void)
{
//	uint8_t i;

	Statistic_Update(xTaskGetTickCount());

//	game_status.game_progress = Message.robo->game_status.game_progress;
//	
//	game_robot_state.robot_id = Message.robo->game_robot_state.robot_id;
//	game_robot_state.robot_level = Message.robo->game_robot_state.robot_level;
//	game_robot_state.remain_HP = Message.robo->game_robot_state.remain_HP;
//	game_robot_state.bullet_speed = Message.robo->shoot_data.bullet_speed;
//	game_robot_state.bullet_speed_limit = Message.robo->game_robot_state.shooter_id1_17mm_speed_limit;

//	GimbalS.mode = Chassis.Mode;
//	GimbalS.supercap = Message.SuperCapR.mode;
//	
//	if(Message.robo->game_robot_state.mains_power_chassis_output == 0 || Message.robo->power_heat_data.chassis_power < 1
//		|| Message.robo->game_robot_state.mains_power_gimbal_output == 0)
//	{
//		SuperCapS.enable = 0x00;
//	}
//	else if((Message.SuperCapR.situation == CAP_CLOSE || Message.SuperCapR.situation == CAP_OPEN) && Message.robo->game_robot_state.robot_level >= 1)
//	{
//		SuperCapS.enable = 0xff;
//	}
//	else
//	{
//		SuperCapS.enable = 0x00;
//	}
//	SuperCapS.mode = 0xff;

//	SuperCapS.power = (uint8_t)Message.robo->power_heat_data.chassis_power;
//	SuperCapS.power_limit = (uint8_t)Message.robo->game_robot_state.chassis_power_limit;




}
