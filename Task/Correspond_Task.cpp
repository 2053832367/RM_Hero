#include "Correspond_Task.h"
#include "tasks.h"
#include "chassis_power_control.h"
#include "app_preference.h"
#include "tasks.h"




//发送数据
float ch[25];

Correspondence_ctrl Corres;

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

void Correspondence_ctrl::Corres_Init(void)
{
 	  WS2812_INIT();
  //OLED_Init();
	
 	/*视觉发送函数*/
	VisualS.mode=1;
	VisualS.check_byte=1;

	Solvetrajectory.aim_x=16.4;
	Solvetrajectory.aim_y=10;
	Solvetrajectory.aim_z=1.25;
	Solvetrajectory.hero_x=0;
	Solvetrajectory.hero_y=13.075;
	Solvetrajectory.hero_z=0;
	Solvetrajectory.bullet_speed=15.5;

}
extern int Key_count;

void Correspondence_ctrl::Corres_Send(void)
{

	
	VISUAL_SERIAL.sendData(&VisualS.header, sizeof(Visual_Send_Data_t));
	
	//vofa调试代码
	ch[0] =0x78f0;   //软件BUG第一个数据无效
	ch[1] = Chassis.Motor[0].speed;
	ch[2] = Chassis.Motor[1].speed;
	ch[3] = Chassis.Motor[2].speed;
	ch[4] = Chassis.Motor[3].speed;
	//ch[4] = time_set;
	
	vofa_justfloat_output(ch,sizeof(ch)/4,&huart8);
	
	CAN_Cmd.SendData(&hfdcan2, CAN_CAP_SENT_ID, &SuperCapS, 8);
	 
}

void Correspondence_ctrl::Corres_Feedback(void)
{
		/**********数据更新***********/
	  switch(judge_type.game_robot_state.robot_id)
	{
		case 1:  //红色英雄
			VisualS.mode = 1;
			break;
		case 101:  //蓝色英雄
			VisualS.mode = 0;
		 
			break;
		default:
			break;
	}
		/*视觉*/
		VisualS.pitch= Gimbal.DM_Pitch.angle;
		VisualS.roll = Message.MPU_DataXY.AngleX.int_16*FP32_MPU_RAD;
		VisualS.yaw  = Message.DM_data.DM_Yaw;		
	  VisualS.shoot_speed = Message.robo->shoot_data.initial_speed;

	  /*雷达定点打击*/
    autoSolveTrajectory(&Solvetrajectory);
	
		Statistic_Update(xTaskGetTickCount());
}


void Correspondence_ctrl::Corres_Calc(void)
{   
	                                                                          //Message.robo->power_heat_data.chassis_power < 1
	 if(Message.robo->game_robot_state.power_management_chassis_output == 0 ||  Message.robo->game_robot_state.power_management_gimbal_output == 0|| Chassis.Mode == CHASSIS_NO_MOVE)
		{
			SuperCapS.enable = 0x00;
		}
	 else if((Message.SuperCapR.situation == CAP_CLOSE || Message.SuperCapR.situation == CAP_OPEN) && Message.robo->game_robot_state.robot_level >= 1)
		{
			
			SuperCapS.enable = 0xff;
		}
		else
		{
			SuperCapS.enable = 0x00;
		}
//		if(Chassis.Mode==CHASSIS_NO_MOVE)
//		{
//			SuperCapS.enable = 0x00;
//		}
//	else{
//			SuperCapS.enable = 0xff;
//		}
//	
		SuperCapS.mode = 0xFF;
		SuperCapS.power = 0;
		SuperCapS.power_limit = Chassis.Power_Ctrl.Power_limit.Max_input_power;//(uint8_t)Message.robo->game_robot_state.chassis_power_limit;

}

