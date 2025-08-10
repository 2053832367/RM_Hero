#include "Robot_Task.h"
#include "tasks.h"
#include "arm_math.h"
#include <math.h>
#include <vector>



Gimbal_Ctrl Gimbal;
int8_t turn_top = 1;//这是什么？（表示云台的转动方向，1表示向上转动，-1表示向下转动）
int16_t last_speed;//上次速度
int Quan,speed1;
int GM6020_last_angle,GM6020_E_ecd;
float GM6020_E_angle;
float lastspeed;
float GM6020_speed;
int16_t time_set;
float tos;
int8_t num = 0;
float last_time;
float angle_4340_set;
int16_t TuChuan_error =1,WuangYuan_error = 1;
void reset();
uint8_t DM_Trigger_flag=1;
float vel=700,angle1;

float Two_Ji_SpeedSet,Two_Ji_FricSpeed;


int Key_count=0;

void KEY_2006()
{
		if(!HAL_GPIO_ReadPin(KEY_2006_GPIO_Port,KeY_2006_Pin)      && Key_count==0)/*往前送弹到第一次碰到微动开关*/
		{
				Key_count=1;
		}
		else if( HAL_GPIO_ReadPin(KEY_2006_GPIO_Port,KeY_2006_Pin) && Key_count==1)/*第一发弹丸发出，微动开关松开*/
		{
			  Key_count=2;
		}
//		else if(!HAL_GPIO_ReadPin(KEY_2006_GPIO_Port,KeY_2006_Pin) && Key_count==2)/*到第二发弹丸碰到微动开关后，拨弹盘停，重置*/
//		{
//			  Key_count=3;
//		}
		
}


//云台任务
void Gimbal_Task(void *pvParameters)
{

	Gimbal.Gimbal_Init();
	/* Infinite loop */
	for(;;)
	{	
		KEY_2006();
		//在behaviour的末尾行执行了Flag_Behaviour_Control
		Gimbal.Behaviour_Mode();
		Gimbal.Feedback_Update();
		//摩擦轮电机的数据更新在Message的任务中Message_Ctrl::CAN3_Process(CanRxMsg *Rx_Message)
		num++;
	  if(num >=2)
		{
			num = 0;
		}
		
		if (Gimbal.Mode == GIMBAL_NO_MOVE)
		{
			
			CAN_Cmd.SendData(&CAN_Cmd.Bottom,0);
			CAN_Cmd.SendData(&CAN_Cmd.Fric,  0,0,0);  
			CAN_Cmd.SendData(&CAN_Cmd.Fric02,0,0,0);
			CAN_Cmd.DM_SendData(&CAN_Cmd.DM_Trigger,0);
			CAN_Cmd.DM_SendData(&CAN_Cmd.Gimbal_DM_Pitch,0,0,0,0,-Gimbal.DM_Pitch.tor_set);
			
			CAN_Cmd.SendData(&CAN_Cmd.TuChuan1,0,0,0,0);
		}
		else
		{
			Gimbal.Control();
			Gimbal.Control_loop();
			
			CAN_Cmd.DM_SendData(&CAN_Cmd.DM_Trigger,Gimbal.Trigger.vel_set);
			if(num)
			{
				
				
				CAN_Cmd.SendData(&CAN_Cmd.TuChuan1,0,Gimbal.TuChuan1.give_current,Gimbal.WuangYuan.give_current,0);
				CAN_Cmd.SendData(&CAN_Cmd.Fric,  Gimbal.Fric1.give_current,Gimbal.Fric2.give_current,Gimbal.Fric3.give_current,Gimbal.Fric4.give_current);
				CAN_Cmd.SendData(&CAN_Cmd.Fric02,  Gimbal.Fric5.give_current,Gimbal.Fric6.give_current,0,0);
				//CAN_Cmd.SendData(&CAN_Cmd.Fric02,Gimbal.Fric4.give_current,Gimbal.Fric5.give_current,Gimbal.Fric6.give_current);
				
				


				
			
			}
			
			
			CAN_Cmd.SendData(&CAN_Cmd.Bottom,Gimbal.Yaw.give_current);
			
			
			CAN_Cmd.DM_SendData(&CAN_Cmd.Gimbal_DM_Pitch,0,0,0,0,-Gimbal.DM_Pitch.tor_set-Gimbal.DM_Pitch.vel_set);
			
			
		}
		
		
		Gimbal.Stop_TickCount = DWT_GetTimeline_s();
		xQueueSend(Message_Queue, &ID_Data[GimbalData],0);
		osDelay(GIMBAL_CONTROL_TIME);
	}
	/* USER CODE END StartDefaultTask */
}

// 初始化
void Gimbal_Ctrl::Gimbal_Init(void)
{
	uint8_t i;
	/* 遥控器数据获取*/
	RC_Ptr = get_remote_control_point();
	
	DM_Pitch.R=0.0f;
	/* 	摩擦轮电机的数据更新在Message的任务中Message_Ctrl::CAN3_Process(CanRxMsg *Rx_Message)  */
	DM_Pitch.gimbal_motor_measure= CAN_Cmd.Gimbal_DM_Pitch.Get_DM_Motor_Measure_Pointer();
	Yaw.gimbal_motor_measure     = CAN_Cmd.Bottom.Get_Motor_Measure_Pointer(0);
	Trigger.gimbal_motor_measure = CAN_Cmd.DM_Trigger.Get_DM_Motor_Measure_Pointer();
	TuChuan1.gimbal_motor_measure = CAN_Cmd.TuChuan1.Get_Motor_Measure_Pointer(0);
	WuangYuan.gimbal_motor_measure = CAN_Cmd.TuChuan1.Get_Motor_Measure_Pointer(1);
	/*摩擦轮数据获取*/
	Fric1.gimbal_motor_measure = CAN_Cmd.Fric.Get_Motor_Measure_Pointer(0);
	Fric2.gimbal_motor_measure = CAN_Cmd.Fric.Get_Motor_Measure_Pointer(1);
	Fric3.gimbal_motor_measure = CAN_Cmd.Fric.Get_Motor_Measure_Pointer(2);
	Fric4.gimbal_motor_measure = CAN_Cmd.Fric02.Get_Motor_Measure_Pointer(0);
	Fric5.gimbal_motor_measure = CAN_Cmd.Fric02.Get_Motor_Measure_Pointer(1);
	Fric6.gimbal_motor_measure = CAN_Cmd.Fric02.Get_Motor_Measure_Pointer(2);

/*
PID初始化区域
*/

	/*手动*/
	
	
	/*Yaw*/
	PID_CT.Init(&Yaw.PositinPid,Yaw_POSITION_PID_MAX_OUT,Yaw_POSITION_PID_MAX_IOUT,Yaw_POSITION_PID_Deadband,Yaw_POSITION_PID_KP,Yaw_POSITION_PID_KI,Yaw_POSITION_PID_KD
	                           ,Yaw_POSITION_PID_I_KA,Yaw_POSITION_PID_I_KB,Yaw_POSITION_PID_OUT_Filter,Yaw_POSITION_PID_D_Filter,Yaw_POSITION_PID_ols_order,Yaw_POSITION_PID_IP);
	
  PID_CT.Init(&Yaw.SpeedPid  ,Yaw_SPEED_PID_MAX_OUT,Yaw_SPEED_PID_MAX_IOUT,Yaw_SPEED_PID_Deadband,Yaw_SPEED_PID_KP,Yaw_SPEED_PID_KI,Yaw_SPEED_PID_KD
	                           ,Yaw_SPEED_PID_I_KA,Yaw_SPEED_PID_I_KB,Yaw_SPEED_PID_OUT_Filter,Yaw_SPEED_PID_D_Filter,Yaw_SPEED_PID_ols_order,Yaw_SPEED_PID_IP);
	
	/*吊射Yaw*/
	PID_CT.Init(&Yaw.PositinPid_GIMBAL_LOB_SHOT,Yaw_LOB_SHOT_POSITION_PID_MAX_OUT,Yaw_LOB_SHOT_POSITION_PID_MAX_IOUT,Yaw_LOB_SHOT_POSITION_PID_Deadband,Yaw_LOB_SHOT_POSITION_PID_KP,Yaw_LOB_SHOT_POSITION_PID_KI,Yaw_LOB_SHOT_POSITION_PID_KD
	                           ,Yaw_LOB_SHOT_POSITION_PID_I_KA,Yaw_LOB_SHOT_POSITION_PID_I_KB,Yaw_LOB_SHOT_POSITION_PID_OUT_Filter,Yaw_LOB_SHOT_POSITION_PID_D_Filter,Yaw_LOB_SHOT_POSITION_PID_ols_order,Yaw_LOB_SHOT_POSITION_PID_IP);
	
  PID_CT.Init(&Yaw.SpeedPid_GIMBAL_LOB_SHOT  ,Yaw_LOB_SHOT_SPEED_PID_MAX_OUT,Yaw_LOB_SHOT_SPEED_PID_MAX_IOUT,Yaw_LOB_SHOT_SPEED_PID_Deadband,Yaw_LOB_SHOT_SPEED_PID_KP,Yaw_LOB_SHOT_SPEED_PID_KI,Yaw_LOB_SHOT_SPEED_PID_KD
	                           ,Yaw_LOB_SHOT_SPEED_PID_I_KA,Yaw_LOB_SHOT_SPEED_PID_I_KB,Yaw_LOB_SHOT_SPEED_PID_OUT_Filter,Yaw_LOB_SHOT_SPEED_PID_D_Filter,Yaw_LOB_SHOT_SPEED_PID_ols_order,Yaw_LOB_SHOT_SPEED_PID_IP);
	/*Pitch*/
	PID_CT.Init(&DM_Pitch.PositinPid,PITCH_POSITION_PID_MAX_OUT,PITCH_POSITION_PID_MAX_IOUT,PITCH_POSITION_PID_Deadband,PITCH_POSITION_PID_KP,PITCH_POSITION_PID_KI,PITCH_POSITION_PID_KD
	                                ,PITCH_POSITION_PID_I_KA,PITCH_POSITION_PID_I_KB,PITCH_POSITION_PID_OUT_Filter,PITCH_POSITION_PID_D_Filter,PITCH_POSITION_PID_ols_order,PITCH_POSITION_PID_IP);
	
  PID_CT.Init(&DM_Pitch.SpeedPid  ,PITCH_SPEED_PID_MAX_OUT,PITCH_SPEED_PID_MAX_IOUT,PITCH_SPEED_PID_Deadband,PITCH_SPEED_PID_KP,PITCH_SPEED_PID_KI,PITCH_SPEED_PID_KD
	                                ,PITCH_SPEED_PID_I_KA,PITCH_SPEED_PID_I_KB,PITCH_SPEED_PID_OUT_Filter,PITCH_SPEED_PID_D_Filter,PITCH_SPEED_PID_ols_order,PITCH_SPEED_PID_IP);
	
	/*拨弹轮4340*/
	PID_CT.Init(&Trigger.PositinPid,TRIGGER_POSITION_PID_MAX_OUT,TRIGGER_POSITION_PID_MAX_IOUT,TRIGGER_POSITION_PID_Deadband,TRIGGER_POSITION_PID_KP \
							,TRIGGER_POSITION_PID_KI,TRIGGER_POSITION_PID_KD,TRIGGER_POSITION_PID_I_KA,TRIGGER_POSITION_PID_I_KB,TRIGGER_POSITION_PID_OUT_Filter \
							,TRIGGER_POSITION_PID_D_Filter,TRIGGER_POSITION_PID_ols_order,TRIGGER_POSITION_PID_IP);
	
  PID_CT.Init(&Trigger.SpeedPid  ,TRIGGER_SPEED_PID_MAX_OUT,TRIGGER_SPEED_PID_MAX_IOUT,TRIGGER_SPEED_PID_Deadband,TRIGGER_SPEED_PID_KP \
							,TRIGGER_SPEED_PID_KI,TRIGGER_SPEED_PID_KD,TRIGGER_SPEED_PID_I_KA,TRIGGER_SPEED_PID_I_KB,TRIGGER_SPEED_PID_OUT_Filter \
							,TRIGGER_SPEED_PID_D_Filter,TRIGGER_SPEED_PID_ols_order,TRIGGER_SPEED_PID_IP);
	
	
	
	
	
	/*Yaw_视觉*/
	PID_CT.Init(&Yaw.FollowPositinPid,Yaw_POSITION_VisualR_PID_MAX_OUT,Yaw_POSITION_VisualR_PID_MAX_IOUT,Yaw_POSITION_VisualR_PID_Deadband,Yaw_POSITION_VisualR_PID_KP,Yaw_POSITION_VisualR_PID_KI,Yaw_POSITION_VisualR_PID_KD
	                              ,Yaw_POSITION_VisualR_PID_I_KA,Yaw_POSITION_VisualR_PID_I_KB,Yaw_POSITION_VisualR_PID_OUT_Filter,Yaw_POSITION_VisualR_PID_D_Filter,Yaw_POSITION_VisualR_PID_ols_order,Yaw_POSITION_VisualR_PID_IP);
	
  PID_CT.Init(&Yaw.FollowSpeedPid  ,Yaw_SPEED_PID_VisualR_MAX_OUT,Yaw_SPEED_PID_VisualR_MAX_IOUT,Yaw_SPEED_PID_VisualR_Deadband,Yaw_SPEED_PID_VisualR_KP,Yaw_SPEED_PID_VisualR_KI,Yaw_SPEED_PID_VisualR_KD
	                              ,Yaw_SPEED_PID_VisualR_I_KA,Yaw_SPEED_PID_VisualR_I_KB,Yaw_SPEED_PID_VisualR_OUT_Filter,Yaw_SPEED_PID_VisualR_D_Filter,Yaw_SPEED_PID_VisualR_ols_order,Yaw_SPEED_PID_VisualR_IP);
	
	/*DM_Pitch_视觉*/
	PID_CT.Init(&DM_Pitch.FollowPositinPid,PITCH_POSITION_PID_VisualR_MAX_OUT,PITCH_POSITION_PID_VisualR_MAX_IOUT,PITCH_POSITION_PID_VisualR_Deadband,PITCH_POSITION_PID_VisualR_KP,PITCH_POSITION_PID_VisualR_KI,PITCH_POSITION_PID_VisualR_KD
	                                ,PITCH_POSITION_PID_VisualR_I_KA,PITCH_POSITION_PID_I_KB,PITCH_POSITION_PID_VisualR_OUT_Filter,PITCH_POSITION_PID_VisualR_D_Filter,PITCH_POSITION_PID_VisualR_ols_order,PITCH_POSITION_PID_VisualR_IP);
	
  PID_CT.Init(&DM_Pitch.FollowSpeedPid  ,PITCH_SPEED_PID_VisualR_MAX_OUT,PITCH_SPEED_PID_VisualR_MAX_IOUT,PITCH_SPEED_PID_VisualR_Deadband,PITCH_SPEED_PID_VisualR_KP,PITCH_SPEED_PID_VisualR_KI,PITCH_SPEED_PID_VisualR_KD
	                                ,PITCH_SPEED_PID_VisualR_I_KA,PITCH_SPEED_PID_VisualR_I_KB,PITCH_SPEED_PID_VisualR_OUT_Filter,PITCH_SPEED_PID_VisualR_D_Filter,PITCH_SPEED_PID_VisualR_ols_order,PITCH_SPEED_PID_VisualR_IP);

	
	
  /*摩擦轮*/
	PID.Init(&Fric1.SpeedPid, POSITION, FRIC3_SPEED_PID_KP, FRIC3_SPEED_PID_KI, FRIC3_SPEED_PID_KD, FRIC3_PID_MAX_OUT, FRIC3_PID_MAX_IOUT, FRIC3_PID_BAND_I);
	PID.Init(&Fric2.SpeedPid, POSITION, FRIC3_SPEED_PID_KP, FRIC3_SPEED_PID_KI, FRIC3_SPEED_PID_KD, FRIC3_PID_MAX_OUT, FRIC3_PID_MAX_IOUT, FRIC3_PID_BAND_I);
	PID.Init(&Fric3.SpeedPid, POSITION, FRIC3_SPEED_PID_KP, FRIC3_SPEED_PID_KI, FRIC3_SPEED_PID_KD, FRIC3_PID_MAX_OUT, FRIC3_PID_MAX_IOUT, FRIC3_PID_BAND_I);
	PID.Init(&Fric4.SpeedPid, POSITION, FRIC3_SPEED_PID_KP, FRIC3_SPEED_PID_KI, FRIC3_SPEED_PID_KD, FRIC3_PID_MAX_OUT, FRIC3_PID_MAX_IOUT, FRIC3_PID_BAND_I);
  	PID.Init(&Fric5.SpeedPid, POSITION, FRIC3_SPEED_PID_KP, FRIC3_SPEED_PID_KI, FRIC3_SPEED_PID_KD, FRIC3_PID_MAX_OUT, FRIC3_PID_MAX_IOUT, FRIC3_PID_BAND_I);
	PID.Init(&Fric6.SpeedPid, POSITION, FRIC6_SPEED_PID_KP, FRIC6_SPEED_PID_KI, FRIC6_SPEED_PID_KD, FRIC6_PID_MAX_OUT, FRIC6_PID_MAX_IOUT, FRIC6_PID_BAND_I);
	/*主动预置轮*/
	//PID.Init(&Fric6.SpeedPid, POSITION, FRIC6_SPEED_PID_KP, FRIC6_SPEED_PID_KI, FRIC6_SPEED_PID_KD, FRIC6_PID_MAX_OUT, FRIC6_PID_MAX_IOUT, FRIC6_PID_BAND_I);
	
	/*图传2006*/
	PID_CT.Init(&TuChuan1.PositinPid,TuChuan1_POSITION_PID_MAX_OUT,TuChuan1_POSITION_PID_MAX_IOUT,TuChuan1_POSITION_PID_Deadband,TuChuan1_POSITION_PID_KP,TuChuan1_POSITION_PID_KI,TuChuan1_POSITION_PID_KD,TuChuan1_POSITION_PID_I_KA,TuChuan1_POSITION_PID_I_KB,TuChuan1_POSITION_PID_OUT_Filter,TuChuan1_POSITION_PID_D_Filter,TuChuan1_POSITION_PID_ols_order,Yaw_POSITION_PID_IP);
	
	PID_CT.Init(&TuChuan1.SpeedPid  ,TuChuan1_SPEED_PID_MAX_OUT,TuChuan1_SPEED_PID_MAX_IOUT,TuChuan1_SPEED_PID_Deadband,TuChuan1_SPEED_PID_KP,TuChuan1_SPEED_PID_KI,TuChuan1_SPEED_PID_KD,TuChuan1_SPEED_PID_I_KA,TuChuan1_SPEED_PID_I_KB,TuChuan1_SPEED_PID_OUT_Filter,TuChuan1_SPEED_PID_D_Filter,TuChuan1_SPEED_PID_ols_order,TuChuan1_SPEED_PID_IP);
	
	/*望远镜2006*/
	PID_CT.Init(&WuangYuan.PositinPid ,WuangYuan_POSITION_PID_MAX_OUT ,WuangYuan_POSITION_PID_MAX_IOUT ,WuangYuan_POSITION_PID_Deadband \
							,WuangYuan_POSITION_PID_KP ,WuangYuan_POSITION_PID_KI ,WuangYuan_POSITION_PID_KD ,WuangYuan_POSITION_PID_I_KA \
							,WuangYuan_POSITION_PID_I_KB ,WuangYuan_POSITION_PID_OUT_Filter ,WuangYuan_POSITION_PID_D_Filter \
							,TuChuan1_POSITION_PID_ols_order,Yaw_POSITION_PID_IP);
	
	
	PID_CT.Init(&WuangYuan.SpeedPid ,WuangYuan_SPEED_PID_MAX_OUT ,WuangYuan_SPEED_PID_MAX_IOUT ,WuangYuan_SPEED_PID_Deadband \
							,WuangYuan_SPEED_PID_KP ,WuangYuan_SPEED_PID_KI ,WuangYuan_SPEED_PID_KD ,WuangYuan_SPEED_PID_I_KA \
							,WuangYuan_SPEED_PID_I_KB ,WuangYuan_SPEED_PID_OUT_Filter,WuangYuan_SPEED_PID_D_Filter \
							,WuangYuan_SPEED_PID_ols_order,WuangYuan_SPEED_PID_IP);
	
	PID_CT.Init(&WuangYuan.SpeedPid0 ,WuangYuan_SPEED0_PID_MAX_OUT ,WuangYuan_SPEED0_PID_MAX_IOUT ,WuangYuan_SPEED0_PID_Deadband \
							,WuangYuan_SPEED0_PID_KP ,WuangYuan_SPEED0_PID_KI ,WuangYuan_SPEED0_PID_KD ,WuangYuan_SPEED0_PID_I_KA \
							,WuangYuan_SPEED0_PID_I_KB ,WuangYuan_SPEED0_PID_OUT_Filter,WuangYuan_SPEED0_PID_D_Filter \
							,WuangYuan_SPEED0_PID_ols_order,WuangYuan_SPEED0_PID_IP);
	
	
	
//	/*拨弹轮*/
//	PID.Init(&Trigger.PositinPid, POSITION, TRIGGER_ANGLE_PID_KP, TRIGGER_ANGLE_PID_KI, TRIGGER_ANGLE_PID_KD, TRIGGER_ANGLE_PID_MAX_OUT, TRIGGER_ANGLE_PID_MAX_IOUT, TRIGGER_ANGLE_PID_BAND_I);
//	PID.Init(&Trigger.SpeedPid,   POSITION, TRIGGER_SPEED_PID_KP, TRIGGER_SPEED_PID_KI, TRIGGER_SPEED_PID_KD, TRIGGER_SPEED_PID_MAX_OUT, TRIGGER_SPEED_PID_MAX_IOUT, TRIGGER_SPEED_PID_BAND_I); /*  */
	

  CAN_Cmd.DM_Motor_Enable     (&CAN_Cmd.DM_Trigger,0x200);
	
	Data.Trigger_once_num = TRIGGER_ONCE_SHOOT_NUM; 
	Data.pitch_offset_ecd = GIMBAL_PITCH_OFFSET_ECD; 
	Data.pitch_max_angle  = GIMBAL_PITCH_MAX_ANGLE;  //Pitch最大俯仰角限位
	Data.pitch_min_angle  = GIMBAL_PITCH_MIN_ANGLE;	 //Pitch最小俯仰角限位

	//测试用的弹速
	Data.FricSpeed = -4800;               /*官方弹丸16.0m/s      4790;  */
	Two_Ji_FricSpeed = -4800;
	
	
	Data.Presets   =0;   //3000;

	Data.Mirror_open        = MIRROR_OPEN_DUTY;
	Data.Mirror_close       = MIRROR_CLOSE_DUTY;
	Data.Image_Hanging_Shot = IMAGE_HANGING_SHOT;
	Data.Image_Normal       = IMAGE_NORMAL;
	
	
	Feedback_Update();
	
	Flags.Mirror_Flag = false;
	Gimbal.Flags.TuChuan_reset_end_Flag = false;
	Gimbal.Flags.start_count_Flag = false;
	
	//CAN_Cmd.DM_Motor_SetZeroT(&CAN_Cmd.Gimbal_DM_Pitch);      //达妙软件标零点

}

//数据更新
void Gimbal_Ctrl::Feedback_Update(void)
{

	if(DM_Trigger_flag  && Gimbal.Trigger.gimbal_motor_measure->state== 1)
	{
		
		angle_4340_set = Gimbal.Trigger.gimbal_motor_measure->POS.fdata;
		
		DM_Trigger_flag--;
		
	}







	
	Gimbal_DWT_dt = DWT_GetDeltaT(&Gimbal_DWT_Count);
	
		/*检测DM电机状态,使能*/
	if(DM_Pitch.gimbal_motor_measure->state!=1){
		CAN_Cmd.DM_Motor_clear_error(&CAN_Cmd.Gimbal_DM_Pitch);
		osDelay(2);
			CAN_Cmd.DM_Motor_Enable(&CAN_Cmd.Gimbal_DM_Pitch);
		
	}
	if(Trigger.gimbal_motor_measure->state!=1){
		CAN_Cmd.DM_Motor_clear_error(&CAN_Cmd.DM_Trigger);
		osDelay(2);
			CAN_Cmd.DM_Motor_Enable     (&CAN_Cmd.DM_Trigger,0x200);
	}
	
	
	if((Gimbal.Yaw.gimbal_motor_measure->ecd - Gimbal.Yaw.gimbal_motor_measure->last_ecd) >5552)
		{
			Quan--;
		}
		else if((Gimbal.Yaw.gimbal_motor_measure->ecd - Gimbal.Yaw.gimbal_motor_measure->last_ecd) <-5552)
		{
			
			Quan++;
		}
		
		
	if((Gimbal.TuChuan1.gimbal_motor_measure->ecd - Gimbal.Last_2006_TuChuan_Ecd)>5555)
	{
		Gimbal.TuChuan_count--;
		
		
	}else if((Gimbal.TuChuan1.gimbal_motor_measure->ecd - Gimbal.Last_2006_TuChuan_Ecd)<-5555)
	{
		Gimbal.TuChuan_count++;
		
	}
	Gimbal.Last_2006_TuChuan_Ecd = Gimbal.TuChuan1.gimbal_motor_measure->ecd;
	
	Gimbal.TuChuan_Ecd = Gimbal.TuChuan1.gimbal_motor_measure->ecd + Gimbal.TuChuan_count *8191;
	
	
	if((Gimbal.WuangYuan.gimbal_motor_measure->ecd - Gimbal.Last_2006_WuangYuan_Ecd)>5555)
	{
		Gimbal.WuangYuan_count--;
		
		
	}else if((Gimbal.WuangYuan.gimbal_motor_measure->ecd - Gimbal.Last_2006_WuangYuan_Ecd)<-5555)
	{
		Gimbal.WuangYuan_count++;
		
	}
	Gimbal.Last_2006_WuangYuan_Ecd = Gimbal.WuangYuan.gimbal_motor_measure->ecd;
	
	Gimbal.WuangYuan_Ecd = Gimbal.WuangYuan.gimbal_motor_measure->ecd + Gimbal.WuangYuan_count *8191;
	
	
		
		
	GM6020_E_ecd = Gimbal.Yaw.gimbal_motor_measure->ecd + Quan*8191;
	GM6020_E_angle = (GM6020_E_ecd-4757)/22.752777;
	
	GM6020_speed = (Yaw.gimbal_motor_measure->speed_rpm*PI/30*0.05)+0.95*lastspeed;
	lastspeed = GM6020_speed;
		
		
	Yaw.angle  =   Message.DM_data.DM_Yaw_angle;//Message.DM_data.DM_Yaw_angle;
 	Yaw.speed  =   Message.MPU_DataZ.Speed_Z.int_16*BMI088_GYRO_2000_SEN;//Message.XRobot_Data.gyro_.z;//Message.MPU_DataZ.Speed_Z.int_16*BMI088_GYRO_2000_SEN;
	
	
	DM_Pitch.angle = -DM_Pitch.gimbal_motor_measure->POS.fdata*180/PI;//Message.XRobot_Data.eulr_.pit;//-DM_Pitch.gimbal_motor_measure->POS.fdata*180/PI;
	DM_Pitch.speed = -DM_Pitch.gimbal_motor_measure->VEl.fdata;//Message.XRobot_Data.gyro_.x;//-Message.MPU_DataXY.Speed_Y.int_16*BMI088_GYRO_2000_SEN;

	
	/*pitch重力补偿*/
  DM_Pitch.tor_set=cos(DM_Pitch.angle/180*PI)*1.043f*DM_Pitch.R;
	DM_Pitch.tor_set=constrain(DM_Pitch.tor_set,-1.0f,1.0f);
	
	
	
	
	
  Trigger.real_angle =Trigger.gimbal_motor_measure->POS.fdata/PI*180.0f;
	if(Trigger.real_angle - Trigger.Last_angle > 180)
	{
	  Trigger.cycle--;
	}
	else if (Trigger.real_angle - Trigger.Last_angle < -180)
	{
	  Trigger.cycle++;
	}
	Trigger.Last_angle = Trigger.real_angle;
	Trigger.angle = Trigger.real_angle + Trigger.cycle * 360.0f;
	
	Trigger.speed = Trigger.gimbal_motor_measure->VEl.fdata;
	
	
	
	
	/*电机转速监测*/
	Trigger.speed = Trigger.gimbal_motor_measure->VEl.fdata;
	Fric1.speed   = Fric1.gimbal_motor_measure->speed_rpm;
	Fric2.speed   = Fric2.gimbal_motor_measure->speed_rpm;
	Fric3.speed   = Fric3.gimbal_motor_measure->speed_rpm;
	Fric4.speed   = Fric4.gimbal_motor_measure->speed_rpm;
	Fric5.speed   = Fric5.gimbal_motor_measure->speed_rpm;
  Fric6.speed   = Fric6.gimbal_motor_measure->speed_rpm;
	
  /*2006电机 图传数据*/
	TuChuan1.angle = Gimbal.TuChuan_Ecd;
	TuChuan1.speed = TuChuan1.gimbal_motor_measure->speed_rpm * 0.01 + Gimbal.TuChuan_last_speed *0.99;	
	TuChuan1.last_angle = TuChuan1.gimbal_motor_measure->last_ecd;
	Gimbal.TuChuan_last_speed = TuChuan1.gimbal_motor_measure->speed_rpm ;
	
	/*2006电机 望远数据*/
	WuangYuan.angle = Gimbal.WuangYuan_Ecd;
	WuangYuan.speed = WuangYuan.gimbal_motor_measure->speed_rpm * 0.01 + Gimbal.WuangYuan_last_speed *0.99;	
	WuangYuan.last_angle = WuangYuan.gimbal_motor_measure->last_ecd;
	Gimbal.WuangYuan_last_speed = WuangYuan.gimbal_motor_measure->speed_rpm ;
	
	
	
	if(Stop_TickCount-last_time>=2.0f)			
			{
				last_time = Stop_TickCount;
				if(Gimbal.Flags.start_count_Flag){
					
					TuChuan_error = TuChuan1.gimbal_motor_measure->ecd - Gimbal.Last_2006_TuChuan_Ecd;	
					WuangYuan_error = WuangYuan.gimbal_motor_measure->ecd - Gimbal.Last_2006_WuangYuan_Ecd;
					
				}					
			}
	
	
	Data.Gear = Message.robo->game_robot_state.robot_level;
	
	if(Message.robo->game_robot_state.power_management_shooter_output == 0)
	{
		Flags.Shoot_Flag = false;
		
	}
	
	

	 if(Mode != GIMBAL_AIM)
 {
	  DM_Pitch.FollowSpeedPid.Iout=0;
	  Yaw.FollowSpeedPid.Iout=0;
	  DM_Pitch.FollowPositinPid.Iout=0;
	  Yaw.FollowPositinPid.Iout=0;
 }
	Statistic_Update(xTaskGetTickCount());
 
}
//按键和拨杆控制
void Gimbal_Ctrl::Behaviour_Mode(void)
{
	/*----------------------------------------键鼠控制---------------------------------*/
	
	if (switch_is_up(RC_Ptr->rc.s[CHANNEL_RIGHT]))
	{
		Flags.RC_Flag = false;
		//Flags.send_test_Flag =false;//谁写的
	}
	else
	{
		Flags.RC_Flag = true;
	}
	if (Flags.RC_Flag == false)
	{ 
		//按键控制
		if(RC.read_key(&RC.Key.R, single, true))
		{
			Gimbal.TUCHUAN_LOB_SHOT +=1000;
		}
		else if(RC.read_key(&RC.Key.ctrl, single, true))
		{
			Gimbal.TUCHUAN_LOB_SHOT -=1000;
		}
		
		if (RC.read_key(&RC.Key.C, single, true))
		{ 
			//无力
			Mode = GIMBAL_NO_MOVE;
		}
		if(RC.read_key(&RC.Key.V, single, true))
		{
			
			
			if(Flags.Mirror_Flag == true)
			{
				reset();
				Flags.Mirror_Flag = false;
				
			}
			else if(Flags.Mirror_Flag == false)
			{
				Flags.Mirror_Flag = true;
				
			}
			
		}
		if(RC.read_key(&RC.Key.X, single, true))
		{
			if(Mode == GIMBAL_NO_MOVE || Mode == GIMBAL_LOB_SHOT)
			{
				Mode = GIMBAL_Normal; 
			}
			
		}
		if(RC.read_key(&RC.Key.E, single, true))
		{
			if(Mode == GIMBAL_Normal)
			{
				Mode = GIMBAL_LOB_SHOT;
			}
			
			
		}
		
		
//		if (RC.read_key(&RC.Key.X, single, true) && Mode == GIMBAL_NO_MOVE)
//		{ 
//			//启动后为人为控制
//			Mode = GIMBAL_Normal;
//		}
		
		if (RC.read_key(&RC.Key.Z, single, true) && Mode != GIMBAL_NO_MOVE)
		{ 
			DM_Pitch.LOB_SHOT_last_angle +=0.05;
			//Yaw.LOB_SHOT_last_angle+=0.05;
		}
		if (RC.read_key(&RC.Key.G, single, true) && Mode != GIMBAL_NO_MOVE)
		{ 
			DM_Pitch.LOB_SHOT_last_angle -=0.05;
			//Yaw.LOB_SHOT_last_angle-=0.05;
		}
		
		
		//视觉开关
		RC.read_key(&RC.Press.R,single,&Flags.Visual_Flag);
		/*软件复位*/
//		RC.read_key(&RC.Key.B,single,&Flags.FRIC_SHOT_Flag);
//		if(Flags.FRIC_SHOT_Flag == true)
//		{
//			Data.Presets=1500;
//			
//			
//			//HAL_NVIC_SystemReset();
//		}
//		else{
//			
//			Data.Presets=0;
//			
//		}
	  //摩擦轮开关
		 RC.read_key(&RC.Key.Q,single, &Flags.Fric_Flag);
		//单发开关
		if(Flags.Fric_Flag == true)
		{
			RC.read_key(&RC.Press.L,single,&Flags.Shoot_Flag);
		}
	 if(Flags.Fric_Flag == false)
		{
			RC.Press.L.count=0;
			RC.Key.R.count=0;
		}
		/*初始化达秒电机*/
		if(RC.read_key(&RC.Key.G,single,true))
		{
			/*检测DM电机状态,使能*/
			if(DM_Pitch.gimbal_motor_measure->state!=1){
					CAN_Cmd.DM_Motor_clear_error(&CAN_Cmd.Gimbal_DM_Pitch);
					osDelay(2);//延时和阻塞2个节拍
					CAN_Cmd.DM_Motor_Enable(&CAN_Cmd.Gimbal_DM_Pitch);
			}
			if(Trigger.gimbal_motor_measure->state!=1){
					CAN_Cmd.DM_Motor_clear_error(&CAN_Cmd.DM_Trigger,0x200);
					osDelay(2);
					CAN_Cmd.DM_Motor_Enable     (&CAN_Cmd.DM_Trigger,0x200);
			}
		}	
	}

	/*----------------------------------------拨杆控制---------------------------------*/
	
	if (switch_is_down(RC_Ptr->rc.s[CHANNEL_RIGHT]))
	{
		Mode = GIMBAL_NO_MOVE;
	}
#if Gimbal_RC_CONTRAL_MODE == 0
	if (switch_is_mid(RC_Ptr->rc.s[CHANNEL_RIGHT]) && switch_is_down(RC_Ptr->rc.s[CHANNEL_LEFT]))
	{
		Mode = GIMBAL_Normal;
	}
	else if (switch_is_mid(RC_Ptr->rc.s[CHANNEL_RIGHT]) && switch_is_mid(RC_Ptr->rc.s[CHANNEL_LEFT]))
	{
		Mode = GIMBAL_Normal;
	}
	else if (switch_is_mid(RC_Ptr->rc.s[CHANNEL_RIGHT]) && switch_is_up(RC_Ptr->rc.s[CHANNEL_LEFT]))
	{
		Mode = GIMBAL_Normal;
	}
#elif Gimbal_RC_CONTRAL_MODE == 1
	if (switch_is_down(RC_Ptr->rc.s[CHANNEL_RIGHT]) && switch_is_down(RC_Ptr->rc.s[CHANNEL_LEFT]))
	{
		Flags.Visual_Flag = false;
		Flags.Fric_Flag   = false;
		Flags.Shoot_Flag  = false;
		
		reset();
		
		Mode = GIMBAL_NO_MOVE;
	}
	else if (switch_is_mid(RC_Ptr->rc.s[CHANNEL_RIGHT]) && switch_is_down(RC_Ptr->rc.s[CHANNEL_LEFT]))
	{
		
		Flags.Visual_Flag = false;
		Flags.Fric_Flag   = false;
		Flags.Shoot_Flag  = false;
		
		/*---------滑杆下拨 发送UI----------*/
		if(RC_Ptr->rc.ch[4] >=650)
		{
		Flags.send_test_Flag =true;			
		}
		else
		{
		Flags.send_test_Flag =false;	
		}
		/*--------------------------------*/
			
		Mode = GIMBAL_Normal;
		
		Flags.Mirror_Flag = false;
		
	}
	//左中 右中
	else if((switch_is_mid(RC_Ptr->rc.s[CHANNEL_RIGHT]) && switch_is_mid(RC_Ptr->rc.s[CHANNEL_LEFT])))
	{
		Flags.Visual_Flag   = true;
		Flags.Fric_Flag     = true;
		Flags.Shoot_Flag    = false;
		Flags.RC_Shoot_Flag = true;
		Trigger.vel_set     =0;
    Data.VisualR_Fric   =0;
		Mode = GIMBAL_LOB_SHOT;
		//Mode = GIMBAL_AIM;
		
		Flags.Mirror_Flag = true;
		
	}
	else if(switch_is_mid(RC_Ptr->rc.s[CHANNEL_RIGHT]) && switch_is_up(RC_Ptr->rc.s[CHANNEL_LEFT]))
	{
		Flags.Visual_Flag = true;
		Flags.Fric_Flag = true;
		if (Flags.RC_Shoot_Flag == true)
		{
			Flags.Shoot_Flag = true;
			Flags.RC_Shoot_Flag = false;
		}
		Mode = GIMBAL_LOB_SHOT;
		//Mode = GIMBAL_AIM;
		
	}
	
#endif
	       /*		左上   右上  复位*/
	else if (switch_is_up(RC_Ptr->rc.s[CHANNEL_RIGHT]) && switch_is_up(RC_Ptr->rc.s[CHANNEL_LEFT]))
	{
		HAL_NVIC_SystemReset();
	}
	Flag_Behaviour_Control();
}
int spees=0;
// 标志位行为设置
void Gimbal_Ctrl::Flag_Behaviour_Control()
{
	if (Message.robo->game_robot_state.robot_id != 0 && Message.robo->game_robot_state.current_HP == 0)
	{//死亡处理
		Mode = GIMBAL_NO_MOVE;
		Flags.Shoot_Flag = false;
		Flags.Visual_Flag = false;
		Flags.Fric_Flag = false;
		
	}
	if (Flags.Fric_Flag == true)
	{
	 Data.FricSpeedSet =Data.FricSpeed;
	 Two_Ji_SpeedSet = Two_Ji_FricSpeed;
	}
	else
	{
		Data.Presets=0;
		
		
	  Data.FricSpeedSet =0;
		Two_Ji_SpeedSet = 0;
		
		Trigger.vel_set  = 0;
	  Flags.Shoot_Flag = false;
	}

	Fric1.speed_set = RAMP_float( Data.FricSpeedSet,Fric1.speed_set,Fric_Set_Frame_Period);
  Fric2.speed_set = RAMP_float( Two_Ji_SpeedSet,Fric2.speed_set,Fric_Set_Frame_Period);//
	Fric3.speed_set = RAMP_float( Data.FricSpeedSet,Fric3.speed_set,Fric_Set_Frame_Period);
	Fric4.speed_set = RAMP_float( Two_Ji_SpeedSet,Fric4.speed_set,Fric_Set_Frame_Period);//
	Fric5.speed_set = RAMP_float( Data.FricSpeedSet,Fric5.speed_set,Fric_Set_Frame_Period);
	Fric6.speed_set = RAMP_float( Two_Ji_SpeedSet,Fric6.speed_set,Fric_Set_Frame_Period);//
	
//	Fric6.speed_set = Data.Presets;
	Data.Last_FricSpeedSet= Data.FricSpeedSet;
	
		/************************视觉模式判定****************************/
	if(Flags.Visual_Flag == true  && Data.VisualR_Goal ==true && Mode != GIMBAL_NO_MOVE)
	{
		 Mode = GIMBAL_AIM;
	}
	else if(( Data.VisualR_Goal ==false|| Flags.Visual_Flag == false) && Mode != GIMBAL_NO_MOVE && Mode != GIMBAL_LOB_SHOT)
	{
		 Mode = GIMBAL_Normal;
	}
//	if(Flags.Visual_Flag == true  && Solvetrajectory.hero_x!=0 && Mode != GIMBAL_NO_MOVE)
//	{
//		 Mode = GIMBAL_AIM;
//	}
//	else if(Solvetrajectory.hero_x  ==0  && Mode != GIMBAL_NO_MOVE)
//	{
//		 Mode = GIMBAL_Normal;
//	}
	
	/************************视觉模式判定****************************/
	//如果视觉识别距离不为 -1，Visual_true_Flag,VisualR_Goal为真，表示识别到目标
	if(Message.VisualR.distance.F!=-1)
	{
		Flags.Visual_true_Flag = true;
	}
	else
	{
		Flags.Visual_true_Flag = false;
	}
	
	
	
	/************************视觉模式开火建议*****************************/
	if(Message.VisualR.fire == true)
	{
		if(Flags.Shoot_Flag==true && Data.VisualR_Fric==0){
			Data.VisualR_Fric=1;
		}
	}
//	if( Flags.Shoot_Flag==true )
//	{
//	  if(Data.VisualR_Fric == true && flagged==0 )
//		{
//			flagged=1;
//		}
//		if(flagged==1  && Data.VisualR_Fric == false )
//		{
//			flagged=2;
//		}
//		if(flagged==2  && Data.VisualR_Fric == true)
//		{
//			flagged=3;
//		}
//	}
  /************************视觉模式开火建议******************************/
	if (Flags.Shoot_Flag == true && Data.VisualR_Fric==1 &&  Mode == GIMBAL_AIM)
	{
		
		Last_Stop_TickCount = DWT_GetTimeline_s();
		Trigger.vel_set     = 7;
		Gimbal.Data.Presets = 3000;
		//Flags.Shoot_Flag    = false;
		Data.VisualR_Fric   = 0;
		
/*------------------------------------------*/	
		
//		//Last_Stop_TickCount = DWT_GetTimeline_s();
//		Trigger.vel_set     = 6;
//		tos = 14.5;
//		Gimbal.Data.Presets = 3000;//3000;
////		if(Trigger.angle == Trigger.angle + 72.0f){
////			Flags.Shoot_Flag    = false;
////			Data.VisualR_Fric   = 0;
////			
////		}
	}
	else if(Flags.Shoot_Flag == true && (Mode == GIMBAL_Normal || Mode == GIMBAL_LOB_SHOT))
	{
		
		Last_Stop_TickCount = DWT_GetTimeline_s();
		//angle_4340_set = (Gimbal.Trigger.angle + 72)*PI/180;
		
		Trigger.vel_set     = 7;
		Gimbal.Data.Presets = 3000;
		//Flags.Shoot_Flag    = false;
		
/*------------------------------------------*/	
		
		
//		Trigger.vel_set     = 6;
//		tos = 14.5;
//		Gimbal.Data.Presets = 3000;//3000;
////		if(Trigger.angle == Trigger.angle + 72.0f){
////			
////			Flags.Shoot_Flag    = false;
////			Data.VisualR_Fric   = 0;
////			
////		}
////		Trigger.pos_set = Trigger.angle + 80.0f;
////		
////		
////		//Last_Stop_TickCount = DWT_GetTimeline_s();
////		//Trigger.vel_set     = 5;
////		Flags.Shoot_Flag    = false;
	}
	

	

	
	if(Key_count==2 && Flags.Shoot_Flag == true)
	{
		Flags.Shoot_Flag = false;
		
		Gimbal.Data.Presets=0;
		
		Trigger.vel_set = 0;
		
		Key_count=0;
		
		//Flags.Shoot_Flag = false;
	}

	
	
//	if(Key_count==3){
//		Flags.Shoot_Flag    = false;
//		Trigger.vel_set  = 0;
//		Key_count=0;
//	}
	
  if(Flags.Mirror_Flag == true)
	{
		//图传
		
		
		
		
		
		Gimbal.TuChuan1.angle_set = Gimbal.TUCHUAN_LOB_SHOT ;    //吊射图传角度
		Gimbal.WuangYuan.angle_set = Gimbal.WUANGYUAN_LOB_SHOT;
		
		__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3,Data.Mirror_open);
		__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4,Data.Image_Hanging_Shot);//望远镜
		
		
		
	}
	else
	{
		
		if(!Gimbal.Flags.TuChuan_reset_end_Flag)
		{	
			Gimbal.Flags.TuChuan_reset_Flag = true;
	  }
		
		__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3,Data.Mirror_close);//望远镜
		__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4,Data.Image_Normal);//图传
		
	}
	
	if (Mode == Last_Mode)
	{
		return;
	}
	else if (Mode != Last_Mode)
	{
		Yaw.angle_set     = Yaw.angle;
		Trigger.pos_set = Trigger.angle;
		DM_Pitch.pos_set  = DM_Pitch.angle;
	  Last_Mode = Mode;
		DM_Trigger_flag = 1;
		
		
		Quan = 0;
		GM6020_last_angle = 0;
		
	}
}

// 摇杆和鼠标输入
void Gimbal_Ctrl::RC_to_Control(fp32 *yaw_set, fp32 *pitch_set)
{
	if (IsInvalid(*yaw_set) || IsInvalid(*pitch_set))
	{
		return;
	}
	// 遥控器原始通道值
	int16_t yaw_channel, pitch_channel;
	fp32 yaw_set_channel, pitch_set_channel;

	if (Flags.RC_Flag == true)
	{
		// 将遥控器的数据处理死区
		rc_deadline_limit(RC_Ptr->rc.ch[YawChannel],   yaw_channel,   RC_DEADLINE);
		rc_deadline_limit(RC_Ptr->rc.ch[PitchChannel], pitch_channel, RC_DEADLINE);

		yaw_set_channel   = -(yaw_channel   * Yaw_RC_SEN);
		pitch_set_channel =  (pitch_channel * Pitch_RC_SEN);
	}
	else if (Flags.RC_Flag == false)
	{
		yaw_set_channel   = -(RC_Ptr->mouse.x * Yaw_Mouse_SEN);
		pitch_set_channel =  -(RC_Ptr->mouse.y * Pitch_Mouse_SEN);
	}
	*yaw_set   =  yaw_set_channel;
	*pitch_set =  pitch_set_channel;
	
}

// 云台控制设定
void Gimbal_Ctrl::Behaviour_Control(fp32 *yaw_set, fp32 *pitch_set)
{
	if (Mode == GIMBAL_NO_MOVE)
	{
		*yaw_set   = 0;
		*pitch_set = 0;
	}
	else if (Mode == GIMBAL_Normal)
	{
		RC_to_Control(yaw_set, pitch_set);
	}
	else if (Mode == GIMBAL_AIM || Mode == GIMBAL_ENERGY)
	{
		*yaw_set  = relative_angle(Message.DM_data.DM_Yaw,Message.VisualR.yaw.F);
		*pitch_set= Message.VisualR.pitch.F;
//	    *yaw_set  =relative_angle(Message.DM_data.DM_Yaw,Solvetrajectory.Yaw);
//	    *pitch_set=Solvetrajectory.Pitch;
	}
}
//电机设定量控制
void Gimbal_Ctrl::Control(void)
{
	fp32 yaw_set;
	fp32 pitch_set;

	Behaviour_Control(&yaw_set, &pitch_set);//获取yaw和pitch的变化量——来自控制器输入
	if (Mode == GIMBAL_NO_MOVE)
	{
		yaw_set = 0;
		pitch_set = 0;
		Fric1.speed_set = 0.0f;
		Fric2.speed_set = 0.0f;
		Fric3.speed_set = 0.0f;
		Fric4.speed_set = 0.0f;
		Fric5.speed_set = 0.0f;
		Fric6.speed_set = 0.0f;
	}
	else if (Mode == GIMBAL_Normal)
	{
		Yaw.angle_set    += yaw_set;
		DM_Pitch.pos_set += pitch_set;
	}
	else if (Mode == GIMBAL_AIM || Mode == GIMBAL_ENERGY)
	{
		Yaw.angle_set    = yaw_set;
		DM_Pitch.pos_set = pitch_set;
	}
	//最值控制
	DM_Pitch.pos_set = constrain(DM_Pitch.pos_set,Data.pitch_min_angle,Data.pitch_max_angle);
}

// 云台控制PID运算
void Gimbal_Ctrl::Control_loop(void)
{
	fp32 YAW_out   = 0;
	fp32 PITCH_out = 0;

	if (Mode == GIMBAL_Normal)
	{
		
		PID_CT.Calc(&Yaw.PositinPid,Yaw.angle,Yaw.angle_set);//yaw轴位置控制 现在位置，目标位置
		PID_CT.Calc(&Yaw.SpeedPid,  Yaw.speed,Yaw.PositinPid.Output);//这里是速度目标值是位置pid的结果，双环控制
	
		PID_CT.Calc(&DM_Pitch.PositinPid,DM_Pitch.angle,DM_Pitch.pos_set);//同上，pitch轴位置控制
    	PID_CT.Calc(&DM_Pitch.SpeedPid,  DM_Pitch.speed,DM_Pitch.PositinPid.Output);
		
		YAW_out   = Yaw.SpeedPid.Output;
		PITCH_out = DM_Pitch.SpeedPid.Output;
		
		Yaw.LOB_SHOT_last_angle = GM6020_E_angle;//更新最近角度
		DM_Pitch.LOB_SHOT_last_angle = DM_Pitch.pos_set;//更新最近角度
		
	}
	else if (Mode == GIMBAL_LOB_SHOT)   //吊射模式
	{	
		PID_CT.Calc(&Yaw.PositinPid_GIMBAL_LOB_SHOT, GM6020_E_angle,Yaw.LOB_SHOT_last_angle);
		PID_CT.Calc(&Yaw.SpeedPid_GIMBAL_LOB_SHOT,  GM6020_speed,Yaw.PositinPid_GIMBAL_LOB_SHOT.Output);
	
		PID_CT.Calc(&DM_Pitch.PositinPid,DM_Pitch.angle,DM_Pitch.LOB_SHOT_last_angle);
    	PID_CT.Calc(&DM_Pitch.SpeedPid,  DM_Pitch.speed,DM_Pitch.PositinPid.Output);
		
		YAW_out   = Yaw.SpeedPid_GIMBAL_LOB_SHOT.Output;
		PITCH_out = DM_Pitch.SpeedPid.Output;
		//同normal
	}
	else if (Mode == GIMBAL_AIM)//这是什么模式？——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
	{     
		PID_CT.Calc(&Yaw.FollowPositinPid,        0, Yaw.angle_set);
		PID_CT.Calc(&Yaw.FollowSpeedPid,  Yaw.speed, Yaw.FollowPositinPid.Output);

		PID_CT.Calc(&DM_Pitch.FollowPositinPid,DM_Pitch.angle, DM_Pitch.pos_set);
		PID_CT.Calc(&DM_Pitch.FollowSpeedPid,  DM_Pitch.speed, DM_Pitch.FollowPositinPid.Output);

		YAW_out     = Yaw.FollowSpeedPid.Output;
		PITCH_out   = DM_Pitch.FollowSpeedPid.Output;
	}
	PITCH_out=constrain(PITCH_out,-4.5,4);
	
	//各个摩擦轮的控制
	PID.Calc(&Fric1.SpeedPid, Fric1.speed, Fric1.speed_set);
	PID.Calc(&Fric2.SpeedPid, Fric2.speed, Fric2.speed_set);
	PID.Calc(&Fric3.SpeedPid, Fric3.speed, Fric3.speed_set);
	PID.Calc(&Fric4.SpeedPid, Fric4.speed, Fric4.speed_set);/*机械装配问题，改方向正常发弹*/
	PID.Calc(&Fric5.SpeedPid, Fric5.speed, Fric5.speed_set);
	PID.Calc(&Fric6.SpeedPid, Fric6.speed, Fric6.speed_set);
	
	

	if(!Gimbal.Flags.TuChuan_reset_Flag)
	{
		PID_CT.Calc(&Gimbal.TuChuan1.PositinPid, Gimbal.TuChuan1.angle,Gimbal.TuChuan1.angle_set);
		PID_CT.Calc(&Gimbal.TuChuan1.SpeedPid,  Gimbal.TuChuan1.speed,Gimbal.TuChuan1.PositinPid.Output);

		//望远镜
		PID_CT.Calc(&Gimbal.WuangYuan.PositinPid, Gimbal.WuangYuan.angle,Gimbal.WuangYuan.angle_set);
		PID_CT.Calc(&Gimbal.WuangYuan.SpeedPid,  Gimbal.WuangYuan.speed,Gimbal.WuangYuan.PositinPid.Output);




		TuChuan1.give_current = Gimbal.TuChuan1.SpeedPid.Output;
		WuangYuan.give_current = Gimbal.WuangYuan.SpeedPid.Output;
		
		
	}
	else
	{
		//未出现标志时，保持望远镜的位置
		PID_CT.Calc(&Gimbal.WuangYuan.SpeedPid0,  Gimbal.WuangYuan.speed,vel);
	  Gimbal.WuangYuan.give_current = Gimbal.WuangYuan.SpeedPid0.Output;
		
		
		TuChuan1.give_current =  2000;	
		//WuangYuan.give_current = 2000;
		//WuangYuan.give_current = Gimbal.WuangYuan.SpeedPid.Output;
		
		Gimbal.Flags.start_count_Flag = true;
		
		if(TuChuan_error == 0 && WuangYuan_error == 0)
		{
			//Gimbal.TuChuan_Ecd = Gimbal.TuChuan1.gimbal_motor_measure->ecd; 好像没用
			
			Gimbal.TuChuan1.angle_set = Gimbal.TuChuan_Ecd;
			Gimbal.WuangYuan.angle_set = Gimbal.WuangYuan_Ecd;
			
			Gimbal.TUCHUAN_LOB_SHOT = Gimbal.TuChuan_Ecd -15265;
			Gimbal.WUANGYUAN_LOB_SHOT = Gimbal.WuangYuan_Ecd-60000;
			
			
			Gimbal.Flags.TuChuan_reset_Flag = false;
			Gimbal.Flags.TuChuan_reset_end_Flag = true;
			Gimbal.Flags.start_count_Flag = false;
		}
		
	}
	

	
	
//PID.Calc(&Trigger.PositinPid, Trigger.angle,Trigger.angle_set);
//PID.Calc(&Trigger.SpeedPid,   Trigger.speed,Trigger.vel_set);
	
	
//	PID_CT.Calc(&Trigger.PositinPid,Trigger.angle,Trigger.pos_set);
//  PID_CT.Calc(&Trigger.SpeedPid,  Trigger.speed,Trigger.PositinPid.Output);
	
	
	

	//将pid的结果赋值给发送值	
	Yaw.give_current     = YAW_out;
	DM_Pitch.vel_set     = PITCH_out;
	Fric1.give_current   = Fric1.SpeedPid.out;
	Fric2.give_current   = Fric2.SpeedPid.out;
	Fric3.give_current   = Fric3.SpeedPid.out;
	Fric4.give_current   = Fric4.SpeedPid.out;
	Fric5.give_current   = Fric5.SpeedPid.out;
	Fric6.give_current   = Fric6.SpeedPid.out;
	//Gimbal.Trigger.vel_set = Trigger.SpeedPid.Output;
	


}

//视觉过零处理 范围±180
fp32 Gimbal_Ctrl::relative_angle(fp32 angle, fp32 Visual_angle)
{
	if(Visual_angle-angle > 180)
	{
		Visual_angle -= 360;
	}
	else if(Visual_angle-angle < -180)
	{
	  Visual_angle += 360;
  };
	return Visual_angle-angle;
}

//规整ANGLE后转化成角度ANGLE，范围±180
fp32 Gimbal_Ctrl::Gyro_relative_angle_to_angle(fp32 angle, fp32 offset_angle)
{

	fp32 relative_ecd = angle - offset_angle;
	if (relative_ecd > 180){
		relative_ecd -= 360;
	}
	else if (relative_ecd <-180 )
	{
		relative_ecd += 360;
	}
	
	return relative_ecd;
}

// 规整ECD后转化成角度DEG，范围±180
fp32 Gimbal_Ctrl::motor_relative_ECD_to_angle(uint16_t angle, uint16_t offset_ecd)
{
	fp32 relative_angle;
	int32_t relative_ecd = angle - offset_ecd;
	if (relative_ecd > Half_ecd_range)
	{
		relative_ecd -= ecd_range;
	}
	else if (relative_ecd < -Half_ecd_range)
	{
		relative_ecd += ecd_range;
	}
	relative_angle = relative_ecd * ECD_TO_DEG;
	return relative_angle;
}


fp32  RAMP_float(fp32 final, fp32 now, fp32 ramp)
{
	  fp32  buffer = 0;
	  buffer = final - now;
	
		if (buffer > 0){
				if (buffer > ramp){  
						now += ramp;
				}   
				else{
						now += buffer;
				}
		}
		else{
				if (buffer < -ramp){
						now += -ramp;
				}
				else{
						now += buffer;
				}
		}	
		return now;
}

void rc_key_v_fresh_Gimbal(RC_ctrl_t *RC)
{
	Gimbal.RC.rc_key_v_set(RC);
}

Gimbal_Ctrl *get_gimbal_ctrl_pointer(void)
{
	return &Gimbal;
}

void reset()
{
	Gimbal.Flags.TuChuan_reset_Flag = true;
	Gimbal.Flags.TuChuan_reset_end_Flag = false;
	Gimbal.Flags.start_count_Flag = false;
	TuChuan_error = 1;
	WuangYuan_error = 1;
	
}
