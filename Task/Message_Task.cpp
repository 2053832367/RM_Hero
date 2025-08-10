#include "Message_Task.h"
#include "tasks.h"
#include "app_vofa.h"


Message_Ctrl Message;

uint8_t qqq;

void Message_Task(void *pvParameters)
{
	
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

void Referee_Rx_Task(void *pvParameters)
{
	/* USER CODE BEGIN StartDefaultTask */
	static ID_Data_t Referee_Rx_Data;
  /* Infinite loop */
  for(;;)
  {		
		if(xQueueReceive(Referee_Rx_Queue, &Referee_Rx_Data.Data_Ptr, portMAX_DELAY))
		{
			referee_data_solve( &(((uint8_t *)Referee_Rx_Data.Data_Ptr)[1]) );
			//Message.FricSpeed_pid();
			Guard.Feed(RefereeData);
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
			rc_key_v_fresh_Gimbal(Message.RC_Ptr);
			rc_key_v_fresh_Chassis(Message.RC_Ptr);
			Guard.Feed(RCData);
		}
  }
  /* USER CODE END StartDefaultTask */
}

void Message_Ctrl::Init()
{	
	Message.RC_Ptr = &RC_ctrl;
	CAN_ALL_Init();//can总线初始化
	Prefence_Init();//给各个ID相应赋值
	Serial_ALL_Init();//串口初始化
	robo = get_robo_data_Point();
}

void Message_Ctrl::Serialx_Hook(uint8_t *Rx_Message, Serialctrl *Serialx_Ctrl)
{
	if(Serialx_Ctrl == &VISUAL_SERIAL)
	{
		Visual_Serial_Hook(Rx_Message);
	}
	if(Serialx_Ctrl == &JUDGE_SERIAL)
	{
		//Radar_Serial_Hook(Rx_Message);
		xQueueSend(Referee_Rx_Queue, &Rx_Message, 0);
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
	uint8_t dect=0;
  uint8_t Len=0;
  while(dect!=0xA5)
  {
			dect=CAN1_Ctrl.read();
			Len=CAN1_Ctrl.available();
			if(Len <= 12)
			return;
  }
  Len=CAN1_Ctrl.available();
  if(Len <= 12)
    return;
  dect=CAN1_Ctrl.read();
  if(dect!=0xA6)
    return;
  if(Len <= 11)
    return;
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
//		case CAN_DJI_Motor5_ID://yaw
//		{
//			static uint8_t i = 0;
//			//处理电机ID号
//			i = Rx_Data.StdId.u32 - CAN_DJI_Motor5_ID;
//			//处理电机数据宏函数
//			MA_get_motor_measure(CAN_Cmd.Bottom.GetData(i), Rx_Data.Data);
//			break;
//		}
			case DM_GimbalR_ID:
		{
			Get_DM_Motor_Measure(CAN_Cmd.Gimbal_DM_Pitch.GetData(),CAN_Cmd.Gimbal_DM_Pitch.LIMIT,Rx_Data.Data);
			break;
		}
			case DM_TriggerR_ID:
		{
			Get_DM_Motor_Measure(CAN_Cmd.DM_Trigger.GetData(),CAN_Cmd.DM_Trigger.LIMIT,Rx_Data.Data);
			break;
		}
		
		case CAN_DJI_Motor6_ID://TuChuan
		case CAN_DJI_Motor7_ID://WuangYuan
		{
			static uint8_t i = 0;
			//处理电机ID号
			i = Rx_Data.StdId.u32 - CAN_DJI_Motor6_ID;
			//处理电机数据宏函数
			MA_get_motor_measure(CAN_Cmd.TuChuan1.GetData(i), Rx_Data.Data);
			break;
		}
		
//      case IMU_ID + 1:
//			{
//        XRobot_Data.gyro_.x = (float)((int16_t)(Rx_Data.Data[0]+(Rx_Data.Data[1]<<8))) / 32767.0f * 34.90658502f;
//        XRobot_Data.gyro_.y = (float)((int16_t)(Rx_Data.Data[2]+(Rx_Data.Data[3]<<8))) / 32767.0f * 34.90658502f;
//        XRobot_Data.gyro_.z = (float)((int16_t)(Rx_Data.Data[4]+(Rx_Data.Data[5]<<8))) / 32767.0f * 34.90658502f;
//        break;
//			}
//      case IMU_ID + 3:
//			{
//				XRobot_Data.eulr_.pit = (float)(Rx_Data.Data[0]+(Rx_Data.Data[1]<<8)) / 32767.0f *2*PI;
//        XRobot_Data.eulr_.rol = (float)(Rx_Data.Data[2]+(Rx_Data.Data[3]<<8)) / 32767.0f *2*PI;
//        XRobot_Data.eulr_.yaw = (float)(Rx_Data.Data[4]+(Rx_Data.Data[5]<<8)) / 32767.0f *2*PI;
//				DM_data.DM_Yaw = (XRobot_Data.eulr_.yaw-PI)*180/PI;//MPU_DataZ.AngleZ.int_16*FP32_MPU_RAD;
//				//DM_data.DM_Roll=MPU_DataXY.AngleX.int_16*FP32_MPU_RAD;
//				
//				DM_data.DM_Error_angle =DM_data.DM_Yaw - DM_data.Last_YAW_angle;
//				if(DM_data.DM_Error_angle > 270.0f){ 
//				  DM_data.DM_Yaw_cycle--;
//				 }
//				 if(DM_data.DM_Error_angle < -270.0f){
//					DM_data.DM_Yaw_cycle++;
//				 }
//				DM_data.DM_Yaw_angle   =DM_data.DM_Yaw + DM_data.DM_Yaw_cycle * 360.0f;
//				DM_data.Last_YAW_angle =DM_data.DM_Yaw;				
//        break;
//				
//			}
//		case CAN_CAP_GET_ID://超级电容
//		{
//			SuperCapR.situation = (uint8_t)(Rx_Data.Data[0]);
//			SuperCapR.mode = (uint8_t)(Rx_Data.Data[1]);
//			SuperCapR.power = (float)((uint16_t)((Rx_Data.Data[2]) | (Rx_Data.Data[3]) << 8)) * 0.1f;
//			SuperCapR.energy = (uint8_t)(Rx_Data.Data[4]);
//			SuperCapR.power_limit = (uint8_t)(Rx_Data.Data[5]);
//			SuperCapR.errorcode = (uint8_t)(Rx_Data.Data[6]);
//		//SuperCapR.enable = (uint8_t)((Rx_Message)->Data.Data[7]);
//		//SuperCapR.enable = (uint8_t)((Rx_Message)->Data.Data[8]);
//			Guard.Feed(SupercapData);
//			break;
//		}
		default:
		{
			break;
		}
		
	}
}

void Message_Ctrl::CAN2_Process(CanRxMsg *Rx_Message)
{
	CanRxMsg Rx_Data;
	uint8_t dect=0;
  uint8_t Len=0;
  while(dect!=0xA5)
  {
				dect=CAN2_Ctrl.read();
				Len=CAN2_Ctrl.available();
				if(Len <= 12)
				return;
  }
  Len=CAN2_Ctrl.available();
  if(Len <= 12)
    return;
  dect=CAN2_Ctrl.read();
  if(dect!=0xA6)
    return;
  if(Len <= 11)
    return;
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
//			 case DM_GimbalR_ID:
//		{
//			Get_DM_Motor_Measure(CAN_Cmd.Gimbal_DM_Pitch.GetData(),CAN_Cmd.Gimbal_DM_Pitch.LIMIT,Rx_Data.Data);
//			break;
//		}
			case CAN_DJI_Motor1_ID:
			case CAN_DJI_Motor2_ID:
			case CAN_DJI_Motor3_ID:
			case CAN_DJI_Motor4_ID://四个麦轮
		{
			static uint8_t i = 0;
			//处理电机ID号
			i = Rx_Data.StdId.u32 - CAN_DJI_Motor1_ID;
			//处理电机数据宏函数,这里是底盘电机
			MA_get_motor_measure(CAN_Cmd.Chassis.GetData(i), Rx_Data.Data);
			break;
		}
			case CAN_DJI_Motor5_ID://yaw
		{
			static uint8_t i = 0;
			//处理电机ID号
			i = Rx_Data.StdId.u32 - CAN_DJI_Motor5_ID;
			//处理电机数据宏函数
			MA_get_motor_measure(CAN_Cmd.Bottom.GetData(i), Rx_Data.Data);
			break;
		}
			case CAN_CAP_GET_ID://超级电容
		{
			SuperCapR.situation = (uint8_t)(Rx_Data.Data[0]);
			SuperCapR.mode = (uint8_t)(Rx_Data.Data[1]);
			SuperCapR.power = (float)((uint16_t)((Rx_Data.Data[2]) | (Rx_Data.Data[3]) << 8)) * 0.1f;
			SuperCapR.energy = (uint8_t)(Rx_Data.Data[4]);
			SuperCapR.power_limit = (uint8_t)(Rx_Data.Data[5]);
			SuperCapR.errorcode = (uint8_t)(Rx_Data.Data[6]);
		//SuperCapR.enable = (uint8_t)((Rx_Message)->Data.Data[7]);
		//SuperCapR.enable = (uint8_t)((Rx_Message)->Data.Data[8]);
			Guard.Feed(SupercapData);
			break;
		}
		default:
		{
			break;
		}
	}
}
void Message_Ctrl::CAN3_Process(CanRxMsg *Rx_Message)
{
	CanRxMsg Rx_Data;
	uint8_t dect=0;
  uint8_t Len=0;
  while(dect!=0xA5)
  {
			dect=CAN3_Ctrl.read();
			Len=CAN3_Ctrl.available();
			if(Len <= 12)
			return;
  }
  Len=CAN3_Ctrl.available();
  if(Len <= 12)
    return;
  dect=CAN3_Ctrl.read();
  if(dect!=0xA6)
    return;
  if(Len <= 11)
    return;
	
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
		case DM_GimbalR_ID:
		{
			
			Get_DM_Motor_Measure(CAN_Cmd.Gimbal_DM_Pitch.GetData(),CAN_Cmd.Gimbal_DM_Pitch.LIMIT,Rx_Data.Data);
			break;
		}
			case CAN_DJI_Motor1_ID:
			case CAN_DJI_Motor2_ID:
			case CAN_DJI_Motor3_ID:
			{
				static uint8_t i = 0;
				//处理电机ID号
				i = Rx_Data.StdId.u32 - CAN_DJI_Motor1_ID;
				//处理电机数据宏函数
				MA_get_motor_measure(CAN_Cmd.Fric.GetData(i), Rx_Data.Data);
				break;
			}
				case CAN_DJI_Motor4_ID:
				case CAN_DJI_Motor5_ID:	
				case CAN_DJI_Motor6_ID:		
			{
				static uint8_t i = 0;
				//处理电机ID号
				i = Rx_Data.StdId.u32 - CAN_DJI_Motor4_ID;
				//处理电机数据宏函数
				MA_get_motor_measure(CAN_Cmd.Fric02.GetData(i), Rx_Data.Data);
				break;
			}
		default:
		{
			break;
		}
	}
}

void Message_Ctrl::Visual_Serial_Hook(uint8_t *Rx_Message)
{
  VisualR.Len = Rx_Message[0];
	VisualR.fire= Rx_Message[2];
	for(uint8_t i=0 ;i<4;i++)
	{
		VisualR.pitch.I[i]=Rx_Message[3+i];
		VisualR.  yaw.I[i]=Rx_Message[7+i];
		VisualR.distance.I[i]=Rx_Message[11+i];
	}

	/*识别到目标*/
	if(VisualR.distance.F != -1){	
		Gimbal.Data.VisualR_Goal=true; 
	}
	else{
		Gimbal.Data.VisualR_Goal=false; 
	}

	VisualR.check_byte =Rx_Message[14];
	
}
void Message_Ctrl::Gyro_Serial_Hook(uint8_t *Rx_Message)
{   
	 uint8_t len = Rx_Message[0];
	if(Rx_Message[1]!=0xAA){
	 return;
	}
	else{
		if(Rx_Message[2]==0xA5)
		{
			MPU_DataZ.HHH=Rx_Message[1];
			MPU_DataZ.KEY=Rx_Message[2];
			MPU_DataZ.AngleZ.uint_8[0] = Rx_Message[3];
			MPU_DataZ.AngleZ.uint_8[1] = Rx_Message[4];
			MPU_DataZ.Speed_Z.uint_8[0]= Rx_Message[5];
			MPU_DataZ.Speed_Z.uint_8[1]= Rx_Message[6];
			MPU_DataZ.Acce_Z.uint_8[0] = Rx_Message[7];
			MPU_DataZ.Acce_Z.uint_8[1] = Rx_Message[8];
			MPU_DataZ.Acce_Y.uint_8[0] = Rx_Message[9];
			MPU_DataZ.Acce_Y.uint_8[1] = Rx_Message[10];
			
		}
		if(Rx_Message[2]==0xA6)
		{
			MPU_DataXY.HHH=Rx_Message[1];
			MPU_DataXY.KEY=Rx_Message[2];
			MPU_DataXY.AngleX.uint_8[0] =Rx_Message[3];
			MPU_DataXY.AngleX.uint_8[1] =Rx_Message[4];
			MPU_DataXY.Speed_X.uint_8[0]=Rx_Message[5];
			MPU_DataXY.Speed_X.uint_8[1]=Rx_Message[6];
			MPU_DataXY.AngleY.uint_8[0] =Rx_Message[7];
			MPU_DataXY.AngleY.uint_8[1] =Rx_Message[8];
			MPU_DataXY.Speed_Y.uint_8[0]=Rx_Message[9];
			MPU_DataXY.Speed_Y.uint_8[1]=Rx_Message[10];
		}
		
		DM_data.DM_Yaw = MPU_DataZ.AngleZ.int_16*FP32_MPU_RAD;
		DM_data.DM_Roll=MPU_DataXY.AngleX.int_16*FP32_MPU_RAD;
		
	  DM_data.DM_Error_angle =MPU_DataZ.AngleZ.int_16*FP32_MPU_RAD - DM_data.Last_YAW_angle;
	  if(DM_data.DM_Error_angle > 270.0f){ 
		  DM_data.DM_Yaw_cycle--;
		 }
	   if(DM_data.DM_Error_angle < -270.0f){
			DM_data.DM_Yaw_cycle++;
		 }
	  DM_data.DM_Yaw_angle   =MPU_DataZ.AngleZ.int_16*FP32_MPU_RAD + DM_data.DM_Yaw_cycle * 360.0f;
	  DM_data.Last_YAW_angle =MPU_DataZ.AngleZ.int_16*FP32_MPU_RAD;
	}
//	  uint8_t len = Rx_Message[0];	
////   if(VerifyData(&Rx_Message[1],Rx_Message[0]))
////	{
//		for(uint8_t i=0 ;i<4;i++)
//		{
//		    XRobot_Receive_Data.time.uint_8[i]=Rx_Message[3+i];
//		    XRobot_Receive_Data.quat_q0.I[i]=Rx_Message[7+i];
//		    XRobot_Receive_Data.quat_q1.I[i]=Rx_Message[11+i];
//		    XRobot_Receive_Data.quat_q2.I[i]=Rx_Message[15+i];
//		    XRobot_Receive_Data.quat_q3.I[i]=Rx_Message[19+i];
//				XRobot_Receive_Data.gyro_X.I[i] =Rx_Message[23+i];
//		    XRobot_Receive_Data.gyro_Y.I[i] =Rx_Message[27+i];
//		    XRobot_Receive_Data.gyro_Z.I[i] =Rx_Message[31+i];
//		    XRobot_Receive_Data.accl_X.I[i] =Rx_Message[35+i];
//		    XRobot_Receive_Data.accl_Y.I[i] =Rx_Message[39+i];
//		    XRobot_Receive_Data.accl_Z.I[i] =Rx_Message[43+i];
//		    XRobot_Receive_Data.eulr_yaw.I[i] =Rx_Message[47+i];
//		    XRobot_Receive_Data.eulr_pit.I[i] =Rx_Message[51+i];
//		    XRobot_Receive_Data.eulr_rol.I[i] =Rx_Message[55+i];
//		} 
//		    XRobot_Data.crc8=Rx_Message[59];
//		
//		    XRobot_Data.eulr_.pit = XRobot_Receive_Data.eulr_pit.F/PI/2*360.0f;
//		    XRobot_Data.eulr_.yaw = XRobot_Receive_Data.eulr_yaw.F/PI/2*360.0f;
//		    XRobot_Data.eulr_.rol = XRobot_Receive_Data.eulr_rol.F/PI/2*360.0f;
//		    XRobot_Data.gyro_.x   = XRobot_Receive_Data.gyro_X.F;
//		    XRobot_Data.gyro_.z   = XRobot_Receive_Data.gyro_Z.F;
////	}
//	      XRobot_Data.eulr_.pit =  (fmod(XRobot_Data.eulr_.pit +180.0f,360.0f)-180.0f);
//	      XRobot_Data.eulr_.yaw = (fmod(XRobot_Data.eulr_.yaw +180.0f,360.0f)-180.0f);
//	      XRobot_Data.eulr_.rol = (fmod(XRobot_Data.eulr_.rol +180.0f,360.0f)-180.0f);
//		    DM_data.DM_Yaw = XRobot_Data.eulr_.yaw;
//		
//	
//	      DM_data.DM_Error_angle = XRobot_Data.eulr_.yaw - DM_data.Last_YAW_angle;
//        if(DM_data.DM_Error_angle > 270.0f){ 
//            DM_data.DM_Yaw_cycle--;
//        }
//        if(DM_data.DM_Error_angle < -270.0f){
//            DM_data.DM_Yaw_cycle++;
//        }
//        DM_data.DM_Yaw_angle      =XRobot_Data.eulr_.yaw + DM_data.DM_Yaw_cycle * 360.0f;
//        DM_data.Last_YAW_angle = XRobot_Data.eulr_.yaw;
	

      
}

void Message_Ctrl::Radar_Serial_Hook(uint8_t *Rx_Message)
{
	uint8_t len = Rx_Message[0];
	
	
	X.I[0]=Rx_Message[8];
	X.I[1]=Rx_Message[9];
	Y.I[0]=Rx_Message[10];
	Y.I[1]=Rx_Message[11];
	
	Solvetrajectory.hero_x=(X.I_16)/100;
	Solvetrajectory.aim_y =(Y.I_16)/100;
	
	
}
RC_ctrl_t *get_remote_control_point(void)
{
    return &RC_ctrl;
}

Message_Ctrl *get_message_ctrl_pointer(void)
{
	return &Message;
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
//按键长按赋值q
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


uint8_t CalculateCRC8(const uint8_t *buf, size_t len, uint8_t crc) {
  while (len-- > 0) {
    crc = CRC8_TAB[(crc ^ *buf++) & 0xff];
  }
  return crc;
}

bool VerifyData(const uint8_t *buf, size_t len) {
  if (len < 2) {
    return false;
  }

  uint8_t expected = CalculateCRC8(buf, len -1, 0xff);
  return expected == buf[len-1];
}
















