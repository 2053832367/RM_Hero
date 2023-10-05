#include "Message_Task.h"
#include "tasks.h"

Message_Ctrl Message;

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
			Message.CAN1_Process((uint32_t *)CAN1_Rx_Data.Data_Ptr);
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
			Message.CAN2_Process((uint32_t *)CAN2_Rx_Data.Data_Ptr);
			Guard.Feed(CanData2);
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
			case SerialData2:
			Message.Serialx_Hook((uint8_t *)Serial_Rx_Data.Data_Ptr, &Serial2_Ctrl);
			break;
			case SerialData4:
			Message.Serialx_Hook((uint8_t *)Serial_Rx_Data.Data_Ptr, &Serial4_Ctrl);
			break;
			case SerialData5:
			Message.Serialx_Hook((uint8_t *)Serial_Rx_Data.Data_Ptr, &Serial5_Ctrl);
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
		if(xQueueReceive(Referee_Rx_Queue, &Referee_Rx_Data, portMAX_DELAY))
		{
			referee_data_solve( &(((uint8_t *)Referee_Rx_Data.Data_Ptr)[1]) );
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
		if(xQueueReceive(DR16_Rx_Queue, &DR16_Rx_Data, portMAX_DELAY))
		{
			sbus_to_rc(&( ((uint8_t *)(DR16_Rx_Data.Data_Ptr))[1]) ,&(Message.rc_ctrl));
			rc_key_v_fresh((RC_ctrl_t *)&(Message.rc_ctrl));
			Guard.Feed(RCData);
		}
  }
  /* USER CODE END StartDefaultTask */
}

void Message_Ctrl::Init()
{	
	CAN_ALL_Init();
	Prefence_Init();
	Serial_ALL_Init();
	robo = get_robo_data_Point();
}

void Message_Ctrl::Serialx_Hook(uint8_t *Rx_Message, Serialctrl *Serialx_Ctrl)
{
	if(Serialx_Ctrl == &JUDGE_SERIAL)
	{
		xQueueSend(Referee_Rx_Queue, &Rx_Message, 0);
	}
	if(Serialx_Ctrl == &GIMBAL_SERIAL)
	{
		Gimbal_Serial_Hook(Rx_Message);
	}
	if(Serialx_Ctrl == &DR16_SERIAL)
	{
		xQueueSend(DR16_Rx_Queue, &Rx_Message, 0);
	}
}

void Message_Ctrl::Gimbal_Serial_Hook(uint8_t *Rx_Message)
{
	uint8_t len = Rx_Message[0];
	if(Verify_CRC8_Check_Sum(&Rx_Message[1], Rx_Message[0]))
	{
		ecd_data.s[0] = Rx_Message[3];
		ecd_data.s[1] = Rx_Message[4];
		GimbalR.ECD = -motor_ecd_to_relative_ecd(ecd_data.d, Gimbal_Motor_Yaw_Offset_ECD);
		GimbalR.goal = Rx_Message[5];
	}
}

void Message_Ctrl::CAN1_Process(uint32_t *Rx_Message)
{
	uint8_t Rx_Date[8];
	for(uint8_t x=0;x<8;x++)
	{
					Rx_Date[x]=CAN1_Ctrl.read();
	}
	switch(*Rx_Message)
	{
	case CAN_CAP_GET_ID:
	{
		SuperCapR.situation = (uint8_t)(Rx_Date[0]);
		SuperCapR.mode = (uint8_t)(Rx_Date[1]);
		SuperCapR.power = (float)((uint16_t)((Rx_Date[2]) | (Rx_Date[3]) << 8)) * 0.1f;
		SuperCapR.energy = (uint8_t)(Rx_Date[4]);
		SuperCapR.power_limit = (uint8_t)(Rx_Date[5]);
		SuperCapR.errorcode = (uint8_t)(Rx_Date[6]);
		// SuperCapR.enable = (uint8_t)((Rx_Message)->Data[7]);
		// SuperCapR.enable = (uint8_t)((Rx_Message)->Data[8]);
		Guard.Feed(SupercapData);
		break;
	}
#ifdef useMecanum
	case CAN_YAW_MOTOR_ID:
	{
		//处理电机数据宏函数
		get_gimbal_motor_measuer(&CAN_Cmd.Gimbal.Yaw_Measure, Rx_Message);
		break;
	}
	case CAN_FRIC_MOTOR_ID:
	{
		get_motor_measure(&CAN_Cmd.Fric.Fric_Measure, Rx_Message);//底盘大弹丸拨弹轮电机
		break;
	}
#endif
#ifdef useSteering
	case CAN_DJI_Motor5_ID:
	case CAN_DJI_Motor6_ID:
	case CAN_DJI_Motor7_ID:
	case CAN_DJI_Motor8_ID:
	{
		static uint8_t i = 0;
		//处理电机ID号
		i = *Rx_Message - CAN_DJI_Motor5_ID;
		//处理电机数据宏函数
		MA_get_motor_measure(CAN_Cmd.Gimbal.GetData(i), Rx_Date);
		break;
	}
#endif
	default:
	break;
	}
}

void Message_Ctrl::CAN2_Process(uint32_t *Rx_Message)
{
	uint8_t Rx_Date[8];
	for(uint8_t x=0;x<8;x++)
	{
					Rx_Date[x]=CAN2_Ctrl.read();
	}
	switch(*Rx_Message)
	{
	case CAN_DJI_Motor1_ID:
	case CAN_DJI_Motor2_ID:
	case CAN_DJI_Motor3_ID:
	case CAN_DJI_Motor4_ID:
	{
		static uint8_t i = 0;
		//处理电机ID号
		i = *Rx_Message - CAN_DJI_Motor1_ID;
		//处理电机数据宏函数
		MA_get_motor_measure(CAN_Cmd.Chassis.GetData(i), Rx_Date);
		break;
	}
	default:
	break;
	}
}

RC_ctrl_t *get_remote_control_point(void)
{
    return &Message.rc_ctrl;
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
