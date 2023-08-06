#include "Message_Task.h"
#include "tasks.h"

Message_Ctrl Message;
		uint8_t r[10];
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
  /* Infinite loop */
  for(;;)
  {		
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

void CAN2_Rx_Task(void *pvParameters)
{
	/* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {		
    osDelay(1);
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
//			Guard.Feed(Serial_Rx_Data.Data_ID);
		}
  }
  /* USER CODE END StartDefaultTask */
}

void Referee_Rx_Task(void *pvParameters)
{
	/* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {		
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

void DR16_Rx_Task(void *pvParameters)
{
	/* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {		
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

void Message_Ctrl::Init()
{
	Serial_ALL_Init();
	Prefence_Init();
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
}

void Message_Ctrl::Gimbal_Serial_Hook(uint8_t *Rx_Message)
{
//	uint8_t len = Rx_Message[0];
//	if(Verify_CRC8_Check_Sum(&Rx_Message[1], Rx_Message[0]))
//	{
//		ecd_data.s[0] = Rx_Message[3];
//		ecd_data.s[1] = Rx_Message[4];
//		GimbalR.ECD = -motor_ecd_to_relative_ecd(ecd_data.d, Gimbal_Motor_Yaw_Offset_ECD);
//		GimbalR.goal = Rx_Message[5];
//	}
	for(int i = 0;i < 10;i++)
	{
		r[i]=Rx_Message[i];
	}
}
