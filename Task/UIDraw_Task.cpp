#include "UIDraw_Task.h"


void UIDraw_Task(void *pvParameters)
{
/* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {		
		        xQueueSend(Message_Queue, &ID_Data[UIdrawData], 0);
    osDelay(2);
  }
  /* USER CODE END StartDefaultTask */  
}

