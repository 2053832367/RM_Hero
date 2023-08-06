#include "Chassis_Task.h"

	void Chassis_Task(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {		
		
		xQueueSend(Message_Queue, &ID_Data[ChassisData], 0);

    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}
