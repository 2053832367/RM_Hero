#ifndef UIDraw_TASK_H
#define UIDraw_TASK_H

#include "cmsis_os2.h"                  // ::CMSIS:RTOS2
#include "string.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "app_preference.h"
#include "protocol_ui.h"
#include "drivers_statistic.h"
#include "priority_ui.h"

#ifdef __cplusplus
extern "C" {
#endif

  void UIDraw_Task(void *pvParameters);
typedef enum{
		RP_1,
		RP_2,
		RP_3,
		RP_4,
		RP_5,
		RP_6,
		RP_7,
		RP_8,
		RP_9,
		VISION_CYCLE_1,
		VISION_CYCLE_2,
		TOP_MODE_CYCLE_1,
		TOP_MODE_CYCLE_2,
		NUM,
	
		
	DYNAMIC_UI_NUM,
}dynamic_ui_cnt_e;

extern void My_Ui_Init(void);
extern void Ui_Info_Update(void);



#ifdef __cplusplus
}
#endif






extern QueueHandle_t Message_Queue;

#endif

