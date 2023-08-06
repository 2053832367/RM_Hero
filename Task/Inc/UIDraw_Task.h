#ifndef UIDraw_TASK_H
#define UIDraw_TASK_H

#include "cmsis_os2.h"                  // ::CMSIS:RTOS2
#include "string.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "app_preference.h"

#ifdef __cplusplus
extern "C" {
#endif

    void UIDraw_Task(void *pvParameters);

#ifdef __cplusplus
}
#endif

extern QueueHandle_t Message_Queue;


#endif
