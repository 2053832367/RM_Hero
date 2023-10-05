#ifndef __TASKS_H
#define __TASKS_H

#ifdef __cplusplus
extern "C" {
#endif
	
#define useFreeRTOS
//	
//#ifndef useFreeRTOS
//#define osDelay( ms )   delay_ms( ms )
//#define Delay_ms( ms )  delay_ms( ms )
//#define Delay_us( us )  delay_us( us )
//#else
//#define osDelay( ms )   vTaskDelay( pdMS_TO_TICKS( ms ) )
//#define Delay_ms( ms )  delay_xms( ms )
//#define Delay_us( us )  delay_xus( us )
//#endif
	
#ifdef __cplusplus
}
#endif

#include "Chassis_Task.h"
#include "Message_Task.h"
#include "Guard_Task.h"
#include "Correspond_Task.h"
#include "UIDraw_Task.h"

//#include "app_led.h"
//#include "app_motor.h"
//#include "app_power_ctrl.h"
//#include "app_serial.h"
//#include "app_preference.h"

extern Chassis_Ctrl Chassis;
extern Message_Ctrl Message;
extern Guard_Ctrl Guard;
extern UI_Draw_Ctrl UIDraw;
extern correspondence_ctrl Corres;

//extern correspondence_ctrl Corres;
//extern UI_Draw_Ctrl UIDraw;

//extern LEDctrl LED;
extern CAN_Ctrl CAN_Cmd;
extern Serial_Ctrl Serial_Cmd;

#endif
