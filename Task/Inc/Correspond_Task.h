#ifndef Correspo_TASK_H
#define Correspo_TASK_H

#include "cmsis_os2.h"                  // ::CMSIS:RTOS2
#include "string.h"
#include "FreeRTOS.h"
#include "Message_Task.h"
#include "queue.h"
#include "app_preference.h"
#include "app_serial.h"
#include "app_rgb.h"
#include "app_oled.h"
#include "app_vofa.h"

#include "drivers_statistic.h"
#include "protocol_crc.h"

#include "algorithm_kalman.h"


#ifdef __cplusplus
extern "C" {
#endif

		#include "algorithm_Solve_Trajectory.h"
	
    void Correspond_Task(void *pvParameters);
	
#ifdef __cplusplus
}
#endif

extern QueueHandle_t Message_Queue;

union I
{
    uint8_t s[2];
    uint16_t d;
};

#define Correspondence_Task_Control_Time 1

/*
使用如下格式来以一字节对齐结构体
#pragma pack(1)
struct
{

};
#pragma pack()
*/

#pragma pack(1)
struct Cap_Data_t
{
    uint8_t enable;
    uint8_t mode;
    uint8_t power;
    uint8_t power_limit;
};
#pragma pack()



#pragma pack(1)
struct game_robot_state_t_
{
    game_robot_state_t_():Header(0xff), Mode(game_robot_state_id) {}
    uint8_t Header;
    uint8_t Mode;
    uint8_t robot_id;
    uint8_t CRC8;
};
#pragma pack()

#pragma pack(1)
struct Game_HP_t
{
    Game_HP_t():Header(0xff), Mode(game_robot_HP_id) {}
    uint8_t Header;
    uint8_t Mode;
    ext_game_robot_HP_t game_robot_HP;
    uint8_t CRC8;
};
#pragma pack()


/*视觉发送数据结构体*/
#pragma pack(1)
struct Visual_Send_Data_t
{
	Visual_Send_Data_t():header(0xff),Tail(0x0d) {}
	uint8_t header;
	uint8_t mode;
	float   roll;
	float   pitch;
	float   yaw;
  float shoot_speed;
	float baoliu1;
	float baoliu2;
	float baoliu3;
	uint8_t check_byte;
	uint8_t Tail; 
	
};
#pragma pack()

class Correspondence_ctrl:public Statistic
{
public:

	  Visual_Send_Data_t        VisualS;
    game_robot_state_t_ 			Game_Sate;
    Game_HP_t 								Game_HP;
	  Cap_Data_t           			SuperCapS;
    void Corres_Init(void);
    void Corres_Send(void);
    void Corres_Feedback(void);
    void Corres_Calc(void);
			
	  void RGB_Send(void);
private:


};

#endif

