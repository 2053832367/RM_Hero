#ifndef Correspo_TASK_H
#define Correspo_TASK_H

#include "cmsis_os2.h"                  // ::CMSIS:RTOS2
#include "string.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "app_preference.h"
#include "app_serial.h"
#include "drivers_statistic.h"
#include "protocol_judgement.h"
#include "protocol_crc.h"


#ifdef __cplusplus
extern "C" {
#endif

    void Correspond_Task(void *pvParameters);

#ifdef __cplusplus
}
#endif

extern QueueHandle_t Message_Queue;

union F
{
    uint8_t I[4];
    float F;
};


struct Cap_Data_t
{
    uint8_t enable;
    uint8_t mode;
    uint8_t power;
    uint8_t power_limit;
};

#pragma pack(1)
struct game_robot_state_t
{
    game_robot_state_t():Header(0xff), Mode(game_robot_state_id)
    {}

    uint8_t Header;
    uint8_t Mode;
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t remain_HP;
    float bullet_speed;
    uint16_t bullet_speed_limit;
    uint8_t CRC8;
};
#pragma pack()

#pragma pack(1)
struct game_robot_HP_t
{
    game_robot_HP_t():Header(0xff), Mode(game_robot_HP_id)
    {}

    uint8_t Header;
    uint8_t Mode;
    ext_game_robot_HP_t ext_game_robot_HP;
    uint8_t CRC8;
};
#pragma pack()

#pragma pack(1)
struct game_status_t
{
    game_status_t():Header(0xff), Mode(game_status_id)
    {}

    uint8_t Header;
    uint8_t Mode;
		uint8_t game_progress; 	//比赛当前状态
    uint8_t CRC8;
};
#pragma pack()

#pragma pack(1)
struct Gimbal_Send_Data_t
{
	Gimbal_Send_Data_t():Header(0xff), Mode(Gimbal_Data_id)
  {}
		
	uint8_t Header;
	uint8_t Mode;
	uint8_t mode;
	uint8_t supercap;
	uint8_t CRC8;
};
#pragma pack()

class correspondence_ctrl:public Statistic
{
public:
    correspondence_ctrl()
    {}

    void Corres_Init(void);
    void Corres_Send(void);
    void Corres_Feedback(void);
private:
    Cap_Data_t SuperCapS;
		Gimbal_Send_Data_t GimbalS;

    game_robot_state_t game_robot_state;
    game_robot_HP_t game_robot_HP;
		game_status_t game_status;

};

#endif

