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
#include "drivers_statistic.h"
#include "protocol_crc.h"

#include "algorithm_kalman.h"


#ifdef __cplusplus
extern "C" {
#endif

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

#define Correspondence_Task_Control_Time 2

#define Gimbal_visual_offset_ecd 7360

/*
使用如下格式来以一字节对齐结构体
#pragma pack(1)
struct
{

};
#pragma pack()
*/

#pragma pack(1)
struct Visual_Posture_Data_t
{
    Visual_Posture_Data_t():Header(0xff), Mode(visual_posture_id) {}
    uint8_t Header;
    uint8_t Mode;
    uint8_t Yaw[4];
    uint8_t Pitch[4];
    uint8_t Shoot[4];
		uint8_t Color;
		uint8_t Progress;
    uint8_t CRC8;
};
#pragma pack()

#pragma pack(1)
struct Visual_Mode_Data_t
{
    Visual_Mode_Data_t():Header(0xff), Mode(visual_mode_id) {}
    uint8_t Header;
    uint8_t Mode;
	uint8_t mode;
    uint8_t CRC8;
};
#pragma pack()

#pragma pack(1)
struct Chassis_Send_Data_t
{
    Chassis_Send_Data_t():Header(0xff), Mode(chassis_data_id) {}
    uint8_t Header;
    uint8_t Mode;
    int16_t ECD;
    uint8_t goal;
    uint8_t CRC8;
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

struct Correspondence_Data_t
{
    union I int_data;
    union F Yaw_Unoin;
    union F Pitch_Unoin;
    union F Shoot_Unoin;

    fp32 Yaw_error;
		fp32 Yaw_visual_angle;
};



class correspondence_ctrl:public Statistic
{
public:
    correspondence_ctrl()
        :P_data(2.f, 0.f, 0.f, 2.f), A_data(1.f, 0.002f, 0.f, 1.f),
        H_data(1.f, 0.f, 0.f, 1.f), Q_data(1.f, 0.f, 0.f, 1.f),
        R_data(200.f, 0.f, 0.f, 400.f),
        Visual_Yaw_Init_Matrix(P_data, A_data, H_data, Q_data, R_data),
        Visual_Pitch_Init_Matrix(P_data, A_data, H_data, Q_data, R_data)
    {}

    Correspondence_Data_t Data;

    Visual_Posture_Data_t Visual_Posture;
    game_robot_state_t_ Game_Sate;
    Visual_Mode_Data_t Visual_Mode;
    Game_HP_t Game_HP;
    Chassis_Send_Data_t ChassisS;

    void Corres_Init(void);
    void Corres_Send(void);
    void Corres_Feedback(void);
    void Corres_Calc(void);
			
	void RGB_Send(void);
private:

    Matrix P_data;
    Matrix A_data;
    Matrix H_data;
    Matrix Q_data;
    Matrix R_data;

    kalman_filter_t Visual_Yaw_Temp;
    kalman_filter_init_t Visual_Yaw_Init;
    kalman_filter_init_t_matrix Visual_Yaw_Init_Matrix;
    kalman_filter_t Visual_Pitch_Temp;
    kalman_filter_init_t_matrix Visual_Pitch_Init_Matrix;
};

#endif

