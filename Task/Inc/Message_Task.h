#ifndef Message_TASK_H
#define Message_TASK_H

#include "cmsis_os2.h"                  // ::CMSIS:RTOS2
#include "string.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "app_preference.h"
#include "app_serial.h"


#ifdef __cplusplus
extern "C" {
#endif

    void Message_Task(void *pvParameters);
    void CAN1_Rx_Task(void *pvParameters);
    void CAN2_Rx_Task(void *pvParameters);
    void Serial_Rx_Task(void *pvParameters);
    void Referee_Rx_Task(void *pvParameters);
    void DR16_Rx_Task(void *pvParameters);

#ifdef __cplusplus
}
#endif

extern QueueHandle_t Message_Queue;
extern QueueHandle_t CAN1_Rx_Queue;
extern QueueHandle_t CAN2_Rx_Queue;
extern QueueHandle_t Serial_Rx_Queue;
extern QueueHandle_t Referee_Rx_Queue;
extern QueueHandle_t DR16_Rx_Queue;

union I
{
    char s[2];
    uint16_t d;
};

#pragma pack(1)
struct Gimbal_Receive_Data_t
{
    Gimbal_Receive_Data_t():Header(0xff), Mode(Gimbal_Data_id)
    {}
    uint8_t Header;
    uint8_t Mode;
    int16_t ECD;
    uint8_t goal;
    uint8_t CRC8;
};
#pragma pack()

struct supercap_Receive_Data_t
{
    uint8_t situation;  //0x00自检 0x0f关闭 0xf0开启 0xff错误
    uint8_t mode;       //0x00模式1 0xff模式2
    float power;
    uint8_t energy;
    uint8_t power_limit;
    uint8_t errorcode;

    uint8_t *ptr;
};



class Message_Ctrl
{
public:
    Gimbal_Receive_Data_t GimbalR;
    supercap_Receive_Data_t SuperCapR;
//    const judge_type_t *robo;

    union I ecd_data;

    void Init();

//    void CAN1_Process(CanRxMsg *Rx_Message);
//    void CAN2_Process(CanRxMsg *Rx_Message);
    void Serialx_Hook(uint8_t *Rx_Message, Serialctrl *Serialx_Ctrl);
    void Gimbal_Serial_Hook(uint8_t *Rx_Message);
};

Message_Ctrl *get_message_ctrl_pointer(void);

#endif

