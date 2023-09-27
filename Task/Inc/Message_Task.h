#ifndef Message_TASK_H
#define Message_TASK_H

#include "cmsis_os2.h"                  // ::CMSIS:RTOS2
#include "string.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "app_preference.h"
#include "dev_can.h"
#include "app_serial.h"
#include "protocol_dbus.h"
#include "protocol_judgement.h"
#include "drivers_statistic.h"

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

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;

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



class Message_Ctrl:public Statistic
{
public:
    Gimbal_Receive_Data_t GimbalR;
    supercap_Receive_Data_t SuperCapR;
	  RC_ctrl_t rc_ctrl;
    const judge_type_t *robo;

    union I ecd_data;

    void Init();

    void CAN1_Process(uint32_t *Rx_Message);
    void CAN2_Process(uint32_t *Rx_Message);
    void Serialx_Hook(uint8_t *Rx_Message, Serialctrl *Serialx_Ctrl);
    void Gimbal_Serial_Hook(uint8_t *Rx_Message);
};

Message_Ctrl *get_message_ctrl_pointer(void);
RC_ctrl_t *get_remote_control_point(void);

typedef struct
{
    uint8_t key_flag;
    uint8_t count;  //次数
    uint8_t last_count;
} count_num_key;

typedef enum
{
    single = 0,
    even,
} key_count_e;

struct rc_key_v_t
{
    //键盘
    count_num_key W;
    count_num_key S;
    count_num_key A;
    count_num_key D;
    count_num_key shift;
    count_num_key ctrl;
    count_num_key Q;
    count_num_key E;
    count_num_key R;
    count_num_key F;
    count_num_key G;
    count_num_key Z;
    count_num_key X;
    count_num_key C;
    count_num_key V;
    count_num_key B;
};

struct rc_press_t
{
    //鼠标
    count_num_key L;
    count_num_key R;
};

class rc_key_c
{
public:
    rc_key_v_t Key;
    rc_press_t Press;

    void rc_key_v_set(RC_ctrl_t *RC);
    uint8_t read_key(count_num_key *temp_count, key_count_e mode, bool clear);
    bool read_key(count_num_key *temp_count, key_count_e mode, bool *temp_bool);
    void clear_key_count(count_num_key *temp_count);

private:
    bool read_key_single(count_num_key *temp_count);
    bool read_key_single(count_num_key *temp_count, bool *temp_bool);
    bool read_key_even(count_num_key *temp_count);
    bool read_key_even(count_num_key *temp_count, bool *temp_bool);
    void sum_key_count(int16_t key_num, count_num_key *temp_count);
};

#endif

