#ifndef __APP_SERIAL_H
#define __APP_SERIAL_H

#ifdef __cplusplus
extern "C" {
#endif
	
#include <stdint.h>	
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "app_preference.h"
#include "dev_serial.h"

	
#ifdef __cplusplus
}
#endif

#define SERIAL1 USART1
#define SERIAL2 USART2
#define SERIAL4 UART4
#define SERIAL5 UART5

typedef enum
{
    game_robot_state_id = 0x01,
    game_robot_HP_id = 0x02,
		game_status_id = 0x03,

    Gimbal_Data_id = 0x51,
}serial_msg_mode_e;

struct Serial_Data_t
{
    uint8_t Header;
    uint8_t Tail;
    uint8_t Lenth0;
    uint8_t Lenth1;
    uint8_t Lenth2;
    uint8_t Lenth3;
		uint8_t buffer_size;
    uint8_t Len;
    uint8_t Temp;
    uint8_t **Data;
    Serial_Data_t(uint8_t Header_, uint8_t Tail_, uint8_t Lenth0_,uint8_t Lenth1_,uint8_t Lenth2_,uint8_t Lenth3_, uint8_t buffer_size_)
        :Header(Header_), Tail(Tail_), Lenth0(Lenth0_),Lenth1(Lenth1_),Lenth2(Lenth2_),Lenth3(Lenth3_), buffer_size(buffer_size_)
    {
        Data = new uint8_t*[2];
				Data[0] = new uint8_t[buffer_size_];
				Data[1] = new uint8_t[buffer_size_];
    };
};

class Serial_Ctrl
{
public:
    Serial_Ctrl()
        :Serial1(Serial1_Data_Header, Serial1_Data_Tail, Serial1_Data_Lenth0, Serial1_Data_Lenth1, Serial1_Data_Lenth2, Serial1_Data_Lenth3, Serial1_Buffer_Size),
        Serial2(Serial2_Data_Header, Serial2_Data_Tail, Serial2_Data_Lenth0, Serial2_Data_Lenth1, Serial2_Data_Lenth2, Serial2_Data_Lenth3, Serial2_Buffer_Size),
        Serial4(Serial4_Data_Header, Serial4_Data_Tail, Serial4_Data_Lenth0, Serial4_Data_Lenth1, Serial4_Data_Lenth2, Serial4_Data_Lenth3, Serial4_Buffer_Size),
        Serial5(Serial5_Data_Header, Serial5_Data_Tail, Serial5_Data_Lenth0, Serial5_Data_Lenth1, Serial5_Data_Lenth2, Serial5_Data_Lenth3, Serial5_Buffer_Size)
    {}

    void Hook(USART_TypeDef *SERIAL, bool mode);
    void Handle(Serialctrl *Serial, Serial_Data_t *Usart, bool mode);
    void Send_to_Message(Serialctrl *SerialCtrl , bool Memory);

    Serialctrl *Tran(USART_TypeDef *SERIAL);

    uint8_t Get_Data(Serial_Data_t *Serial, uint8_t *buf);

    void SendData(USART_TypeDef *SERIAL, uint8_t ch);
    void SendData(USART_TypeDef *SERIAL, const void *str);
    void SendData(USART_TypeDef *SERIAL, const void *buf, uint8_t len);
    void SendData(Serialctrl *Serial, uint8_t ch);
    void SendData(Serialctrl *Serial, const void *str);
    void SendData(Serialctrl *Serial, const void *buf, uint8_t len);

    ~Serial_Ctrl() {}

    Serial_Data_t Serial1;
    Serial_Data_t Serial2;
    Serial_Data_t Serial4;
    Serial_Data_t Serial5;
			
private:
    void *buf;
};

extern void Serial_ALL_Init();
#endif
