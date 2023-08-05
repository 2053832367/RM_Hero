#include "app_serial.h"
#include "Message_Task.h"
#include "app_preference.h"

Serial_Ctrl Serial_Cmd;

void Serial1_Hook(bool mode)
{
    Serial_Cmd.Hook(SERIAL1, mode);
}

void Serial2_Hook(bool mode)
{
    Serial_Cmd.Hook(SERIAL2, mode);
}

void Serial4_Hook(bool mode)
{
    Serial_Cmd.Hook(SERIAL4, mode);
}

void Serial5_Hook(bool mode)
{
    Serial_Cmd.Hook(SERIAL5, mode);
}

void Serial_ALL_Init(void)
{
	HAL_UART_Receive_IT(&huart1,&Serial1_Ctrl.receive_RXNE,1);   // 重新使能接收中断
	
    Serial1_Ctrl.attachInterrupt(Serial1_Hook);
    Serial2_Ctrl.attachInterrupt(Serial2_Hook);
    Serial4_Ctrl.attachInterrupt(Serial4_Hook);
    Serial5_Ctrl.attachInterrupt(Serial5_Hook);
}

Serialctrl *Serial_Ctrl::Tran(USART_TypeDef *SERIAL)
{
    if(SERIAL == SERIAL1)
    {
        return &Serial1_Ctrl;
    }
    if(SERIAL == SERIAL2)
    {
        return &Serial2_Ctrl;
    }
    if(SERIAL == SERIAL4)
    {
        return &Serial4_Ctrl;
    }
    if(SERIAL == SERIAL5)
    {
        return &Serial5_Ctrl;
    }
    return NULL;
}

void Serial_Ctrl::Hook(USART_TypeDef *SERIAL, bool mode)
{
    if(SERIAL == SERIAL1)
    {
        Handle(&Serial1_Ctrl, &Serial1, mode);
    }
    if(SERIAL == SERIAL2)
    {
        Handle(&Serial2_Ctrl, &Serial2, mode);
    }
    if(SERIAL == SERIAL4)
    {
        Handle(&Serial4_Ctrl, &Serial4, mode);
    }
    if(SERIAL == SERIAL5)
    {
        Handle(&Serial5_Ctrl, &Serial5, mode);
    }
}

void Serial_Ctrl::Handle(Serialctrl *SerialCtrl, Serial_Data_t *Serial, bool mode)
{
    if(Serial->buffer_size == 0)
    {
        Send_to_Message(SerialCtrl);
        return;
    }
    if(mode == 0)
    {
        if(Serial->Header == NULL || Serial->Lenth == NULL)
        {
            return;
        }
        Serial->Temp = SerialCtrl->peek();
        if(Serial->Temp != Serial->Header)
        {
            SerialCtrl->read();
            return;
        }
        Serial->Len = SerialCtrl->available();
        if(Serial->Len < Serial->Lenth || Serial->Lenth == NULL)
        {
            return;
        }
        Serial->Data[0] = Serial->Lenth;
        for(uint8_t i = 0; i < Serial->Len; i++)
        {
            Serial->Data[i + 1] = SerialCtrl->read();
        }
        if(Serial->Tail == NULL || (Serial->Tail != NULL && Serial->Data[Serial->Lenth] == Serial->Tail))
        {
            Send_to_Message(SerialCtrl);
        }
    }
    else if(mode == 1)
    {
        Serial->Len = SerialCtrl->available();
        Serial->Data[0] = Serial->Len;
        for(uint8_t i = 0; i < Serial->Len; i++)
        {
            Serial->Data[i + 1] = SerialCtrl->read();
        }
				Serial->Len = SerialCtrl->available();
        if(Serial->Lenth != NULL && Serial->Lenth != Serial->Len)
        {
            Serial->Data[0] = 0;
        }
        if(Serial->Tail != NULL && Serial->Data[Serial->Len] != Serial->Tail)
        {
            Serial->Data[0] = 0;
        }
        if(Serial->Data[0] != 0)
        {
            Send_to_Message(SerialCtrl);
        }
    }
}

uint8_t Serial_Ctrl::Get_Data(Serial_Data_t *Serial, uint8_t *buf)
{
    if(Serial->Len == 0)
    {
        return 0;
    }
    buf = Serial->Data;
    return Serial->Len;
}

void Serial_Ctrl::Send_to_Message(Serialctrl *SerialCtrl)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if(SerialCtrl == &Serial1_Ctrl)
    {
        ID_Data[SerialData1].Data_Ptr = Serial1.Data;
        xQueueSendFromISR(Serial_Rx_Queue, &ID_Data[SerialData1], &xHigherPriorityTaskWoken);
    }
    if(SerialCtrl == &Serial2_Ctrl)
    {
        ID_Data[SerialData2].Data_Ptr = Serial2.Data;
        xQueueSendFromISR(Serial_Rx_Queue, &ID_Data[SerialData2], &xHigherPriorityTaskWoken);
    }
    if(SerialCtrl == &Serial4_Ctrl)
    {
        ID_Data[SerialData4].Data_Ptr = Serial4.Data;
        xQueueSendFromISR(Serial_Rx_Queue, &ID_Data[SerialData4], &xHigherPriorityTaskWoken);
    }
    if(SerialCtrl == &Serial5_Ctrl)
    {
        ID_Data[SerialData5].Data_Ptr = Serial5.Data;
        xQueueSendFromISR(Serial_Rx_Queue, &ID_Data[SerialData5], &xHigherPriorityTaskWoken);
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void Serial_Ctrl::SendData(USART_TypeDef *SERIAL, const void *buf, uint8_t len)
{
    Serialctrl *Serial = Tran(SERIAL);
    SendData(Serial, buf, len);
}

void Serial_Ctrl::SendData(USART_TypeDef *SERIAL, uint8_t ch)
{
    Serialctrl *Serial = Tran(SERIAL);
    SendData(Serial, ch);
}

void Serial_Ctrl::SendData(USART_TypeDef *SERIAL, const void *str)
{
    Serialctrl *Serial = Tran(SERIAL);
    SendData(Serial, str);
}

void Serial_Ctrl::SendData(Serialctrl *Serial, const void *buf, uint8_t len)
{
    Serial->sendData(buf, len);
}

void Serial_Ctrl::SendData(Serialctrl *Serial, uint8_t ch)
{
    Serial->sendData(ch);
}

void Serial_Ctrl::SendData(Serialctrl *Serial, const void *str)
{
    Serial->sendData(str);
}
