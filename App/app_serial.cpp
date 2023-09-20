#include "app_serial.h"
#include "Message_Task.h"
#include "app_preference.h"
#include "drivers_dma.h"

int a;
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
    //	HAL_UART_Receive_IT(&huart1,&Serial1_Ctrl.receive_RXNE,1);   // 重新使能接收中断

    //		__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);  //idle interrupt
    //		HAL_UART_Receive_DMA(&huart1, (uint8_t*)&(Serial_Cmd.Serial1.Data[1]), Serial1_Buffer_Size);
    MA_UART_Receive_DMA_Init(&huart1, &hdma_usart1_rx, (uint8_t *)&(Serial_Cmd.Serial1.Data[0][1]), (uint8_t *)&(Serial_Cmd.Serial1.Data[1][1]), Serial1_Buffer_Size);

    //		__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);  //idle interrupt
    //		HAL_UART_Receive_DMA(&huart2, (uint8_t*)&(Serial_Cmd.Serial2.Data[1]), Serial2_Buffer_Size);
    MA_UART_Receive_DMA_Init(&huart2, &hdma_usart2_rx, (uint8_t *)&(Serial_Cmd.Serial2.Data[0][1]), (uint8_t *)&(Serial_Cmd.Serial2.Data[1][1]), Serial2_Buffer_Size);

    Serial1_Ctrl.attachInterrupt(Serial1_Hook);
    Serial2_Ctrl.attachInterrupt(Serial2_Hook);
    Serial4_Ctrl.attachInterrupt(Serial4_Hook);
    Serial5_Ctrl.attachInterrupt(Serial5_Hook);
}

void Serial_Ctrl::Hook(USART_TypeDef *SERIAL, bool mode)
{
    if (SERIAL == SERIAL1)
    {
        Handle(&Serial1_Ctrl, &Serial1, mode);
    }
    if (SERIAL == SERIAL2)
    {
        Handle(&Serial2_Ctrl, &Serial2, mode);
    }
    if (SERIAL == SERIAL4)
    {
        Handle(&Serial4_Ctrl, &Serial4, mode);
    }
    if (SERIAL == SERIAL5)
    {
        Handle(&Serial5_Ctrl, &Serial5, mode);
    }
}

void Serial_Ctrl::Handle(Serialctrl *SerialCtrl, Serial_Data_t *Serial, bool mode)
{
    if (mode == 1)
    {
        bool Memory;
        /* Current memory buffer used is Memory 0 */
        // disable DMA
        // 失效DMA
        __HAL_DMA_DISABLE(SerialCtrl->hdma_usart_rx);

        // get receive data length, length = set_data_length - remain_length
        // 获取接收数据长度,长度 = 设定长度 - 剩余长度
        Serial->Len = Serial->buffer_size - ((DMA_Stream_TypeDef *)SerialCtrl->hdma_usart_rx->Instance)->NDTR;

        // reset set_data_lenght
        // 重新设定数据长度
        ((DMA_Stream_TypeDef *)SerialCtrl->hdma_usart_rx->Instance)->NDTR = Serial->buffer_size;

        if ((((DMA_Stream_TypeDef *)SerialCtrl->hdma_usart_rx->Instance)->CR & DMA_SxCR_CT) == RESET)
        {
            // set memory buffer 1
            // 设定缓冲区1
            ((DMA_Stream_TypeDef *)SerialCtrl->hdma_usart_rx->Instance)->CR |= DMA_SxCR_CT;
            Memory = 0;
        }
        else
        {
            // set memory buffer 0
            // 设定缓冲区0
            ((DMA_Stream_TypeDef *)SerialCtrl->hdma_usart_rx->Instance)->CR &= ~(DMA_SxCR_CT);
            Memory = 1;
        }
				
				
		// enable DMA
        // 使能DMA
        __HAL_DMA_ENABLE(SerialCtrl->hdma_usart_rx);
				
				
        if ((Serial->Len == Serial->Lenth0 || Serial->Lenth1 || Serial->Lenth2 || Serial->Lenth3) && (Serial->Len != NULL))
        {
            Serial->Data[Memory][0] = Serial->Len;
            if (Serial->Header == NULL && Serial->Tail == NULL)
            {
                Send_to_Message(SerialCtrl, Memory);
            }
            else if (Serial->Header == Serial->Data[Memory][1] && Serial->Tail == Serial->Data[Memory][Serial->Len])
            {
                Send_to_Message(SerialCtrl, Memory);
            }
        }
				else if(SerialCtrl == &JUDGE_SERIAL && (Serial->Len != NULL))
				{
					Send_to_Message(SerialCtrl, Memory);
				}
    }
}

uint8_t Serial_Ctrl::Get_Data(Serial_Data_t *Serial, uint8_t *buf)
{
    if (Serial->Len == 0)
    {
        return 0;
    }
    buf = Serial->Data[0];
    return Serial->Len;
}

void Serial_Ctrl::Send_to_Message(Serialctrl *SerialCtrl, bool Memory)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (SerialCtrl == &Serial1_Ctrl)
    {
        ID_Data[SerialData1].Data_Ptr = Serial1.Data[Memory];
        xQueueSendFromISR(Serial_Rx_Queue, &ID_Data[SerialData1], &xHigherPriorityTaskWoken);
    }
    if (SerialCtrl == &Serial2_Ctrl)
    {
        ID_Data[SerialData2].Data_Ptr = Serial2.Data[Memory];
        xQueueSendFromISR(Serial_Rx_Queue, &ID_Data[SerialData2], &xHigherPriorityTaskWoken);
    }
    if (SerialCtrl == &Serial4_Ctrl)
    {
        ID_Data[SerialData4].Data_Ptr = Serial4.Data[Memory];
        xQueueSendFromISR(Serial_Rx_Queue, &ID_Data[SerialData4], &xHigherPriorityTaskWoken);
    }
    if (SerialCtrl == &Serial5_Ctrl)
    {
        ID_Data[SerialData5].Data_Ptr = Serial5.Data[Memory];
        xQueueSendFromISR(Serial_Rx_Queue, &ID_Data[SerialData5], &xHigherPriorityTaskWoken);
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
