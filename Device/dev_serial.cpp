#include "dev_serial.h"
#include "app_preference.h"

Serialctrl Serial1_Ctrl(&huart1, Serial1_Buffer_Size);
Serialctrl Serial2_Ctrl(&huart2, Serial2_Buffer_Size);
Serialctrl Serial4_Ctrl(&huart4, Serial4_Buffer_Size);
Serialctrl Serial5_Ctrl(&huart5, Serial5_Buffer_Size);

Serialctrl::Serialctrl(UART_HandleTypeDef *_huartx, uint32_t BufferSize)
{
    this->huartx = _huartx;
    USART_Function = 0;
    newBuffer(&_rx_buffer, BufferSize);
}

void Serialctrl::attachInterrupt(USART_CallbackFunction_t Function)
{
    USART_Function = Function;
}

//void Serialctrl::IRQHandler(void)
//{
//    if(USART_GetITStatus(USARTx, USART_IT_RXNE) != RESET)
//    {
//        uint8_t c = USART_ReceiveData(USARTx);
//        Buffer_Write(&_rx_buffer, c);
//        if(USART_Function)
//        {
//            USART_Function(0);
//        }
//        USART_ClearITPendingBit(USARTx, USART_IT_RXNE);
//    }

//    if(USART_GetITStatus(USARTx, USART_IT_IDLE) != RESET)
//    {
//        uint8_t c = USART_ReceiveData(USARTx);
//        if(USART_Function)
//        {
//            USART_Function(1);
//        }
//        USART_ClearITPendingBit(USARTx, USART_IT_IDLE);
//    }
//}

void Serialctrl::IRQHandler_RXNE(uint8_t c)
{
        Buffer_Write(&_rx_buffer, c);
        if(USART_Function)
        {
            USART_Function(0);
        }
}

void Serialctrl::IRQHandler_IDLE(void)
{
        if(USART_Function)
        {
            USART_Function(1);
        }
}

void Serialctrl::sendData(uint8_t ch)
{
//    USART_SendData(this->USARTx, ch);
//    while(USART_GetFlagStatus(this->USARTx, USART_FLAG_TXE) == RESET);
	HAL_UART_Transmit(this->huartx, &ch, 1, 99);

}

void Serialctrl::sendData(const void *str)
{
    unsigned int k = 0;
    do
    {
        sendData(*((uint8_t *)str + k));
        k++;
    } while(*((uint8_t *)str + k) != '\0');
//    while(USART_GetFlagStatus(this->USARTx, USART_FLAG_TC) == RESET) {}
}

void Serialctrl::sendData(const void *buf, uint8_t len)
{
    uint8_t *ch = (uint8_t *)buf;
    while(len--)
    {
        sendData(*ch++);
    }
}

int Serialctrl::available(void)
{
    return ((unsigned int)(_rx_buffer.buf_size + _rx_buffer.pw - _rx_buffer.pr)) % _rx_buffer.buf_size;
}

uint8_t Serialctrl::read(void)
{
    uint8_t c = 0;
    Buffer_Read(&_rx_buffer, &c);
    return c;
}

int Serialctrl::peek(void)
{
    if(_rx_buffer.pr == _rx_buffer.pw)
    {
        return -1;
    }
    else
    {
        return _rx_buffer.fifo[_rx_buffer.pr];
    }
}

void Serialctrl::flush(void)
{
    _rx_buffer.pr = _rx_buffer.pw;
}

extern "C"{

//串口接收中断回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1){
        
		Serial1_Ctrl.IRQHandler_RXNE(Serial1_Ctrl.receive_RXNE);
    HAL_UART_Receive_IT(&huart1, &Serial1_Ctrl.receive_RXNE, 1);    // 继续使能RX中断
        
    }
}
//串口接收空闲中断回调函数，这个是从hal里面新加的
void HAL_UART_IdleCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1){  
        
    Serial1_Ctrl.receive_IDLE = huart1.Instance->RDR;
		Serial1_Ctrl.IRQHandler_IDLE(); 
    }
}

//串口接收错误中断
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if(HAL_UART_GetError(huart) & HAL_UART_ERROR_PE){		/*!< Parity error            */
		//奇偶校验错误
		__HAL_UART_CLEAR_PEFLAG(huart);
	}else if(HAL_UART_GetError(huart) & HAL_UART_ERROR_NE){ /*!< Noise error             */
		//噪声错误
		__HAL_UART_CLEAR_NEFLAG(huart);
	}else if(HAL_UART_GetError(huart) & HAL_UART_ERROR_FE){ /*!< Frame error             */
		//帧格式错误
		__HAL_UART_CLEAR_FEFLAG(huart);
	}else if(HAL_UART_GetError(huart) & HAL_UART_ERROR_ORE){ /*!< Overrun error           */
		//数据太多串口来不及接收错误
		__HAL_UART_CLEAR_OREFLAG(huart);
	}
    //当这个串口发生了错误，一定要在重新使能接收中断
    if(huart ->Instance == USART1){
		HAL_UART_Receive_IT(&huart1, &Serial1_Ctrl.receive_RXNE, 1);
	}
    //其他串口......
}

}
