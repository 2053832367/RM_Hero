#include "dev_can.h"
#include "app_motor.h"
#include "Message_Task.h"
CAN_Ctrl CAN_Cmd;

void CAN1_Send(CanRxMsg *FDCAN_RxID)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	ID_Data[CanData1].Data_Ptr = FDCAN_RxID;
	xQueueSendFromISR(CAN1_Rx_Queue, &ID_Data[CanData1], &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void CAN2_Send(CanRxMsg *FDCAN_RxID)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	ID_Data[CanData2].Data_Ptr = FDCAN_RxID;
	xQueueSendFromISR(CAN2_Rx_Queue, &ID_Data[CanData2], &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void CAN3_Send(CanRxMsg *FDCAN_RxID)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	ID_Data[CanData3].Data_Ptr = FDCAN_RxID;
	xQueueSendFromISR(CAN3_Rx_Queue, &ID_Data[CanData3], &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void CAN_ALL_Init(void)
{
	CAN1_Ctrl.attachInterrupt(CAN1_Send);
	CAN2_Ctrl.attachInterrupt(CAN2_Send);
}

void CAN_Ctrl::SendData(CANctrl *CANx_Ctrl, uint32_t StdID, const void *buf, uint8_t len)
{
	CANx_Ctrl->ChangeID(StdID);
	CANx_Ctrl->SendData(buf, len);
}

void CAN_Ctrl::SendData(FDCAN_HandleTypeDef *CANx, uint32_t StdID, const void *buf, uint8_t len)
{
	if(CANx == &hfdcan1)
	{
		SendData(&CAN1_Ctrl, StdID, buf, len);
	}
	if(CANx == &hfdcan2)
	{
		SendData(&CAN2_Ctrl, StdID, buf, len);
	}
}

void CAN_Ctrl::SendData(Motor_CAN_Ctrl *Motor, int16_t Motor1, int16_t Motor2, int16_t Motor3, int16_t Motor4)
{
	FDCAN_HandleTypeDef *CANx;
	uint32_t StdID;
	uint8_t Num;
	uint8_t FDCAN_TxData[8];
	FDCAN_TxData[0] = Motor1 >> 8;
	FDCAN_TxData[1] = Motor1;
	FDCAN_TxData[2] = Motor2 >> 8;
	FDCAN_TxData[3] = Motor2;
	FDCAN_TxData[4] = Motor3 >> 8;
	FDCAN_TxData[5] = Motor3;
	FDCAN_TxData[6] = Motor4 >> 8;
	FDCAN_TxData[7] = Motor4;

	Motor->GetData(StdID, Num);
	CANx = Motor->CANx;
	SendData(CANx, StdID, FDCAN_TxData, 8);
}

void CAN_Ctrl::SendData(Motor_CAN_Ctrl *Motor, int16_t Motor1, int16_t Motor2, int16_t Motor3)
{
	FDCAN_HandleTypeDef *CANx;
	uint32_t StdID;
	uint8_t Num;
	uint8_t FDCAN_TxData[6];
	FDCAN_TxData[0] = Motor1 >> 8;
	FDCAN_TxData[1] = Motor1;
	FDCAN_TxData[2] = Motor2 >> 8;
	FDCAN_TxData[3] = Motor2;
	FDCAN_TxData[4] = Motor3 >> 8;
	FDCAN_TxData[5] = Motor3;

	Motor->GetData(StdID, Num);
	CANx = Motor->CANx;
	SendData(CANx, StdID, FDCAN_TxData, 6);
}

void CAN_Ctrl::SendData(Motor_CAN_Ctrl *Motor, int16_t Motor1, int16_t Motor2)
{
	FDCAN_HandleTypeDef *CANx;
	uint32_t StdID;
	uint8_t Num;
	uint8_t FDCAN_TxData[4];
	FDCAN_TxData[0] = Motor1 >> 8;
	FDCAN_TxData[1] = Motor1;
	FDCAN_TxData[2] = Motor2 >> 8;
	FDCAN_TxData[3] = Motor2;

	Motor->GetData(StdID, Num);
	CANx = Motor->CANx;
	SendData(CANx, StdID, FDCAN_TxData, 4);
}

void CAN_Ctrl::SendData(Motor_CAN_Ctrl *Motor, int16_t Motor1)
{
	FDCAN_HandleTypeDef *CANx;
	uint32_t StdID;
	uint8_t Num;
	uint8_t FDCAN_TxData[2];
	FDCAN_TxData[0] = Motor1 >> 8;
	FDCAN_TxData[1] = Motor1;

	Motor->GetData(StdID, Num);
	CANx = Motor->CANx;
	SendData(CANx, StdID, FDCAN_TxData, 2);
}

void CAN_Ctrl::CAN_CMD_RESET_ID(Motor_CAN_Ctrl *Motor)
{
	FDCAN_HandleTypeDef *CANx;
	uint32_t StdID;
	uint8_t Num;
	Motor->GetData(StdID, Num);
	CANx = Motor->CANx;
	StdID = 0x700;
	uint8_t Data[8];
	for(uint8_t i = 0; i < 8; i++)
	{
		Data[i] = 0;
	}
	SendData(CANx, StdID, Data, 8);
}
