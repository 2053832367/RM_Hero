#ifndef __DEVICE_CAN_H
#define __DEVICE_CAN_H

#ifdef __cplusplus
extern "C" {
#endif



#ifdef __cplusplus
}
#endif

#include "dev_system.h"
#include "drivers_buffer.h"
#include "fdcan.h"
#include "stm32h7xx_hal.h"

typedef void(*CAN_CallbackFunction_t)(uint32_t *FDCAN_RxID);

class CANctrl: public Buffer
{
public:
	CANctrl(FDCAN_HandleTypeDef *CANx, uint32_t BufferSize);

	void attachInterrupt(CAN_CallbackFunction_t Function);

	void ChangeID(uint16_t StdID);

	void SendData(const void *buf, uint8_t len);

	void IRQHandler(FDCAN_HandleTypeDef *hfdcan , uint32_t RxFifo0ITs);
		
	FDCAN_RxHeaderTypeDef FDCAN_RxHeader;
	uint32_t FDCAN_RxID;
	uint8_t FDCAN_RxData[8];
		
	FDCAN_TxHeaderTypeDef FDCAN_TxHeader;

	uint8_t read(void);
  RingBuffer _rx_buffer;
		
protected:

	FDCAN_HandleTypeDef *CANx;
	CAN_CallbackFunction_t CAN_Function;
private:
	uint32_t StdId;
};

extern void CAN_ALL_Init();

extern CANctrl CAN1_Ctrl;
extern CANctrl CAN2_Ctrl;

#endif /* __DEVICE_CAN_H */
