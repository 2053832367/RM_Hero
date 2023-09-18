#ifndef __DEVICE_CAN_H
#define __DEVICE_CAN_H

#ifdef __cplusplus
extern "C" {
#endif



#ifdef __cplusplus
}
#endif

#include "dev_system.h"
//#include "drivers_can.h"

//typedef void(*CAN_CallbackFunction_t)(CanRxMsg *RxMessage);

class CANctrl
{
//public:
//	CANctrl(CAN_TypeDef *CANx){}

//	void attachInterrupt(CAN_CallbackFunction_t Function);

//	void ChangeID(uint16_t StdID);

//	void SendData(const void *buf, uint8_t len);

//	void IRQHandler(void);

//	CanRxMsg Rx_Message;
//protected:

//	CAN_TypeDef *CANx;
//	CAN_CallbackFunction_t CAN_Function;
//private:
//	uint32_t StdId;
};

extern void CAN_ALL_Init();

extern CANctrl CAN1_Ctrl;
extern CANctrl CAN2_Ctrl;

#endif /* __DEVICE_CAN_H */
